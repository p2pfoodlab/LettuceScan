#include <boost/make_shared.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_representation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/fast_bilateral.h>
#include <pcl/surface/mls.h>

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <stdlib.h>
#include <iostream>
#include <sstream>
 
using pcl::visualization::PointCloudColorHandlerGenericField;
using pcl::visualization::PointCloudColorHandlerCustom;

using namespace std;

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef PointCloud::Ptr PointCloudPtr;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

double FOCAL = 570.3;
double MM_PER_M = 1000;
int WIDTH = 240;
int HEIGHT = 320;
double DEPTH_THRESHOLD = 400;   
double X_THRESHOLD = 1000;   
double Y_THRESHOLD = 1000;   
const float normal_estimation_search_radius = 1.0f;
pcl::visualization::PCLVisualizer *p;
int vp_1, vp_2;

struct PCD
{
  PointCloud::Ptr cloud;
  std::string f_name;

  PCD() : cloud (new PointCloud) {};
};


void downsample (PointCloudPtr &points, float leaf_size, PointCloudPtr &downsampled_out)
{
  pcl::VoxelGrid<PointT> vox_grid;
  vox_grid.setLeafSize (leaf_size, leaf_size, leaf_size);
  vox_grid.setInputCloud (points);
  vox_grid.filter (*downsampled_out);
}

void bilfil (PointCloud& pcloud, PointCloud &output)
{
   pcl::FastBilateralFilter<PointT> fbFilter;
   fbFilter.setInputCloud(pcloud.makeShared());
       
   fbFilter.setSigmaR(25.0f);
   fbFilter.setSigmaS(2.f);
   fbFilter.applyFilter(output);
}

void loadPCD (string path, int k, vector<PCD, Eigen::aligned_allocator<PCD> > &data)
{
      PCD m;      
      ostringstream pcdfile;
      //pcdfile<<path<<k<<"_mls.pcd";
      pcdfile<<path<<k<<".pcd";
      //cout<<pcdfile.str()<<endl;
      m.f_name = pcdfile.str();
      pcl::io::loadPCDFile<PointT> (pcdfile.str(), *m.cloud);
      data.push_back (m);
}


void loadTransform(string trfile, Eigen::Matrix4f &transform)
{
   ifstream rfile;
   rfile.open (trfile.c_str (), std::ios::binary);

   for (int i = 0; i < 4; ++i)
   {
    for (int j = 0; j < 4; ++j)
    {
     rfile.read (reinterpret_cast<char*>(&transform (i, j)), sizeof (float));
    }
   }
}

void mls_smoothing(PointCloud& pcloud, PointCloudWithNormals& mls_points)
{
   float search_radius=2;
   float sqr_gauss_param=4;
   bool use_polynomial_fit=false;
   int polynomial_order=2;
   
   pcl::MovingLeastSquares<PointT, PointNormalT> mls;
   mls.setInputCloud (pcloud.makeShared());
   mls.setSearchRadius (search_radius);
   mls.setSqrGaussParam (sqr_gauss_param);
   mls.setPolynomialFit (use_polynomial_fit);
   mls.setPolynomialOrder (polynomial_order);
   
   mls.setUpsamplingMethod (pcl::MovingLeastSquares<PointT, PointNormalT>::NONE);
   mls.setPointDensity (60000 * int (search_radius)); // 300 points in a 5 cm radius
   mls.setUpsamplingRadius (0.025);
   mls.setUpsamplingStepSize (0.015);
   mls.setDilationIterations (2);
   mls.setDilationVoxelSize (0.01f);
   
   pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
   mls.setSearchMethod (tree);
   mls.setComputeNormals (true);
   mls.process (mls_points);
}


void showClouds(const PointCloud::Ptr cloud_target, const PointCloud::Ptr cloud_source, int vp, string id1, string id2)
{
  PointCloudColorHandlerCustom<PointT> tgt_h (cloud_target, 0, 255, 0);
  PointCloudColorHandlerCustom<PointT> src_h (cloud_source, 255, 0, 0);
  p->addPointCloud (cloud_target, tgt_h, id1, vp);
  p->addPointCloud (cloud_source, src_h, id2, vp);
}

void filtersor(PointCloud& pcloud, int mk,float th)
{
   pcl::StatisticalOutlierRemoval<PointT> sor;
   sor.setInputCloud (pcloud.makeShared());
   sor.setMeanK (mk);
   sor.setStddevMulThresh (th);
   sor.filter (pcloud);
   std::vector<int> indices;
   pcl::removeNaNFromPointCloud(pcloud, pcloud, indices);   
}


int main (int argc, char** argv)
{
   int i0=atoi(argv[2]);
   int N=atoi(argv[3]);
   int s=atoi(argv[4]);

   bool display=true;
   if (display){
      p = new pcl::visualization::PCLVisualizer (argc, argv, "Pairwise Incremental Registration example");

      p->createViewPort (0.0, 0, 0.5, 1.0, vp_1);
      p->createViewPort (0.5, 0, 1.0, 1.0, vp_2);
   }
   string d = "17.03.17";
   string data_folder="/home/kodda/Dropbox/p2pflab/lettuceScan2/data/";
   ostringstream file;
   file<<data_folder<<d<<"/"<<argv[1]<<"/pcd/";

   vector<PCD, Eigen::aligned_allocator<PCD> > data;
   for (int i=0;i<100;i++) loadPCD (file.str(), i, data);
   //for (int i=0;i<30;i++) loadPCD (file, i, data);

   PointCloudPtr acc (new PointCloud);
   /*
         ostringstream trfile;
         trfile<<"/home/kodda/Dropbox/p2pflab/lettuceScan/raw/pcd/"<<k<<".transform";
         Eigen::Matrix4f transform;
         loadTransform(trfile.str(), transform);
         cout<<transform<<endl;
   */
   acc=data[i0].cloud;
   for (int i=i0+s; i<N/s;i++)
   {  
         ostringstream trfile;
         trfile <<data_folder<<d<<"/"<<argv[1]<<"/transf/"<<(i-s)%100<<"_"<< i%100 << ".transform";
         Eigen::Matrix4f transform;
         loadTransform(trfile.str(), transform);
         //cout<<transform<<endl;
         cout<<i<<" "<<trfile.str()<<endl;
     
         PointCloudPtr tmp (new PointCloud);

         pcl::transformPointCloud (*acc, *acc, transform);   
         //pcl::transformPointCloud (*data[i-1].cloud, *tmp, transform);   
         //if (display) showClouds(data[i].cloud, acc, vp_1,"vp1_target","vp1_source");
         //if (display) showClouds(data[i].cloud, acc, vp_2,"vp2_target","vp2_source");
         
         //*acc=*tmp+*data[i].cloud;
         //bilfil (*data[i].cloud, *data[i].cloud);
         //PointCloudWithNormals mls_points;
         //mls_smoothing(*data[i].cloud, mls_points);
         //copyPointCloud(mls_points, *data[i].cloud);
         *acc=*acc+*data[s*i].cloud;

         //if (i%3==0){  
            filtersor(*acc, 50,1.); 
            downsample (acc, .5, acc);
         //}
         
         //if (display) showClouds(data[i].cloud, acc, vp_2,"vp2_target","vp2_source");
         
         //cout<<(*acc).points.size()<<endl;
         //PointCloudWithNormals mls_points;
         //mls_smoothing(*acc, mls_points);
         //copyPointCloud(mls_points, *acc);
         //bilfil (*acc, *acc);

         
         if (display)
         {
           showClouds(data[i].cloud, acc, vp_1,"vp1_target","vp1_source");
           p->addPointCloud<PointT> (acc, "targ", vp_2);
           ostringstream pngfile;
           //pngfile<<file<<"pics_pw_"<<s<<"/"<<i<<".png";
           pngfile<<data_folder<<d<<"/"<<argv[1]<<"/pics_merge/"<<i<<".png";
           p->saveScreenshot(pngfile.str());
           p->removeAllPointClouds ();
         }
          
    }
    //downsample (acc, 1, acc);
    ostringstream pcdfile;
    pcdfile<<data_folder<<d<<"/"<<argv[1]<<"/merged.pcd";
    pcl::io::savePCDFileASCII (pcdfile.str(), *acc);
}
