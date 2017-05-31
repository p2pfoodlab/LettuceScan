#include <boost/make_shared.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_representation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>

#include <pcl/filters/filter.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>

#include <stdlib.h>
#include <iostream>
#include <sstream>
 
using pcl::visualization::PointCloudColorHandlerGenericField;
using pcl::visualization::PointCloudColorHandlerCustom;

using namespace std;

typedef pcl::PointXYZRGB PointT;
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

class MyPointRepresentation : public pcl::PointRepresentation <PointNormalT>
{
  using pcl::PointRepresentation<PointNormalT>::nr_dimensions_;
public:
  MyPointRepresentation ()
  {
    // Define the number of dimensions
    nr_dimensions_ = 4;
  }

  // Override the copyToFloatArray method to define our feature vector
  virtual void copyToFloatArray (const PointNormalT &p, float * out) const
  {
    // < x, y, z, curvature >
    out[0] = p.x;
    out[1] = p.y;
    out[2] = p.z;
    out[3] = p.curvature;
  }
};


void downsample (PointCloudPtr &points, float leaf_size, PointCloudPtr &downsampled_out)
{
  pcl::VoxelGrid<PointT> vox_grid;
  vox_grid.setLeafSize (leaf_size, leaf_size, leaf_size);
  vox_grid.setInputCloud (points);
  vox_grid.filter (*downsampled_out);
}


void loadPCD (string path, int k, vector<PCD, Eigen::aligned_allocator<PCD> > &data)
{
      PCD m;      
      ostringstream pcdfile;
      pcdfile<<path<<"pcd/"<<k<<".pcd";
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

void saveTransform (const std::string &file, const Eigen::Matrix4f &transform)
{
  ofstream ofs;
  ofs.open (file.c_str (), ios::trunc | ios::binary);
  for (int i = 0; i < 4; ++i)
    for (int j = 0; j < 4; ++j)
      ofs.write (reinterpret_cast<const char*>(&transform (i, j)), sizeof (float));  
  ofs.close ();
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

void pairAlign (const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr output, Eigen::Matrix4f &final_transform, bool downsample)
{
  //Downsample
  PointCloud::Ptr src (new PointCloud);
  PointCloud::Ptr tgt (new PointCloud);
  pcl::VoxelGrid<PointT> grid;
  if (downsample)
  {
    grid.setLeafSize (1, 1, 1);
    grid.setInputCloud (cloud_src);
    grid.filter (*src);

    grid.setInputCloud (cloud_tgt);
    grid.filter (*tgt);
  }
  else
  {
    src = cloud_src;
    tgt = cloud_tgt;
  }

  //get Normals
  PointCloudWithNormals::Ptr points_with_normals_src (new PointCloudWithNormals);
  PointCloudWithNormals::Ptr points_with_normals_tgt (new PointCloudWithNormals);

  pcl::NormalEstimation<PointT, PointNormalT> norm_est;
  pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
  norm_est.setSearchMethod (tree);
  norm_est.setKSearch (30);
  
  norm_est.setInputCloud (src);
  norm_est.compute (*points_with_normals_src);
  pcl::copyPointCloud (*src, *points_with_normals_src);

  norm_est.setInputCloud (tgt);
  norm_est.compute (*points_with_normals_tgt);
  pcl::copyPointCloud (*tgt, *points_with_normals_tgt);

  MyPointRepresentation point_representation;
  float alpha[4] = {1.0, 1.0, 1.0, 1.0};
  point_representation.setRescaleValues (alpha);

  //Register
  pcl::IterativeClosestPointNonLinear<PointNormalT, PointNormalT> reg;
  reg.setTransformationEpsilon (1e-6);
  reg.setMaxCorrespondenceDistance (10);  
  reg.setPointRepresentation (boost::make_shared<const MyPointRepresentation> (point_representation));
  reg.setInputSource (points_with_normals_src);
  reg.setInputTarget (points_with_normals_tgt);

  // Run the same optimization in a loop and visualize the results
  Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity (), prev, sourceToTarget;
  PointCloudWithNormals::Ptr reg_result = points_with_normals_src;
  reg.setMaximumIterations (5);
  for (int i = 0; i < 5; ++i)
  {
    //PCL_INFO ("Iteration Nr. %d.\n", i);
    points_with_normals_src = reg_result;

    // Estimate
    reg.setInputSource (points_with_normals_src);
    reg.align (*reg_result);

    Ti = reg.getFinalTransformation () * Ti;

    if (fabs ((reg.getLastIncrementalTransformation () - prev).sum ()) < reg.getTransformationEpsilon ())
    {
      reg.setMaxCorrespondenceDistance (reg.getMaxCorrespondenceDistance () - 0.01);
      //cout<<reg.getMaxCorrespondenceDistance ()<<endl;
    }
    prev = reg.getLastIncrementalTransformation ();
    cout<<i<<" "<<reg.getFitnessScore()<<endl;
 
  }

  sourceToTarget = Ti;//.inverse();
  pcl::transformPointCloud (*cloud_src, *output, sourceToTarget);
  final_transform = sourceToTarget;
  
}



int main (int argc, char** argv)
{
   int i0=atoi(argv[3]);
   int N=atoi(argv[4]);
   int s=atoi(argv[5]);
   bool display=true;
   
   if (display)
   {
      p = new pcl::visualization::PCLVisualizer (argc, argv, "match fusion");

      p->createViewPort (0.0, 0, 0.5, 1.0, vp_1);
      p->createViewPort (0.5, 0, 1.0, 1.0, vp_2);
   }

   string d = "17.03.17";
   string data_folder="/home/kodda/Dropbox/p2pflab/lettuceScan2/data/";
   ostringstream file;
   file<<data_folder<<argv[1]<<"/"<<argv[2]<<"/";

   vector<PCD, Eigen::aligned_allocator<PCD> > data;
   for (int i=0;i<N;i++) loadPCD (file.str(), i, data);
      
   Eigen::Matrix4f pairTransform;
   PointCloudPtr acc (new PointCloud);
   PointCloud::Ptr temp (new PointCloud); 
   acc=data[0].cloud;
   for (int i=i0+s; i<N+s;i++)
   {  
      acc=data[(i-s)%100].cloud;      
      pairAlign (acc, data[i%100].cloud, temp, pairTransform, true);
      if (display){
         showClouds(data[(i-s)%100].cloud, data[i%100].cloud, vp_1,"vp1_target","vp1_source");
         showClouds(temp, data[i%100].cloud, vp_2,"vp2_target"," vp2_source");
      }   
      
      //*acc=*temp+*data[i].cloud;
      //filtersor(*acc, 25,2.); 
      //downsample (acc, 2., acc);
      
      std::stringstream sT;
      //sT <<file<<"transf_pw_"<<s<<"/"<<(i-s)<<"_"<< (i-s)%100 << ".transform";
      sT <<data_folder<<argv[1]<<"/"<<argv[2]<<"/transf/"<<(i-s)%100<<"_"<< i%100 << ".transform";
      saveTransform (sT.str(), pairTransform);
      
      if (display)
      {
         //p->addPointCloud<PointT> (acc, "targ", vp_2);
         ostringstream pngfile;
         //pngfile<<file<<"pics_pw_"<<s<<"/"<<i<<".png";
         pngfile<<data_folder<<argv[1]<<"/"<<argv[2]<<"/pics_transf/"<<i<<".png";
         p->saveScreenshot(pngfile.str());
         p->removeAllPointClouds ();
      }
    }   
    //pcl::io::savePCDFileASCII ("/home/kodda/Dropbox/p2pflab/lettuceScan/data/raw/all.pcd", *acc);
    
}
