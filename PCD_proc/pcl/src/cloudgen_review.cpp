#include <boost/make_shared.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/io/png_io.h>
#include <pcl/point_cloud.h>

#include <pcl/filters/filter.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/fast_bilateral.h>

#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <stdlib.h>
#include"cnpy.h"
#include <iostream>
#include <sstream>

using pcl::visualization::PointCloudColorHandlerGenericField;
using pcl::visualization::PointCloudColorHandlerCustom;

using namespace cv;
using namespace cnpy;
using namespace std;

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef PointCloud::Ptr PointCloudPtr;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;
pcl::visualization::PCLVisualizer *p;

double FOCAL = 570.3;
double MM_PER_M = 1000;
int WIDTH = 240;
int HEIGHT = 320;
double DEPTH_THRESHOLD = 400;   
double X_THRESHOLD = 1000;   
double Y_THRESHOLD = 1000;   
int vp_1;

struct PCD
{
  PointCloud::Ptr cloud;
  std::string f_name;

  PCD() : cloud (new PointCloud) {};
};


void loadPCD (NpyArray vert_arr, NpyArray sync_arr, Mat confimg, PointCloud& pointcloud)
{
   short* vert=reinterpret_cast<short*>(vert_arr.data);
   uint8_t* sync=reinterpret_cast<uint8_t*>(sync_arr.data);
   int s=vert_arr.shape[0]*vert_arr.shape[1];
   const float bad_point = std::numeric_limits<float>::quiet_NaN();

   // convert all the points in the mask into point cloud
   for (int i = 0; i < confimg.rows; i++)
   {
     for (int j = 0; j < confimg.cols; j++)
     {
      int confidence = (int)confimg.at<uchar> (i, j);
      int k=(j+i*vert_arr.shape[1]);
      if (confidence>254 && vert[3*k+2]<DEPTH_THRESHOLD && vert[3*k]<X_THRESHOLD && vert[3*k+1]<Y_THRESHOLD && ((vert[3*k]*.32+vert[3*k+1]*1.18+391.9-vert[3*k+2])>100)){
      //if (((vert[3*k]*.32+vert[3*k+1]*1.18+391.9-vert[3*k+2])>100)&&confidence>254){
   
        PointT point;
        point.x = vert[3*k];
        point.y = vert[3*k+1];
        point.z = vert[3*k+2];
        point.b = sync[3*k];
        point.g = sync[3*k+1];
        point.r = sync[3*k+2];
        pointcloud.points.push_back (point);
      }
      else{
        PointT point;
        point.x = bad_point;
        point.y = bad_point;
        point.z = bad_point;
        point.b = bad_point;
        point.g = bad_point;
        point.r = bad_point;
        pointcloud.points.push_back (point); 
      } 
        
    }
  }
  pointcloud.points.resize(confimg.rows*confimg.cols);
  //pointcloud.width=pointcloud.points.size();
  //pointcloud.height=1;
  pointcloud.width=confimg.rows;
  pointcloud.height=confimg.cols;

  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(pointcloud, pointcloud, indices);
  
}

void bilfil (PointCloud& pcloud, PointCloud &output)
{
   pcl::FastBilateralFilter<PointT> fbFilter;
   fbFilter.setInputCloud(pcloud.makeShared());
       
   fbFilter.setSigmaR(25.0f);
   fbFilter.setSigmaS(2.f);
   fbFilter.applyFilter(output);
}

void prefilters(PointCloud& pcloud)
{
   
   pcl::PassThrough<PointT> passz;
   passz.setInputCloud (pcloud.makeShared());
   passz.setFilterFieldName ("z");
   passz.setFilterLimits (-500.0, 500.0);
   passz.setKeepOrganized (true); 
   passz.filter (pcloud);
   
   pcl::PassThrough<PointT> passx;
   passx.setInputCloud (pcloud.makeShared());
   passx.setFilterFieldName ("x");
   passx.setFilterLimits (-500.0, 500.0);
   passx.setKeepOrganized (true); 
   passx.filter (pcloud);

   pcl::PassThrough<PointT> passy;
   passy.setInputCloud (pcloud.makeShared());
   passy.setFilterFieldName ("y");
   passy.setFilterLimits (-500.0, 500.0);
   passy.setKeepOrganized (true); 
   passy.filter (pcloud);
   
   bilfil (pcloud, pcloud);
   /* 
   pcl::StatisticalOutlierRemoval<PointT> sor;
   sor.setInputCloud (pcloud.makeShared());
   sor.setMeanK (30);
   sor.setStddevMulThresh (1.);
   sor.setKeepOrganized (true);
   sor.filter (pcloud);
   //cout<<pcloud.isOrganized()<<endl;

   std::vector<int> indices;
   pcl::removeNaNFromPointCloud(pcloud, pcloud, indices);
   //cout<<pcloud.isOrganized()<<endl;
   */
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

int main (int argc, char** argv)
{
   p = new pcl::visualization::PCLVisualizer ("Raw PCD");
   p->initCameraParameters ();
   //p->setSize(300,300);
   //p->addCoordinateSystem (1.0);
   p->createViewPort (0.0, 0, 1.0, 1.0, vp_1);

   int k=atoi(argv[3]);
   string data_folder = "/home/kodda/Dropbox/p2pflab/data/";
   string d = "17.03.21";
   string svg="/home/kodda/Dropbox/p2pflab/lettuceScan2/data/";

   cout<<"Loading..."; 
   cout<<data_folder<<argv[1]<<"/"<<argv[2]<<"...";
   NpyArray vert_arr; 
   NpyArray sync_arr;
   Mat confimg;
   ostringstream vertfile;
   vertfile<<data_folder<<argv[1]<<"/"<<argv[2]<<"/"<<"vert_"<<k<<".npy";
   vert_arr = npy_load(vertfile.str());

   ostringstream syncfile;
   syncfile<<data_folder<<argv[1]<<"/"<<argv[2]<<"/"<<"sync_"<<k<<".npy";
   sync_arr = npy_load(syncfile.str());
   
   ostringstream conffile;
   conffile<<data_folder<<argv[1]<<"/"<<argv[2]<<"/"<<"confidence_"<<k<<".png";
   confimg = imread (conffile.str(), 0);   
   
   PointCloud pcloud;
   loadPCD (vert_arr, sync_arr, confimg, pcloud);
   cout<<"done"<<endl;
   vert_arr.destruct();
   sync_arr.destruct();
   
   prefilters(pcloud);   
   ostringstream pcdfile;
   pcdfile<<svg<<argv[1]<<"/"<<argv[2]<<"/pcd/"<<k<<".pcd";
   cout<<pcdfile.str()<<endl;
   //pcl::io::savePCDFileASCII (pcdfile.str(), pcloud);
   //pcl::io::savePCDFileASCII ("test.pcd", pcloud);
   p->addPointCloud<PointT> (pcloud.makeShared(), "targ");
   p->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "targ");
   //p->initCameraParameters ();
   //p->setSize(300,300);
   //p->addCoordinateSystem (1.0);
   
   ostringstream pngfile;
   pngfile<<svg<<argv[1]<<"/"<<argv[2]<<"/pics_pcd_review/"<<k<<".png";
   p->saveScreenshot(pngfile.str());
   //p->saveScreenshot("test.png");
   p->removeAllPointClouds ();

   /*
   PointCloudWithNormals mls_points;
   mls_smoothing(pcloud, mls_points);
   ostringstream pcdfile2;
   pcdfile2<<svg<<k<<"_mls.pcd";
   cout<<pcdfile2.str()<<endl;
   pcl::io::savePCDFileASCII (pcdfile2.str(), mls_points);
   */
}
