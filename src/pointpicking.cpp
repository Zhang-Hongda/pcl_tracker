#include <iostream> 
#include <boost/thread/thread.hpp> 
#include <pcl/common/common_headers.h> 
#include <pcl/features/normal_3d.h> 
#include <pcl/io/pcd_io.h> 
#include <pcl/visualization/pcl_visualizer.h> 
#include <pcl/console/parse.h> 
#include <pcl/surface/convex_hull.h> 
#include <pcl/visualization/pcl_plotter.h> 
#include <pcl/visualization/point_picking_event.h> 

void 
pp_callback(const pcl::visualization::PointPickingEvent& event, void* 
viewer_void){ 
   std::cout << "Picking event active" << std::endl; 
     if(event.getPointIndex()!=-1) 
     { 
         float x,y,z; 
         event.getPoint(x,y,z); 
         std::cout << x<< ";" << y<<";" << z << std::endl; 
     } 
} 

boost::shared_ptr<pcl::visualization::PCLVisualizer> customColourVis 
(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud) 
{ 
   // -------------------------------------------- 
   // -----Open 3D viewer and add point cloud----- 
   // -------------------------------------------- 
   boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new 
pcl::visualization::PCLVisualizer ("3D Viewer")); 
   viewer->setBackgroundColor (0, 0, 0); 
pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> 
single_color(cloud, 0, 255, 0); 
   viewer->addPointCloud<pcl::PointXYZ> (cloud, single_color, "sample cloud"); 
   viewer->setPointCloudRenderingProperties 
(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud"); 
   viewer->addCoordinateSystem (1.0); 
   viewer->initCameraParameters (); 
   viewer->registerPointPickingCallback (pp_callback,(void*)&viewer); 
   return (viewer); 
} 


int 
main ( int argc, char** argv ) { 
   pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr (new 
pcl::PointCloud<pcl::PointXYZ>); 
   std::cout << "Genarating example point clouds.\n\n"; 
   // We're going to make an ellipse extruded along the z-axis. The colour for 
   // the XYZRGB cloud will gradually go from red to green to blue. 
   uint8_t r(255), g(15), b(15); 
   for (float z(-1.0); z <= 1.0; z += 0.05) 
   { 
     for (float angle(0.0); angle <= 360.0; angle += 5.0) 
     { 
       pcl::PointXYZ basic_point; 
       basic_point.x = 0.5 * cosf (pcl::deg2rad(angle)); 
       basic_point.y = sinf (pcl::deg2rad(angle)); 
       basic_point.z = z; 
       basic_cloud_ptr->points.push_back(basic_point); 
     } 

   } 
   basic_cloud_ptr->width = (int) basic_cloud_ptr->points.size (); 
   basic_cloud_ptr->height = 1; 


   boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer; 
   viewer = customColourVis(basic_cloud_ptr); 
   while (!viewer->wasStopped ()) 
   { 
     viewer->spinOnce (100); 
     boost::this_thread::sleep (boost::posix_time::microseconds (100000)); 
   } 



} 


