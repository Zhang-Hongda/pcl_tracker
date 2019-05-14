#
#if !defined(CLOUD_FUNCTIONS_H)
#define CLOUD_FUNCTIONS_H

#include <pcl/ModelCoefficients.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types_conversion.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

typedef pcl::PointXYZRGB RefPointType;
typedef pcl::PointCloud<pcl::PointXYZRGB> Cloud;
typedef Cloud::Ptr CloudPtr;
typedef Cloud::ConstPtr CloudConstPtr;

typedef pcl::PointXYZHSV hsvRefPointType;
typedef pcl::PointCloud<pcl::PointXYZHSV> hsvCloud;
typedef hsvCloud::Ptr hsvCloudPtr;
namespace cloud_functions
{
boost::shared_ptr<pcl::visualization::PCLVisualizer> rgbVis(CloudPtr cloud)
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("RGB Viewer"));
  viewer->setBackgroundColor(0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<RefPointType> rgb(cloud);
  viewer->addPointCloud<RefPointType>(cloud, rgb, "sample cloud");
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "sample cloud");
  viewer->addCoordinateSystem(1.0);
  viewer->initCameraParameters();
  viewer->setCameraPosition(0, 0, -1.5, 0, -1, 0, 0);
  // viewer->registerPointPickingCallback (pp_callback,(void*)&viewer);
  return (viewer);
}
boost::shared_ptr<pcl::visualization::PCLVisualizer> rgbVis(const char* WindowName)
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer(WindowName));
  viewer->setBackgroundColor(0, 0, 0);
  // viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters();
  viewer->setCameraPosition(0, 0, -1.5, 0, -1, 0, 0);
  // viewer->registerPointPickingCallback (pp_callback,(void*)&viewer);
  return (viewer);
}
void update_pointcloud(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer, CloudPtr cloud_)
{
  viewer->removeAllPointClouds();
  pcl::visualization::PointCloudColorHandlerRGBField<RefPointType> rgb(cloud_);
  viewer->addPointCloud<RefPointType>(cloud_, rgb, "sample cloud");
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "sample cloud");
  viewer->spinOnce(100);
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> hsvVis(pcl::PointCloud<pcl::PointXYZHSV>::Ptr cloud)
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("HSV Viewer"));
  viewer->setBackgroundColor(0, 0, 0);
  pcl::visualization::PointCloudColorHandlerHSVField<hsvRefPointType> hsv(cloud);
  viewer->addPointCloud<hsvRefPointType>(cloud, hsv, "sample cloud");
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "sample cloud");
  viewer->addCoordinateSystem(1.0);
  viewer->initCameraParameters();
  // viewer->registerPointPickingCallback (pp_callback_hsv,(void*)&viewer);
  return (viewer);
}
boost::shared_ptr<pcl::visualization::PCLVisualizer> hsvVis(const char* WindowName)
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer(WindowName));
  viewer->setBackgroundColor(0, 0, 0);
  // viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters();
  viewer->setCameraPosition(0, 0, -1.5, 0, -1, 0, 0);
  // viewer->registerPointPickingCallback (pp_callback_hsv,(void*)&viewer);
  return (viewer);
}
void update_pointcloud(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer, hsvCloudPtr cloud_)
{
  viewer->removeAllPointClouds();
  pcl::visualization::PointCloudColorHandlerHSVField<hsvRefPointType> hsv(cloud_);
  viewer->addPointCloud<hsvRefPointType>(cloud_, hsv, "sample cloud");
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "sample cloud");
  viewer->spinOnce(100);
}
void PointCloudXYZRGBtoXYZHSV(Cloud& in, hsvCloud& out)
{
  out.width = in.width;
  out.height = in.height;
  out.header = in.header;
  for (size_t i = 0; i < in.points.size(); i++)
  {
    hsvRefPointType p;
    pcl::PointXYZRGBtoXYZHSV(in.points[i], p);
    p.x = in.points[i].x;
    p.y = in.points[i].y;
    p.z = in.points[i].z;
    out.points.push_back(p);
  }
}

void PointCloudXYZHSVtoXYZRGB(hsvCloud& in, Cloud& out)
{
  out.width = in.width;
  out.height = in.height;
  out.header = in.header;
  for (size_t i = 0; i < in.points.size(); i++)
  {
    RefPointType p;
    pcl::PointXYZHSVtoXYZRGB(in.points[i], p);
    p.x = in.points[i].x;
    p.y = in.points[i].y;
    p.z = in.points[i].z;
    out.points.push_back(p);
  }
}
template <typename PointT>
void CloudDownSample(typename pcl::PointCloud<PointT>::Ptr input_cloud,
                     typename pcl::PointCloud<PointT>::Ptr output_cloud, float LS = 0.01f)
{
  float LeafeSize = LS;
  pcl::VoxelGrid<RefPointType> filter;
  filter.setInputCloud(input_cloud);
  filter.setLeafSize(LeafeSize, LeafeSize, LeafeSize);
  filter.filter(*output_cloud);
}
void color_filter_hsv(hsvCloudPtr cloud, hsvCloudPtr cloud_filtered, std::vector<float> boundary)
{
  float hMax, hMin, sMax, sMin, vMax, vMin;
  hMax = boundary[0];
  hMin = boundary[1];
  sMax = boundary[2];
  sMin = boundary[3];
  vMax = boundary[4];
  vMin = boundary[5];
  pcl::PassThrough<hsvRefPointType> pass;
  pass.setInputCloud(cloud);
  pass.setFilterFieldName("h");
  pass.setFilterLimits(hMin, hMax);
  pass.filter(*cloud_filtered);
  pass.setInputCloud(cloud_filtered);
  pass.setFilterFieldName("s");
  pass.setFilterLimits(sMin, sMax);
  pass.filter(*cloud_filtered);
  pass.setInputCloud(cloud_filtered);
  pass.setFilterFieldName("v");
  pass.setFilterLimits(vMin, vMax);
  pass.filter(*cloud_filtered);
}
template <typename T>
inline T maximum(T a, T b)
{
  return a > b ? a : b;
}
template <typename T>
inline T minimum(T a, T b)
{
  return a > b ? b : a;
}
template <typename T>
inline T maximum(T a, T b, T c)
{
  return maximum(maximum(a, b), c);
}
template <typename T>
inline T minimum(T a, T b, T c)
{
  return minimum(minimum(a, b), c);
}
template <typename T>
inline T median(T a, T b, T c)
{
  return maximum(minimum(a, b), minimum(maximum(a, b), c));
}
template <typename T>
inline int get_index(std::vector<T> V, T number)
{
  std::vector<double>::iterator it = std::find(V.begin(), V.end(), number);
  return std::distance(V.begin(), it);
}
}
#endif  // CLOUD_FUNCTIONS_H