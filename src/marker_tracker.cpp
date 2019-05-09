#include <ros/ros.h>
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>
#include <eigen_conversions/eigen_msg.h>

#include <iostream>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h> // PCL specific includes
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_types_conversion.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/crop_box.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "cloud_functions.h"
#include "ball_extraction.h"
#include "get_position.h"

typedef pcl::PointXYZRGB RefPointType;
typedef pcl::PointCloud<pcl::PointXYZRGB> Cloud;
typedef Cloud::Ptr CloudPtr;
typedef Cloud::ConstPtr CloudConstPtr;
typedef pcl::PointXYZHSV hsvRefPointType;
typedef pcl::PointCloud<pcl::PointXYZHSV> hsvCloud;
typedef hsvCloud::Ptr hsvCloudPtr;

ros::Publisher pub;
tf::Transform transform_marker;
tf::Transform transform_tool;
boost::shared_ptr<tf::TransformBroadcaster> br;
boost::shared_ptr<tf::TransformBroadcaster> br_tool;
float _value[3] = {135.714, 0.587413, 0.560784};
float delta = 20.0;
float delta2 = 0.10;
float hMax = _value[0] + delta;
float hMin = _value[0] - delta;
float sMax = _value[1] + delta2;
float sMin = _value[1] - delta2;
float vMax = _value[2] + delta2;
float vMin = _value[2] - delta2;
CloudPtr cloud(new Cloud());
hsvCloudPtr cloud_hsv(new hsvCloud());
CloudPtr cloud_filtered(new Cloud());
hsvCloudPtr cloud_hsv_filtered(new hsvCloud());
boost::shared_ptr<pcl::visualization::PCLVisualizer> hsvviewer = cloud_functions::hsvVis("hsv cloud");
boost::shared_ptr<pcl::visualization::PCLVisualizer> filtered_viewer;
boost::shared_ptr<pcl::visualization::PCLVisualizer> clustered_viewer;
boost::shared_ptr<pcl::visualization::PCLVisualizer> original_viewer;

Eigen::Affine3f t = Eigen::Affine3f::Identity();

double tool_length = 0.15;
bool show_filtered_viewer = true;
bool show_clustered_viewer = true;
bool show_original_viewer = true;

void pp_callback_hsv(const pcl::visualization::PointPickingEvent &event, void *viewer_void)
{
  std::cout << "Picking event active" << std::endl;
  if (event.getPointIndex() != -1)
  {
    float x, y, z;
    float h, s, v;
    event.getPoint(x, y, z);
    std::cout << "point index" << event.getPointIndex() << "  ";
    h = cloud_hsv->points[event.getPointIndex()].h > 0 ? cloud_hsv->points[event.getPointIndex()].h : _value[0];
    s = cloud_hsv->points[event.getPointIndex()].s > 0 ? cloud_hsv->points[event.getPointIndex()].s : _value[1];
    v = cloud_hsv->points[event.getPointIndex()].v > 0 ? cloud_hsv->points[event.getPointIndex()].v : _value[2];
    // std::cout << x << ";" << y << ";" << z << "    ";
    std::cout << h << ";" << s << ";" << v << std::endl;
    hMax = h + delta;
    hMin = h - delta;
    sMax = s + delta2;
    sMin = s - delta2;
    vMax = v + delta2;
    vMin = v - delta2;
  }
}
void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &input)
{
  cloud.reset(new Cloud());
  cloud_filtered.reset(new Cloud());
  cloud_hsv.reset(new hsvCloud());
  cloud_hsv_filtered.reset(new hsvCloud());
  // hsvCloudPtr cloud_cluster_all(new hsvCloud());
  std::vector<hsvCloudPtr> cloud_clusters;
  hsvviewer->removeAllShapes();
  hsvviewer->removeCoordinateSystem("marker");
  hsvviewer->removeCoordinateSystem("tool");

  std::vector<hsvCloudPtr> sphere_cloud_set;
  std::vector<pcl::ModelCoefficientsPtr> coefficients_sphere_set;
  pcl::fromROSMsg(*input, *cloud); 
  // pcl::io::savePCDFileASCII("/home/eric/work_space/test_ws/src/pcl_tracker/data/workspace.pcd", *cloud);
  if (show_original_viewer)
    cloud_functions::update_pointcloud(original_viewer, cloud);
  std::vector<int> mapping;
  pcl::removeNaNFromPointCloud(*cloud, *cloud_filtered, mapping);
  // pcl::CropBox<RefPointType> crop;
  // crop.setMin(Eigen::Vector4f(-2.0, -2.0, 0.0, 1.0));
  // crop.setMax(Eigen::Vector4f(2.0, 2.0, 1.5, 1.0));
  // crop.setInputCloud(cloud);
  // crop.setUserFilterValue(0.1f);
  // crop.setKeepOrganized(true);
  // crop.filter(*cloud_filtered);
  // cloud_functions::CloudDownSample<RefPointType>(cloud_filtered, cloud_filtered, 0.005);
  cloud_functions::PointCloudXYZRGBtoXYZHSV(*cloud_filtered, *cloud_hsv);
  std::vector<float> boundary;
  float boundary_list[6] = {hMax, hMin, sMax, sMin, vMax, vMin};
  for (int i = 0; i < 6; i++)
    boundary.push_back(boundary_list[i]);
  cloud_functions::color_filter_hsv(cloud_hsv, cloud_hsv_filtered, boundary); // hsv filter
  if (show_filtered_viewer)
    cloud_functions::update_pointcloud(filtered_viewer, cloud_hsv_filtered);

  if (cloud_hsv_filtered->points.size() < cloud_hsv->points.size() * 0.3 && cloud_hsv_filtered->points.size() > 50)
  {
    if (ball_extraction(cloud_hsv_filtered, cloud_clusters, sphere_cloud_set, coefficients_sphere_set))
    {
      int temp = 0;
      t = get_position(coefficients_sphere_set);
      if (t.matrix() != Eigen::Affine3f::Identity().matrix())
      {
        Eigen::Affine3f t_tool = t * Eigen::Translation3f(0, 0, tool_length);
        tf::transformEigenToTF(t.cast<double>(), transform_marker);
        tf::transformEigenToTF(t_tool.cast<double>(), transform_tool);
        br->sendTransform(tf::StampedTransform(transform_marker, ros::Time::now(), "kinect2_link", "marker"));
        br_tool->sendTransform(tf::StampedTransform(transform_tool, ros::Time::now(), "kinect2_link", "tool"));
        hsvviewer->addCoordinateSystem(0.2, t, "marker", 0);
        hsvviewer->addCoordinateSystem(0.2, t_tool, "tool", 0);
      }
      for (std::vector<pcl::ModelCoefficientsPtr>::const_iterator it = coefficients_sphere_set.begin(); it != coefficients_sphere_set.end(); it++)
      {
        std::stringstream ss;
        ss << "sphere_" << temp;
        const pcl::PointXYZ center((*it)->values[0], (*it)->values[1], (*it)->values[2]);
        double radius = (*it)->values[3];
        hsvviewer->addSphere(center, radius, 255, 0, 0, ss.str());
        temp++;
      }
      if (show_clustered_viewer)
      {
        clustered_viewer->removeAllPointClouds();
        std::stringstream ss;
        int i = 0;
        for (std::vector<hsvCloudPtr>::const_iterator it = cloud_clusters.begin(); it != cloud_clusters.end(); it++)
        {
          ss << "cluster" << i;
          i++;
          pcl::visualization::PointCloudColorHandlerRandom<pcl::PointXYZHSV> handler(*it);
          clustered_viewer->addPointCloud<pcl::PointXYZHSV>(*it, handler, ss.str());
          clustered_viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, ss.str());
          ss.str("");
        }
      }
      // cloud_functions::update_pointcloud(clustered_viewer, cloud_cluster_all);
    }
  }
  cloud_functions::update_pointcloud(hsvviewer, cloud_hsv);
  // sensor_msgs::PointCloud2 output;        
  // pcl::toROSMsg(*cloud_filtered, output); 
  // pub.publish(output);
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "marker_tracker_node"); // Initialize ROS
  ROS_INFO("marker_tracker_node Start!");
  ros::NodeHandle nh;
  nh.getParam("tool_length", tool_length);
  nh.getParam("show_filtered_viewer", show_filtered_viewer);
  nh.getParam("show_clustered_viewer", show_clustered_viewer);
  nh.getParam("show_original_viewer", show_original_viewer);
  ROS_INFO("tool_length: %f", tool_length);
  ROS_INFO("show_fitered_viewer: %s", std::to_string(show_filtered_viewer).c_str());
  ROS_INFO("show_clustered_viewer: %s", std::to_string(show_clustered_viewer).c_str());
  ROS_INFO("show_original_viewer: %s", std::to_string(show_original_viewer).c_str());
  if (show_clustered_viewer)
    clustered_viewer = cloud_functions::hsvVis("clustered cloud");
  if (show_filtered_viewer)
    filtered_viewer = cloud_functions::hsvVis("filtered cloud");
  if (show_original_viewer)
    original_viewer = cloud_functions::rgbVis("original cloud");

  br.reset(new tf::TransformBroadcaster());
  br_tool.reset(new tf::TransformBroadcaster());
  hsvviewer->registerPointPickingCallback(pp_callback_hsv, (void *)&hsvviewer);
  hsvviewer->setCameraPosition(0, 0, -1.0, 0, 0, 0, 0);
  // hsvviewer->addCoordinateSystem(0.2,0,0,0,"camera",0);
  ros::Subscriber sub = nh.subscribe("input", 1, cloud_cb);
  // pub = nh.advertise<sensor_msgs::PointCloud2>("output", 1);
  ros::spin(); // Spin
}
