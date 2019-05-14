#
#if !defined(COLOR_FILTER_H)
#define COLOR_FILTER_H
#include <pcl/console/parse.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <iostream>

typedef pcl::PointXYZRGB RefPointType;
typedef pcl::PointCloud<pcl::PointXYZRGB> Cloud;
typedef Cloud::Ptr CloudPtr;
typedef Cloud::ConstPtr CloudConstPtr;
typedef pcl::PointXYZHSV hsvRefPointType;
typedef pcl::PointCloud<pcl::PointXYZHSV> hsvCloud;
typedef hsvCloud::Ptr hsvCloudPtr;
class color_filter_rgb
{
private:
  pcl::PointCloud<RefPointType>::Ptr cloud_{ new pcl::PointCloud<RefPointType> };
  pcl::PointCloud<RefPointType>::Ptr cloud_filtered_{ new pcl::PointCloud<RefPointType> };
  pcl::ConditionAnd<RefPointType>::Ptr color_cond{ new pcl::ConditionAnd<RefPointType> };
  pcl::ConditionalRemoval<RefPointType> condrem;

public:
  color_filter_rgb()
  {
  }
  ~color_filter_rgb()
  {
  }
  void setInputCloud(pcl::PointCloud<RefPointType>::Ptr cloud)
  {
    cloud_ = cloud;
  }
  void setColorBoundary(int rMax, int rMin, int gMax, int gMin, int bMax, int bMin)
  {
    color_cond->addComparison(pcl::PackedRGBComparison<RefPointType>::Ptr(
        new pcl::PackedRGBComparison<RefPointType>("r", pcl::ComparisonOps::LT, rMax)));
    color_cond->addComparison(pcl::PackedRGBComparison<RefPointType>::Ptr(
        new pcl::PackedRGBComparison<RefPointType>("r", pcl::ComparisonOps::GT, rMin)));
    color_cond->addComparison(pcl::PackedRGBComparison<RefPointType>::Ptr(
        new pcl::PackedRGBComparison<RefPointType>("g", pcl::ComparisonOps::LT, gMax)));
    color_cond->addComparison(pcl::PackedRGBComparison<RefPointType>::Ptr(
        new pcl::PackedRGBComparison<RefPointType>("g", pcl::ComparisonOps::GT, gMin)));
    color_cond->addComparison(pcl::PackedRGBComparison<RefPointType>::Ptr(
        new pcl::PackedRGBComparison<RefPointType>("b", pcl::ComparisonOps::LT, bMax)));
    color_cond->addComparison(pcl::PackedRGBComparison<RefPointType>::Ptr(
        new pcl::PackedRGBComparison<RefPointType>("b", pcl::ComparisonOps::GT, bMin)));
  }
  void filter(pcl::PointCloud<RefPointType>::Ptr cloud_filtered)
  {
    cloud_filtered_ = cloud_filtered;
    condrem.setCondition(color_cond);
    condrem.setInputCloud(cloud_);
    condrem.setKeepOrganized(true);
    condrem.filter(*cloud_filtered_);
  }
};

class color_filter_hsv
{
private:
  pcl::PassThrough<hsvRefPointType> pass;
  pcl::PointCloud<hsvRefPointType>::Ptr cloud_ = boost::make_shared<pcl::PointCloud<hsvRefPointType> >();
  pcl::PointCloud<hsvRefPointType>::Ptr cloud_filtered_ = boost::make_shared<pcl::PointCloud<hsvRefPointType> >();
  float hMax_, hMin_, sMax_, sMin_, vMax_, vMin_;

public:
  color_filter_hsv()
  {
  }
  ~color_filter_hsv()
  {
  }
  void setInputCloud(pcl::PointCloud<hsvRefPointType>::Ptr cloud)
  {
    cloud_ = cloud;
  }
  void setColorBoundary(float hMax, float hMin, float sMax, float sMin, float vMax, float vMin)
  {
    hMax_ = hMax;
    hMin_ = hMin;
    sMax_ = sMax;
    sMin_ = sMin;
    vMax_ = vMax;
    vMin_ = vMin;
  }
  void filter(pcl::PointCloud<hsvRefPointType>::Ptr cloud_filtered)
  {
    cloud_filtered_ = cloud_filtered;
    pass.setInputCloud(cloud_);
    pass.setFilterFieldName("h");
    pass.setFilterLimits(hMin_, hMax_);
    pass.filter(*cloud_filtered_);
    pass.setInputCloud(cloud_filtered_);
    pass.setFilterFieldName("s");
    pass.setFilterLimits(sMin_, sMax_);
    pass.filter(*cloud_filtered_);
    pass.setInputCloud(cloud_filtered_);
    pass.setFilterFieldName("v");
    pass.setFilterLimits(vMin_, vMax_);
    pass.filter(*cloud_filtered_);
  }
};

#endif  // COLOR_FILTER_H
