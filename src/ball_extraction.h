#
#if !defined(BALL_EXTRACTION_H)
#define BALL_EXTRACTION_H

#include <pcl/ModelCoefficients.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/mls.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/surface/impl/mls.hpp>
typedef pcl::PointXYZHSV hsvRefPointType;
typedef pcl::PointCloud<hsvRefPointType> hsvCloud;
typedef hsvCloud::Ptr hsvCloudPtr;

void fit_sphere(hsvCloudPtr in, hsvCloudPtr cloud_sphere, pcl::ModelCoefficients::Ptr coefficients_sphere)
{
  pcl::NormalEstimation<hsvRefPointType, pcl::Normal> ne;
  pcl::search::KdTree<hsvRefPointType>::Ptr tree(new pcl::search::KdTree<hsvRefPointType>());
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
  ne.setSearchMethod(tree);
  ne.setInputCloud(in);
  ne.setKSearch(50);
  ne.compute(*cloud_normals);

  pcl::SACSegmentationFromNormals<hsvRefPointType, pcl::Normal> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_NORMAL_SPHERE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setNormalDistanceWeight(0.1);
  seg.setMaxIterations(10000);
  seg.setDistanceThreshold(0.05);
  seg.setRadiusLimits(0, 0.1);
  seg.setInputCloud(in);
  seg.setInputNormals(cloud_normals);
  pcl::PointIndices::Ptr inliers_sphere(new pcl::PointIndices);

  seg.segment(*inliers_sphere, *coefficients_sphere);  // Obtain the sphere inliers and coefficients
  // std::cerr << "Sphere coefficients: " << *coefficients_sphere << std::endl;
  pcl::ExtractIndices<hsvRefPointType> extract;
  extract.setInputCloud(in);
  extract.setIndices(inliers_sphere);
  extract.setNegative(false);
  extract.filter(*cloud_sphere);
}
bool ball_extraction(hsvCloudPtr cloud, std::vector<hsvCloudPtr>& cloud_clusters,
                     std::vector<hsvCloudPtr>& sphere_cloud_set,
                     std::vector<pcl::ModelCoefficientsPtr>& coefficients_sphere_set)
{
  if (cloud->size() == 0)
    return (false);
  hsvCloudPtr cloud_filtered(new hsvCloud());
  // hsvCloudPtr cloud_cluster_all (new hsvCloud());
  // cloud_cluster_all.reset(new hsvCloud());
  cloud_clusters.clear();
  cloud_filtered = cloud;
  pcl::search::KdTree<hsvRefPointType>::Ptr tree(new pcl::search::KdTree<hsvRefPointType>);
  tree->setInputCloud(cloud_filtered);
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<hsvRefPointType> ec;
  ec.setClusterTolerance(0.02);
  ec.setMinClusterSize(50);
  ec.setMaxClusterSize(25000);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud_filtered);
  ec.extract(cluster_indices);

  // std::cout << "Find " << cluster_indices.size() << " balls" << "\n";
  int cluster_number = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
  {
    hsvCloudPtr cloud_cluster(new hsvCloud());
    hsvCloudPtr sphere_cloud(new hsvCloud());
    pcl::ModelCoefficients::Ptr coefficients_sphere(new pcl::ModelCoefficients);
    for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
      cloud_cluster->points.push_back(cloud_filtered->points[*pit]);

    cloud_cluster->width = cloud_cluster->points.size();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;
    fit_sphere(cloud_cluster, sphere_cloud, coefficients_sphere);  // fit sphere and save coefficients
    sphere_cloud_set.push_back(sphere_cloud);
    coefficients_sphere_set.push_back(coefficients_sphere);
    cloud_clusters.push_back(cloud_cluster);
    cluster_number++;
    // *cloud_cluster_all += *cloud_cluster;
  }
  return cluster_number == 3;
}
#endif  // BALL_EXTRACTION_H
