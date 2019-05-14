#ifndef GET_POSITON_H
#define GET_POSITON_H

#include <pcl/ModelCoefficients.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "cloud_functions.h"
using namespace cloud_functions;
Eigen::Affine3f get_position(std::vector<pcl::ModelCoefficientsPtr> coefficients_sphere_set)
{
  std::vector<Eigen::Vector3f> center_set;
  std::vector<double> radius_set;
  std::vector<Eigen::Vector3f> axis;
  std::vector<double> norm_set;
  Eigen::Vector3f x, y, z;
  Eigen::Matrix3f R = Eigen::Matrix3f::Identity();
  Eigen::Vector3f T(0, 0, 0);
  Eigen::Affine3f t = Eigen::Affine3f::Identity();
  for (std::vector<pcl::ModelCoefficientsPtr>::const_iterator it = coefficients_sphere_set.begin();
       it != coefficients_sphere_set.end(); it++)
  {
    Eigen::Vector3f center((*it)->values[0], (*it)->values[1], (*it)->values[2]);
    double radius = (*it)->values[3];
    center_set.push_back(center);
    radius_set.push_back(radius);
  }
  axis.push_back(center_set[0] - center_set[1]);
  axis.push_back(center_set[0] - center_set[2]);
  axis.push_back(center_set[1] - center_set[2]);
  for (std::vector<Eigen::Vector3f>::const_iterator it = axis.begin(); it != axis.end(); it++)
    norm_set.push_back(it->norm());
  // double max_length = maximum<double>(axis[0].norm(), axis[1].norm(), axis[2].norm());
  double min_length = minimum<double>(axis[0].norm(), axis[1].norm(), axis[2].norm());
  double median_length = median<double>(axis[0].norm(), axis[1].norm(), axis[2].norm());
  // std :: cout << min_length << "\t" << median_length << "\n";
  int x_index = get_index<double>(norm_set, min_length);
  int y_index = get_index<double>(norm_set, median_length);
  if (x_index == 0 && y_index == 1)
  {
    x = -axis[0].normalized();
    y = -axis[1].normalized();
    z = x.cross(y).normalized();
    x = y.cross(z).normalized();
    T = center_set[0];
  }
  else if (x_index == 1 && y_index == 0)
  {
    x = -axis[1].normalized();
    y = -axis[0].normalized();
    z = x.cross(y).normalized();
    x = y.cross(z).normalized();
    T = center_set[0];
  }
  else if (x_index == 0 && y_index == 2)
  {
    x = axis[0].normalized();
    y = -axis[2].normalized();
    z = x.cross(y).normalized();
    x = y.cross(z).normalized();
    T = center_set[1];
  }
  else if (x_index == 2 && y_index == 0)
  {
    x = -axis[2].normalized();
    y = axis[0].normalized();
    z = x.cross(y).normalized();
    x = y.cross(z).normalized();
    T = center_set[1];
  }
  else if (x_index == 1 && y_index == 2)
  {
    x = axis[1].normalized();
    y = axis[2].normalized();
    z = x.cross(y).normalized();
    x = y.cross(z).normalized();
    T = center_set[2];
  }
  else if (x_index == 2 && y_index == 1)
  {
    x = axis[2].normalized();
    y = axis[1].normalized();
    z = x.cross(y).normalized();
    x = y.cross(z).normalized();
    T = center_set[2];
  }
  Eigen::AngleAxisf r_v;
  r_v.fromRotationMatrix(R);
  t.rotate(R);
  t.translation() = T.matrix();
  // std::cout<<"R\n" << R.matrix() << "\n";
  // std::cout<<"T\n" << T.matrix() << "\n";
  // std::cout<<"t\n" << t.matrix() << "\n";
  return t;
}

#endif  // GET_POSITON_H