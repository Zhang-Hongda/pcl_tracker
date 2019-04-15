
#if !defined(CLOUD_VIEWER_H)
#define CLOUD_VIEWER_H
#include <iostream>

#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

using namespace std;

template<class pointT>
class cloudViewer
{
private:
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = boost::make_shared<pcl::visualization::PCLVisualizer>();
public:
  cloudViewer(const char *WindowName)
  {
    viewer->setWindowName(WindowName);
    viewer->setBackgroundColor(0, 0, 0);
    viewer->initCameraParameters();
    viewer->setCameraPosition(0, 0, -1.5, 0, -1, 0, 0);
  }

  ~cloudViewer()
  {
  }

  void update_PointCloud(boost::shared_ptr<pcl::PointCloud<pointT> > cloud)
  {
    viewer->removeAllPointClouds();
    pcl::visualization::PointCloudColorHandlerRGBField<pointT> rgb(cloud);
    viewer->addPointCloud<pointT>(cloud, rgb, "sample cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
    // viewer->addCoordinateSystem (1.0);
    viewer->spinOnce(100);
  }
    void update_PointCloud_HSV(boost::shared_ptr<pcl::PointCloud<pointT> > cloud)
  {
    viewer->removeAllPointClouds();
    pcl::visualization::PointCloudColorHandlerHSVField<pointT> hsv(cloud);
    viewer->addPointCloud<pointT>(cloud, hsv, "sample cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
    // viewer->addCoordinateSystem (1.0);
    viewer->spinOnce(100);
  }
  void viewPointCloud(boost::shared_ptr<pcl::PointCloud<pointT> > cloud)
  {
    viewer->removeAllPointClouds();
    pcl::visualization::PointCloudColorHandlerRGBField<pointT> rgb(cloud);
    viewer->addPointCloud<pointT>(cloud, rgb, "sample cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
    // viewer->addCoordinateSystem (1.0);
    viewer->spin();
  }
  void registerPPCallback(void (*callback)(const pcl::visualization::PointPickingEvent&, void*))
  {
    viewer->registerPointPickingCallback(callback, (void*)&viewer);
  }
};

#endif // CLOUD_VIEWER_H
