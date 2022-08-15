#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
using namespace std;

void VisualizeCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& filter_cloud) {
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("show point cloud"));

    int v1(0), v2(0);
    viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
    viewer->setBackgroundColor(0, 0, 0, v1);
    viewer->addText("point clouds", 10, 10, "v1_text", v1);
    viewer->createViewPort(0.5, 0.0, 1, 1.0, v2);
    viewer->setBackgroundColor(0.1, 0.1, 0.1, v2);
    viewer->addText("filtered point clouds", 10, 10, "v2_text", v2);
    // Rendering according to the z field, changing z to x or y is rendering according to the x or y field
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZRGB> fildColor(cloud, "z");
    viewer->addPointCloud<pcl::PointXYZRGB>(cloud, fildColor, "sample cloud", v1);

    viewer->addPointCloud<pcl::PointXYZRGB>(filter_cloud, "cloud_filtered", v2);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "cloud_filtered", v2);
    //viewer->addCoordinateSystem(1.0);
    //viewer->initCameraParameters();
    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
}

int
main()
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PCDReader reader;
    reader.read<pcl::PointXYZRGB>("*.pcd", *cloud);
    cout << "Cloud before filtering:\n " << *cloud << endl;
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    sor.setInputCloud(cloud);   
    sor.setStddevMulThresh(1);  
    sor.filter(*cloud_filtered); 
    cout << "Cloud after filtering: \n" << *cloud_filtered << endl;

    pcl::PCDWriter writer;
    writer.write<pcl::PointXYZRGB> ("*.pcd", *cloud_filtered, true);
    //VisualizeCloud(cloud, cloud_filtered);

    return (0);
}

