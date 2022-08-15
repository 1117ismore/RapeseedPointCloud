#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>

using namespace std;

int
main(int argc, char** argv)
{
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointNormal>);

    if (pcl::io::loadPCDFile<pcl::PointNormal>("*.pcd", *cloud) == -1)
    {
        PCL_ERROR("Could not read file\n");
    }
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("CloudCompare-XYZNormal viewer"));
    viewer->setWindowName("CloudCompare-XYZNormal");
    viewer->addText("CloudCompare-PointNormal", 50, 50, 0, 1, 0, "v1_text");
    viewer->addPointCloud<pcl::PointNormal>(cloud, "CloudCompare-XYZNormal");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "CloudCompare-XYZNormal");
    viewer->addPointCloudNormals<pcl::PointNormal>(cloud, 20, 0.02, "normals");

    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }

    return 0;
}

