#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/random.h>
#include <pcl/common/generate.h> // Generate a Gaussian distributed point cloud
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>

using namespace std;

#pragma region// visualization
void CloudViewer(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& gauss_cloud)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("ShowCloud"));
	viewer->setWindowName("Add Gaussian noise to the point cloud");
	int v1(0);
	viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
	viewer->setBackgroundColor(0, 0, 0, v1);
	viewer->addText("Raw point clouds", 10, 10, "v1_text", v1);
	int v2(0);
	viewer->createViewPort(0.5, 0.0, 1, 1.0, v2);
	viewer->setBackgroundColor(0.1, 0.1, 0.1, v2);
	viewer->addText("gauss point clouds", 10, 10, "v2_text", v2);

	viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud", v1);
	viewer->addPointCloud<pcl::PointXYZ>(gauss_cloud, "gauss", v2);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "sample cloud", v1);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "gauss", v2);
	//viewer->addCoordinateSystem(1.0);
	//viewer->initCameraParameters();
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
}
#pragma endregion

int main(int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	if (pcl::io::loadPCDFile<pcl::PointXYZ>("*.pcd", *cloud) == -1)
	{
		PCL_ERROR("Couldn't read pcd file \n");
		return (-1);
	}

	float xmean = 0, ymean = 0, zmean = 0;
	float xstddev = 0, ystddev = 0, zstddev = 0;
	pcl::PointCloud<pcl::PointXYZ>::Ptr gauss_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::common::CloudGenerator<pcl::PointXYZ, pcl::common::NormalGenerator<float> > generator;
	uint32_t seed = static_cast<uint32_t> (time(NULL));
	pcl::common::NormalGenerator<float>::Parameters x_params(xmean, xstddev, seed++);
	generator.setParametersForX(x_params);
	pcl::common::NormalGenerator<float>::Parameters y_params(ymean, ystddev, seed++);
	generator.setParametersForY(y_params);
	pcl::common::NormalGenerator<float>::Parameters z_params(zmean, zstddev, seed++);
	generator.setParametersForZ(z_params);
	generator.fill((*cloud).width, (*cloud).height, *gauss_cloud);
	for (size_t i = 0; i < cloud->points.size(); ++i)
	{
		gauss_cloud->points[i].x += cloud->points[i].x;
		gauss_cloud->points[i].y += cloud->points[i].y;
		gauss_cloud->points[i].z += cloud->points[i].z;
	}
	printf("Gaussian noise added! ");
	
	pcl::io::savePCDFileBinary("2022-8-6-“∂∆¨÷ÿΩ®//2015.1.7_huayou1_downsampling-1.2_noise_0.pcd", *gauss_cloud);
	
	CloudViewer(cloud, gauss_cloud);

	return 0;
}

