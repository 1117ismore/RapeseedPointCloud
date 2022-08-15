#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <boost/thread/thread.hpp>
#include <pcl/filters/convolution_3d.h>
#include <pcl/visualization/pcl_visualizer.h>

using namespace std;
int main()
{

	//------------------��������------------------------
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

	if (pcl::io::loadPCDFile("2022-7-25-LA����ʵ��//Data//2020.11.6����Ͳ�_40.pcd", *cloud) == -1)
	{
		PCL_ERROR("Couldn't read file pcd\n");
		return(-1);
	}

	//-----------���ڸ�˹�˺����ľ���˲�ʵ��------------------------
	pcl::filters::GaussianKernel<pcl::PointXYZ, pcl::PointXYZ> kernel;
	kernel.setSigma(4);//��˹�����ı�׼������������Ŀ��
	kernel.setThresholdRelativeToSigma(4);//�������Sigma�����ľ�����ֵ
	kernel.setThreshold(0.05);//���þ�����ֵ���������������ֵ���迼��
	cout << "Kernel made" << endl;

	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(cloud);
	cout << "KdTree made" << endl;

	//---------����Convolution ��ز���---------------------------
	pcl::filters::Convolution3D<pcl::PointXYZ, pcl::PointXYZ, pcl::filters::GaussianKernel<pcl::PointXYZ, pcl::PointXYZ>> convolution;
	convolution.setKernel(kernel);//���þ����
	convolution.setInputCloud(cloud);
	convolution.setNumberOfThreads(8);
	convolution.setSearchMethod(tree);
	convolution.setRadiusSearch(0.01);
	cout << "Convolution Start" << endl;

	convolution.convolve(*cloud_filtered);
	pcl::io::savePCDFileASCII("2022-7-25-LA����ʵ��//Data//GS.pcd", *cloud_filtered);
	//--------------------��ʾ����--------------------------
	boost::shared_ptr<pcl::visualization::PCLVisualizer> view(new pcl::visualization::PCLVisualizer("ShowCloud"));

	int v1(0);
	view->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
	view->setBackgroundColor(0, 0, 0, v1);
	view->addText("Raw point clouds", 10, 10, "v1_text", v1);
	int v2(0);
	view->createViewPort(0.5, 0.0, 1, 1.0, v2);
	view->setBackgroundColor(0.1, 0.1, 0.1, v2);
	view->addText("filtered point clouds", 10, 10, "v2_text", v2);

	view->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud", v1);
	view->addPointCloud<pcl::PointXYZ>(cloud_filtered, "cloud_filtered", v2);
	view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "sample cloud", v1);
	view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "cloud_filtered", v2);
	//view->addCoordinateSystem(1.0);
	//view->initCameraParameters();
	while (!view->wasStopped())
	{
		view->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}

	return 0;
}
