#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/kdtree.h>
#include <pcl/surface/mls.h>
#include <pcl/visualization/pcl_visualizer.h> 
#include <boost/thread/thread.hpp>

int
main(int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::io::loadPCDFile("*.pcd", *cloud);
	cout << "points size is:" << cloud->size() << endl;

	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	
	pcl::PointCloud<pcl::PointNormal>::Ptr mls_points_normal(new pcl::PointCloud<pcl::PointNormal>);
	// Definition object (the second type of definition is to store normals, even if it is not used, it needs to be defined)
	pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
	mls.setInputCloud(cloud);
	mls.setComputeNormals(true);     // Whether to calculate the normal, set to true to calculate the normal
	mls.setPolynomialFit(true);      // Set to true to use polynomial fitting during smoothing to improve accuracy
	mls.setPolynomialOrder(2);       // Set the order of the MLS fitting, the default is 2
	mls.setSearchMethod(tree);       
	mls.setSearchRadius(0.005);     
	mls.setNumberOfThreads(4);       // Set the number of threads for multithreading acceleration
	mls.process(*mls_points_normal); 
	cout << "mls poits size is: " << mls_points_normal->size() << endl;

	//pcl::io::savePCDFile ("mls.pcd", mls_points_normal);

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("CloudShow"));
	viewer->setBackgroundColor(0, 0, 0);
	viewer->setWindowName("MLS calculates point cloud normal vector and displays it");
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> v(mls_points_normal, 0, 255, 0);
	viewer->addPointCloud<pcl::PointNormal>(mls_points_normal, v, "point");
	viewer->addPointCloudNormals<pcl::PointNormal>(mls_points_normal, 10, 0.1, "normal");

	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(1000));
	}

	return 0;
}

