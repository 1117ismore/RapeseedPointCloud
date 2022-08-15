#include <iostream>
#include <sstream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

using namespace std;

int
main(int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	
	pcl::PCDReader reader;
	reader.read<pcl::PointXYZ>("*.pcd", *cloud);

	cout << "PointCloud before filtering: " << cloud->width * cloud->height
		<< " data points (" << pcl::getFieldsList(*cloud) << ")." << endl;

	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud);
	sor.setLeafSize(0.002f, 0.002f, 0.002f); //Set the size of the grid voxels
	pcl::PointCloud<pcl::PointXYZ> ::Ptr voxel_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	sor.filter(*voxel_filtered);
	//-----------KNN------------
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(cloud);
	pcl::PointIndicesPtr inds = std::shared_ptr<pcl::PointIndices>(new pcl::PointIndices());
	for (size_t i = 0; i < voxel_filtered->points.size(); i++) {
		pcl::PointXYZ searchPoint;
		searchPoint.x = voxel_filtered->points[i].x;
		searchPoint.y = voxel_filtered->points[i].y;
		searchPoint.z = voxel_filtered->points[i].z;

		int K = 1;
		vector<int> pointIdxNKNSearch(K);
		vector<float> pointNKNSquaredDistance(K);
		if (kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {

			inds->indices.push_back(pointIdxNKNSearch[0]);

		}

	}
	pcl::PointCloud<pcl::PointXYZ>::Ptr final_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud(*cloud, inds->indices, *final_filtered);
	cout << "the number of final downsampling cloud " << final_filtered->points.size() << endl;

	pcl::PCDWriter writer;
	writer.write<pcl::PointXYZ>("*.pcd", *final_filtered, true);

	return (0);
}
