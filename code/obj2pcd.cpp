#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/io/obj_io.h>
#include <pcl/PolygonMesh.h>
#include <pcl/io/vtk_lib_io.h>//loadPolygonFileOBJ

int main()
{

    pcl::PolygonMesh meshData;
    pcl::io::loadPolygonFile("*.obj", meshData);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromPCLPointCloud2(meshData.cloud, *cloud);//Convert point cloud data to mesh data
    pcl::io::savePCDFileASCII("*.pcd", *cloud);
    return 0;
}

