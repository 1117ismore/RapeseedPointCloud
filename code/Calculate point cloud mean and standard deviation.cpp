#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>

using namespace std;
#pragma region
void CauculateMeanStd(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointXYZ& Mean, pcl::PointXYZ& Std)
{
    Mean = { 0,0,0 }; Std = { 0,0,0 };
    float X = 0.0, Y = 0.0, Z = 0.0;
    for (auto iter = cloud->begin(); iter != cloud->end(); ++iter)
    {
        X += (*iter).x;
        Y += (*iter).y;
        Z += (*iter).z;
    }
    Mean.x = X / cloud->size();
    Mean.y = Y / cloud->size();
    Mean.z = Z / cloud->size();
    pcl::PointXYZ SquareData{ 0,0,0 };
    for (auto iter = cloud->begin(); iter != cloud->end(); iter++)
    {
        SquareData.x += pow((Mean.x - (*iter).x), 2);
        SquareData.y += pow((Mean.y - (*iter).y), 2);
        SquareData.z += pow((Mean.z - (*iter).z), 2);
    }

    Std.x = sqrt((SquareData.x) / cloud->size());
    Std.y = sqrt((SquareData.y) / cloud->size());
    Std.z = sqrt((SquareData.z) / cloud->size());

}
#pragma endregion

int main(int argc, char** argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("*.pcd", *cloud) == -1)
    {
        PCL_ERROR("Could not read file\n");
    }

    pcl::PointXYZ Mean, Std;
    CauculateMeanStd(cloud, Mean, Std);

    cout << "The mean of each field of point cloud coordinates is£º" << Mean << "The standard deviation is£º" << Std << endl;

    return 0;
}

