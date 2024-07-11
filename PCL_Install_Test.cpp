#include <iostream> // 包含输入输出流库，用于在控制台打印信息
#include <vector>   // 包含 STL 的 vector 容器类
#include <ctime>    // 包含时间库，用于生成随机数种子
#include <pcl/point_cloud.h> // 包含点云类的定义
#include <pcl/octree/octree.h> // 包含 Octree 数据结构的定义
#include <boost/thread/thread.hpp> // 包含 Boost 线程库
#include <pcl/visualization/pcl_visualizer.h> // 包含点云可视化工具

using namespace std; // 使用标准命名空间，方便使用标准库中的对象

int main(int argc, char** argv)
{
    // 设置随机数种子
    srand((unsigned int)time(NULL));
    
    // 创建一个新的点云对象，使用 pcl::PointXYZ 类型的点
    // pcl::PointCloud<pcl::PointXYZ>::Ptr 是一个指向点云对象的智能指针
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    
    // 初始化点云的宽度和高度
    cloud->width = 1000; // 点云的宽度，即点的数量
    cloud->height = 1;   // 点云的高度，对于一维点云，height 设置为 1
    cloud->points.resize(cloud->width * cloud->height); // 预分配内存以容纳所有点
    
    // 填充点云数据
    for (size_t i = 0; i < cloud->points.size(); ++i)
    {
        // 为每个点生成随机的 x, y, z 坐标
        cloud->points[i].x = 1024.0f * rand() / (RAND_MAX + 1.0f);
        cloud->points[i].y = 1024.0f * rand() / (RAND_MAX + 1.0f);
        cloud->points[i].z = 1024.0f * rand() / (RAND_MAX + 1.0f);
    }
 
    // 创建一个 Octree 对象，用于空间分区和近邻搜索
    // 0.1 是 Octree 的分辨率，即每个 Octree 节点的边长
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(0.1);
    
    // 设置点云数据，并将点云数据添加到 Octree 中
    octree.setInputCloud(cloud);
    octree.addPointsFromInputCloud();
    
    // 创建一个随机搜索点
    pcl::PointXYZ searchPoint;
    searchPoint.x = 1024.0f * rand() / (RAND_MAX + 1.0f);
    searchPoint.y = 1024.0f * rand() / (RAND_MAX + 1.0f);
    searchPoint.z = 1024.0f * rand() / (RAND_MAX + 1.0f);
 
    // 半径内近邻搜索
    vector<int> pointIdxRadiusSearch; // 存储搜索到的点的索引
    vector<float> pointRadiusSquaredDistance; // 存储每个点到搜索点的平方距离
    float radius = 256.0f * rand() / (RAND_MAX + 1.0f); // 随机生成搜索半径
    
    cout << "Neighbors within radius search at (" << searchPoint.x
         << " " << searchPoint.y
         << " " << searchPoint.z
         << ") with radius=" << radius << endl;
    
    // 执行半径搜索，并将结果存储在 pointIdxRadiusSearch 和 pointRadiusSquaredDistance 中
    if (octree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
    {
        // 打印搜索到的邻居点的信息
        for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i)
            cout << "    " << cloud->points[pointIdxRadiusSearch[i]].x
                 << " " << cloud->points[pointIdxRadiusSearch[i]].y
                 << " " << cloud->points[pointIdxRadiusSearch[i]].z
                 << " (squared distance: " << pointRadiusSquaredDistance[i] << ")" << endl;
    }
    
    // 初始化点云可视化对象
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("显示点云"));
    
    // 设置背景颜色为黑色
    viewer->setBackgroundColor(0, 0, 0);
    
    // 创建点云的颜色处理器，并将点云显示为红色
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_color(cloud, 255, 0, 0);
    
    // 将点云添加到可视化对象中
    viewer->addPointCloud<pcl::PointXYZ>(cloud, target_color, "target cloud");
    
    // 设置点云的点大小
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "target cloud");
 
    // 循环直到可视化窗口关闭
    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100); // 处理可视化窗口的事件
        boost::this_thread::sleep(boost::posix_time::microseconds(1000)); // 休眠 1 毫秒
    }
 
    return 0; // 程序正常结束
}
