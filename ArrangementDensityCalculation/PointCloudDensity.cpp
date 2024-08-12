#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/search/kdtree.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/surface/concave_hull.h>
#include <vector>
#include <fstream>
#include <Eigen/Dense>
#include <iostream>
#include <chrono>
#include <sstream>
#include <chrono>

typedef pcl::PointXYZ PointT;

// 函数用于判断一个点是否在一个多边形内，使用射线交叉法
bool isPointInPolygon(const pcl::PointXYZ& point, const std::vector<pcl::PointXYZ>& polygonVertices) {
    bool inside = false;
    size_t n = polygonVertices.size();
    for (size_t i = 0, j = n - 1; i < n; j = i++) {
        if (((polygonVertices[i].y > point.y) != (polygonVertices[j].y > point.y)) &&
            (point.x < (polygonVertices[j].x - polygonVertices[i].x) * (point.y - polygonVertices[i].y) /
                (polygonVertices[j].y - polygonVertices[i].y) + polygonVertices[i].x)) {
            inside = !inside;
        }
    }
    return inside;
}

// 提取凹包内部的点，这里的内部点是指距离轮廓点云中最近的点的距离超过给定阈值的点
pcl::PointCloud<pcl::PointXYZ>::Ptr extractInternalPoints(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& hull,
    float distance_threshold
) {
    // 创建输出点云
    float dis_threshold = distance_threshold * distance_threshold;
    pcl::PointCloud<pcl::PointXYZ>::Ptr internal_points(new pcl::PointCloud<pcl::PointXYZ>);

    // 创建KD树用于快速查找最近邻
    pcl::search::KdTree<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(hull);

    // 遍历点云中的每个点，找到它们到轮廓点云的最近距离
    for (const auto& point : cloud->points) {
        std::vector<int> nearest_k_indices(1);
        std::vector<float> nearest_k_sqr_distances(1);
        kdtree.nearestKSearch(point, 1, nearest_k_indices, nearest_k_sqr_distances);

        // 如果最近距离大于阈值，那么这个点被认为是内部点
        if (nearest_k_sqr_distances[0] > dis_threshold) {
            internal_points->push_back(point);
        }
    }

    return internal_points;
}

/*
pcl::ConcaveHull

该类是PCL库中用于生成点云数据的凸壳的类。在点云处理中，凸壳（convex hull）是一种常见的处理方式，它可以将点云数据包围在一个封闭的外壳中，该外壳完全由凸面体构成。这个类可以生成一个点云的凸壳，同时还能保留凹面部分，这在某些应用场景下是非常有用的。该算法的原理是先根据输入点云创建一个Delaunay三角剖分（Delaunay triangulation），再通过迭代逐渐减小三角剖分的大小，将凹陷部分的点逐渐融合，直到最后形成一个平滑的封闭曲面。该算法相对于传统的凸壳生成算法，具有更好的形状逼近能力和准确性。
pcl::ConvexHull

该类用于生成点云的凸包。凸包是一个封闭的凸多边形或凸多面体，它是包含给定点集的最小凸体积或凸面积的集合。在计算机视觉和机器人领域，凸包广泛应用于物体识别、图像分析和机器人路径规划等任务中。

pcl::ConvexHull实现的主要方法是QuickHull算法，这是一种非常快速和有效的凸包生成算法。该算法的主要思想是将点云分成两部分：凸包表面和不在凸包表面上的点。然后在凸包表面上找到最远的点，以构建一个新的凸包表面。不断重复这个过程，直到找到所有在凸包表面上的点。QuickHull算法的时间复杂度为O(n log n)，其中n是点云的大小。

该类的主要函数是pcl::ConvexHull::reconstruct()，它接受一个输入点云并生成一个凸包。在实现过程中，该类还包括一些辅助函数，如计算点与凸包表面之间的距离、计算凸包的面积和体积等。此外，该类还提供了一些参数来调整凸包生成的精度和效率，如最大迭代次数、面积阈值和体积阈值等。

pcl::ConvexHull和pcl::ConcaveHull的区别与联系

pcl::ConvexHull和pcl::ConcaveHull都是PCL库中用于点云表面重建的工具，但是它们有几个重要的区别：

几何形状：ConvexHull生成的是凸壳（convex hull），这意味着所有生成的面都是凸的，而ConcaveHull则生成的是凹壳（concave hull），这意味着生成的面可以是凸的也可以是凹的。因此，ConcaveHull可以更准确地保留点云表面的细节。
算法复杂度：ConvexHull的算法复杂度通常比ConcaveHull低，因为凸壳的生成算法相对简单。而ConcaveHull的算法复杂度通常更高，因为需要考虑点云表面的曲率变化等细节。
参数设置：ConvexHull通常只需要设置一个参数，即构建凸壳的方法（例如Qhull或者OpenCV的convexHull）。而ConcaveHull则需要设置更多的参数，例如曲率阈值、网格密度等，以控制生成凹壳的质量和细节。
应用场景：由于ConcaveHull可以更准确地保留点云表面的细节，因此通常用于需要高精度表面重建的应用场景，例如三维建模、虚拟现实等。而ConvexHull则通常用于需要快速简化点云的应用场景，例如碰撞检测、体积计算等。

*/



void statPointCloudDensity(
    std::string path,
    std::string filename,
    float alpha,
    float distance_threshold,
    float radius = 2.0
) {
 
    const float pai = 3.1415926535;
    // 加载点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(path + filename, *cloud) == -1) {
        PCL_ERROR("Error while reading the point cloud file!\n");
        return;
    }

    // 构建凹包
    pcl::PointCloud<pcl::PointXYZ>::Ptr hull(new pcl::PointCloud<pcl::PointXYZ>);
    //pcl::ConvexHull<pcl::PointXYZ> chull;
    pcl::ConcaveHull<pcl::PointXYZ> chull;
    //float alpha = 2.5; // 适当选择一个正值
    chull.setAlpha(alpha);
    chull.setInputCloud(cloud);
    chull.reconstruct(*hull);

    // 保存凹包轮廓点云
    if (pcl::io::savePCDFileBinary(path + "hull_.pcd" , *hull) == -1) {
        PCL_ERROR("Error saving hull point cloud.\n");
    }

    // 提取凹包内部的点
    pcl::PointCloud<pcl::PointXYZ>::Ptr internal_points = extractInternalPoints(cloud, hull, distance_threshold);

    // 创建KD树以进行邻近搜索
    pcl::search::KdTree<pcl::PointXYZ> tree;
    //tree.setInputCloud(internal_points);
    tree.setInputCloud(cloud);

    // 计算点密度
    // 
 
    std::vector<int> densities(internal_points->size(), 0);
    for (size_t i = 0; i < internal_points->size(); ++i) {
        std::vector<int> indices;
        std::vector<float> distances_squared;
        tree.radiusSearch(i, radius, indices, distances_squared);

        densities[i] = indices.size();

    }

    // 存储结果
    std::ofstream outputFile(path + "density_results.txt");
    if (!outputFile) {
        PCL_ERROR("Error opening output file.\n");
        return;
    }

    // 计算平均邻居数量
    double totalNeighbors = 0;
    for (size_t i = 0; i < densities.size(); ++i) {
        totalNeighbors += densities[i];
    }
    double averageNeighbors = totalNeighbors / densities.size();


    // 计算点云密度rou=num/paiR^2还是rou=num/（4/3paiR^3)
    
    float rou = averageNeighbors / (pai * radius * radius);

    // 计算点云理论距离d=sqrt（1/rou）
    //首先，我们需要理解圆的面积公式： A = pai*r ^ 2
    //    如果圆覆盖了n个格点，那么可以假设这些格点在圆内分布得相当均匀。因此，每个格点占据的面积可以近似等于整个圆的面积除以n： Agrid = pair ^ 2 / n
    //    由于我们假设格点均匀分布在网格上，那么每个格点所占据的空间也可以看作是一个正方形，其面积为Agrid​。设正方形边长为d，则有： d ^ 2 = Agrid = pair2n​
    //    从上式可以解出d： d = sqrt(pair ^ 2 / n)

    float dis = sqrt(1 / rou);

    std::cout << "Average number of neighbors within " << radius << " millimeters: " << averageNeighbors << "\n";
    std::cout << "Average density of point cloud " << rou << "\n";
    
    std::cout << "Theoretical distance of point cloud " << dis << " millimeters: " << "\n";

    outputFile << averageNeighbors <<"\t"<< rou<<"\t"<< dis << "\n";



    // 另外可以保存过滤后的点云
    if (pcl::io::savePCDFileBinary(path + "filtered_.pcd" , *internal_points) == -1) {
        PCL_ERROR("Error saving filtered point cloud.\n");
    }

    outputFile.close();
}

int main() {
    auto start = std::chrono::high_resolution_clock::now();

    std::string path = path to data";
    std::string filename = "testClouddata.pcd";

    statPointCloudDensity(path, filename,2.5,10.0, 4.0);
    auto endstatPointCloudDensity = std::chrono::high_resolution_clock::now();
    elapsed = endstatPointCloudDensity-start ;
    std::cout << "Time taken: " << elapsed.count() << " s" << std::endl;
    
    return 0;
}
