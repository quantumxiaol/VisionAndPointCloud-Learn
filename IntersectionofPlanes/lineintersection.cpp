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
#include <vector>
#include <fstream>
#include <Eigen/Dense>
#include <iostream>
#include <chrono>
#include <sstream>
typedef pcl::PointXYZ PointT;
struct CloudData {
    float x, y, z;
};
// Function to calculate the intersection line of two planes
/*
首先从给定的平面方程（以齐次坐标表示）中提取出法向量normal1和normal2。
然后，计算这两个法向量的叉乘得到交线的方向向量direction。
如果方向向量的范数为零，这意味着两个平面平行或重合，因此没有唯一的交线，函数返回false。
接下来，创建一个3x3矩阵，其中包含两个法向量和平行于交线的方向向量。
计算常数向量constants，其中包含了平面方程中的常数项（以负号表示）和一个零元素（对应于交线方向向量）。
使用QR分解求解矩阵方程Ax = b，其中A是包含法向量和方向向量的矩阵，b是常数向量。这将给出位于交线上的点point。
将计算出的点赋值给linePoint，并将方向向量规范化后赋值给lineDirection。
*/
bool computePlaneIntersection(
    const Eigen::Vector4f& plane1, //平面方程1
    const Eigen::Vector4f& plane2, //平面方程2
    Eigen::Vector3f& linePoint, //交线上的一个点
    Eigen::Vector3f& lineDirection//交线的方向向量
) {
    Eigen::Vector3f normal1(plane1[0], plane1[1], plane1[2]);
    Eigen::Vector3f normal2(plane2[0], plane2[1], plane2[2]);
    // direction 向量是两个平面法线向量的叉积，是两平面相交时交线的方向。
    Eigen::Vector3f direction = normal1.cross(normal2);
    // 向量 direction 的范数（即长度或模），如果范数为零，两个平面要么完全平行（没有交线），
    // 要么实际上重合为同一平面（有无数条交线，但无法定义一个唯一的交线方向），则输出一条错误消息并返回 false。
    if (direction.norm() == 0) {
        std::cerr << "Planes are parallel or coincident, no unique intersection line." << std::endl;
        return false;
    }
    /*
    Eigen::Matrix3f matrix;
    matrix << normal1, normal2, direction;
    Eigen::Vector3f constants(-plane1[3], -plane2[3], 0);
    // 解线性方程组 Ax = b，其中 A 是由 matrix 表示的矩阵，b 是由 constants 表示的向量。
    // 使用 Eigen 库中的 colPivHouseholderQr() 方法来计算 A 的 QR 分解，并求解线性方程组。
    // 结果存储在 point 向量中，point 即是方程组的解。
    Eigen::Vector3f point = matrix.colPivHouseholderQr().solve(constants);
    */

    //下面寻找交线上一个点，
    //直线上的点同时满足a1*x + b1*y + c1*z + d1 = 0 和 a2*x + b2*y + c2*z + d2 = 0
    //不妨假定 x = 0 一点为（0 , y0 , z0）
    //y0 , z0通过解方程组b1*y + c1*z + d1 = 0 和 b2*y + c2*z + d2 = 0 获得
    //事实上 z = -(b2*d1-b1*d2)/(b2*c1-b1*c2)
    //y = (d1*(c1*b2-b1*c2))/(b1*(b2c1-b1c2))


    // 找到可以设为0的坐标轴
    int axisToZero = -1;
    if (std::abs(normal2[0]) > std::numeric_limits<float>::epsilon()) {
        axisToZero = 0; // x-axis
    }
    else if (std::abs(normal2[1]) > std::numeric_limits<float>::epsilon()) {
        axisToZero = 1; // y-axis
    }
    else if (std::abs(normal2[2]) > std::numeric_limits<float>::epsilon()) {
        axisToZero = 2; // z-axis
    }

    if (axisToZero == -1) {
        std::cerr << "Normal vector is too close to zero in all components, cannot determine intersection." << std::endl;
        return false;
    }

    // 根据所选轴设置线性方程组
    Eigen::Matrix2f subMatrix;
    Eigen::Vector2f subConstants;

    switch (axisToZero) {
    case 0: // 如果 x=0
        subMatrix << normal1[1], normal1[2],
            normal2[1], normal2[2];
        subConstants << -plane1[3], -plane2[3];
        break;
    case 1: // 如果 y=0
        subMatrix << normal1[0], normal1[2],
            normal2[0], normal2[2];
        subConstants << -plane1[3], -plane2[3];
        break;
    case 2: // 如果 z=0
        subMatrix << normal1[0], normal1[1],
            normal2[0], normal2[1];
        subConstants << -plane1[3], -plane2[3];
        break;
    }

    // 求解线性方程组
    Eigen::Vector2f solution;
    // QR 分解将矩阵 分解为一个正交矩阵和一个上三角矩阵的乘积
    // 执行带有列主元（column pivoting）的Householder QR分解。列主元是指在分解过程中，矩阵的列可能会被重新排序，以增加数值稳定性
    //Eigen::Vector2f solution = subMatrix.colPivHouseholderQr().solve(subConstants);

    // LU分解将矩阵 分解为一个下三角矩阵和一个上三角矩阵的乘积
    Eigen::FullPivLU<Eigen::Matrix2f> luDecomposition(subMatrix);

    // 检查矩阵是否可逆（即分解是否成功）
    if (luDecomposition.isInvertible()) {
        // 使用 LU 分解求解线性方程组
        solution = luDecomposition.solve(subConstants);
        // solution 现在包含了解方程组的解
    }
    else {
        std::cerr << "The matrix is singular and cannot be inverted." << std::endl;
    }


    // 构建交线上的一点
    switch (axisToZero) {
    case 0:
        linePoint << 0, solution[0], solution[1];
        break;
    case 1:
        linePoint << solution[0], 0, solution[1];
        break;
    case 2:
        linePoint << solution[0], solution[1], 0;
        break;
    }

    lineDirection = direction.normalized();
    return true;
}


// Function to segment planes in the point cloud and compute the intersection line
/*
* 随机采样一致性
首先，创建一个SACSegmentation对象seg，设置模型类型为SACMODEL_PLANE，方法类型为SAC_RANSAC，最大迭代次数为50000，距离阈值为12。
然后，创建一个指向输入点云的指针remainingCloud，一个指向点云中的内点的指针inliers，一个指向模型系数的指针coefficients。
接下来，对于每个平面，调用SACSegmentation::segment()方法，提取出内点和模型系数。
如果内点的数量小于100，说明无法估计给定数据集的平面模型，函数返回。
将模型系数存储到planeCoefficients中，并创建一个ExtractIndices对象extract，设置输入点云为remainingCloud，内点为inliers。
调用extract.filter()方法，将内点从remainingCloud中提取出来。
如果planeCoefficients的大小小于2，说明在点云中找不到至少两个平面，函数返回。
调用computePlaneIntersection()函数，计算两个平面的交线，将结果存储到linePoint和lineDirection中。
*/

//返回交线在点云上的端点
void IntersectionPoint(
    Eigen::Vector3f& linePoint,//交线上的一个点
    Eigen::Vector3f& lineDirection,//交线的方向向量
    Eigen::Vector3f endPointMin,//交线的起点
    Eigen::Vector3f endPointMax,//交线的终点
    float RadiusSearch = 2.0,//半径滤波的半径
    int MinNeighborsInRadius = 8,//半径滤波的邻居数
    std::string outputDir = "C:\\work\\test1\\"//输出路径
) {
    //打开第二的点云，对应斜板面
    pcl::PointCloud<PointT>::Ptr cloud2(new pcl::PointCloud<PointT>());
    std::stringstream ss;
    ss << outputDir << "output_ascii_" << std::setw(4) << std::setfill('0') << 1 << ".pcd";
    //pcl::io::loadPCDFile<pcl::PointXYZI>(ss, *cloud2);
    if (pcl::io::loadPCDFile<PointT>(ss.str(), *cloud2) == -1)
    {
        PCL_ERROR("Error while reading the point cloud file!\n");

    }
    //将点云中的点投射到直线上

    //半径滤波，滤去离群点，可能会导致交线长度减少，和RadiusSearch正相关
    pcl::RadiusOutlierRemoval<PointT> outrem;
    outrem.setInputCloud(cloud2);
    outrem.setRadiusSearch(RadiusSearch);
    outrem.setMinNeighborsInRadius(MinNeighborsInRadius);
    outrem.setKeepOrganized(true);

    pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
    outrem.filter(*cloud_filtered);
    cloud2.swap(cloud_filtered);

    // 投影点云到交线上
    std::vector<Eigen::Vector3f> projectedPoints;
    for (const auto& point : cloud2->points)
    {
        Eigen::Vector3f pointVec(point.x, point.y, point.z);
        // 投影点到直线
        Eigen::Vector3f projectedPoint = linePoint + ((pointVec - linePoint).dot(lineDirection)) * lineDirection;
        projectedPoints.push_back(projectedPoint);
    }

    // 计算投影点的极值
    float minProj = std::numeric_limits<float>::max();
    float maxProj = std::numeric_limits<float>::lowest();
    for (const auto& projPoint : projectedPoints)
    {
        float projLength = (projPoint - linePoint).dot(lineDirection);
        minProj = std::min(minProj, projLength);
        maxProj = std::max(maxProj, projLength);
    }

    // 确定线段端点
    endPointMin = linePoint + minProj * lineDirection;
    endPointMax = linePoint + maxProj * lineDirection;

    std::cout << endPointMin << std::endl << endPointMax << std::endl << std::endl;

    //储存结果到txt
    // 写入文本文件
    std::stringstream si;
    std::string filePath = "result.txt"; // 输出文件路径
    si << outputDir << "output_ascii_" << filePath;

    // 使用 ofstream 将向量写入文件
    std::ofstream outputFile(si.str());
    if (outputFile.is_open()) {
        // 写入向量
        outputFile << endPointMin(0) << " " << endPointMin(1) << " " << endPointMin(2) << std::endl;
        outputFile << endPointMax(0) << " " << endPointMax(1) << " " << endPointMax(2) << std::endl;
        outputFile.close();
    }
    else {
        std::cerr << "Unable to open file: " << filePath << std::endl;
    }
    
}

//计算平面交点
void PlanesIntersection(
    const pcl::PointCloud<PointT>::Ptr& inputCloud,//待处理点云
    std::vector<Eigen::Vector4f>& planeCoefficients,//平面方程系数ax+by+cz+d=0
    Eigen::Vector3f& linePoint,//交线上的一个点
    Eigen::Vector3f& lineDirection,//交线的方向向量
    Eigen::Vector3f endPointMin,//交线的起点
    Eigen::Vector3f endPointMax,//交线的终点
    int numPlanes = 2,//要拆分的平面数量，两平面才有交线
    float distanceThreshold = 12.0f,//距离阈值
    float RadiusSearch=2.0,//半径滤波的半径
    int MinNeighborsInRadius=8,//半径滤波的邻居数
    int maxIterations = 50000,//计算平面最大迭代次数
    std::string outputDir //输出路径

)
{
    pcl::SACSegmentation<PointT> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    //随机采样出一部分点。
    //使用这些点来估计一个模型，估计的方法一般为最小二乘法
    //将所有点带入这个模型，如果该点到当前模型的误差小于阈值，则将该点记为内点。
    //用所有内点重新估计一次模型
    //比较当前模型和之前推出的最好的模型的内点的数量，记录最大内点数的模型参数和内点数。
    //重复1 - ５步，直到迭代结束或者当前模型的内点数大于一定的阈值。
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);
    //auto remainingCloud = std::make_shared<pcl::PointCloud<PointT>>();
    pcl::PointCloud<PointT>::Ptr remainingCloud(new pcl::PointCloud<PointT>(*inputCloud));
    //remainingCloud(inputCloud);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());

    for (int i = 0; i < numPlanes; ++i) {
        pcl::PointCloud<PointT>::Ptr cloud_plane(new pcl::PointCloud<PointT>());
        pcl::PointCloud<PointT>::Ptr cloud_filter(new pcl::PointCloud<PointT>());
        seg.setInputCloud(remainingCloud);
        seg.segment(*inliers, *coefficients);
        if (inliers->indices.size() < 100) {
            std::cerr << "Could not estimate a planar model for the given dataset. Number of inliers: " << inliers->indices.size() << std::endl;
            return;
        }
        planeCoefficients.push_back(Eigen::Vector4f(coefficients->values[0], coefficients->values[1], coefficients->values[2], coefficients->values[3]));

        pcl::ExtractIndices<PointT> extract;
        extract.setInputCloud(remainingCloud);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*cloud_plane);

        //储存点云
        std::stringstream ss;
        ss << outputDir << "output_ascii_" << std::setw(4) << std::setfill('0') << i << ".pcd";
        pcl::io::savePCDFileASCII(ss.str(), *cloud_plane);

        //提取除了指定索引之外的点云数据。然后，extract.filter(*cloud)将应用过滤器并将结果存储在cloud变量中，即过滤掉了两个平面之间的点云数据。
        extract.setNegative(true);
        extract.filter(*cloud_filter);

        remainingCloud.swap(cloud_filter);
        //释放cloud_filter
        cloud_filter.reset(new pcl::PointCloud<PointT>());


    }
    //释放内存
    remainingCloud.reset(new pcl::PointCloud<PointT>());

    if (planeCoefficients.size() < 2) {
        std::cerr << "Could not find at least two planes in the point cloud." << std::endl;
        return;
    }

    if (computePlaneIntersection(planeCoefficients[0], planeCoefficients[1], linePoint, lineDirection)) {
        std::cout << "Intersection line: point(" << linePoint.x() << ", " << linePoint.y() << ", " << linePoint.z() << "), direction("
            << lineDirection.x() << ", " << lineDirection.y() << ", " << lineDirection.z() << ")" << std::endl;
    }
    else {
        std::cerr << "Error computing the intersection line." << std::endl;
    }
    std::cout << "segment process end" << std::endl;
    /*
    * 当调用 reset() 或出作用域后自动析构 pcl::PointCloud::Ptr 时，出现内存异常
    * 是Eigen Memory.h中的handmade_aligned_free
    * 0x00007FFB52E55BB6 (ntdll.dll)处(位于 PCL_test.exe 中)引发的异常: 0xC0000005: 读取位置 0xFFFFFFFFFFFFFFFF 时发生访问冲突
    * 程序中某个指针没有被正确初始化，或者指向了一个已经被释放的内存区域，或者越界访问数组时
    这里内存有问题，有一些解决方法：
        （1）VS2019可以通过项目属性->C/C++->代码生成->启动增强指令集->选择AVX来解决；
        （2）qt 中在pro文件添加 QMAKE_CXXFLAGS += /arch:AVX；
        （3）工程是用CMake构建的，原因可能是开启了-march=native的CPU指令集优化，造成该变量被自动释放，
        而share_ptr去寻找并尝试析构的时候报错，解决方案，关闭该优化即可；
        （4）而在VS2022中，要解决这个内存释放问题，则需要改成 VS：通过项目属性->C/C++->代码生成->启用增强指令集->未设置
    */

    IntersectionPoint(linePoint, lineDirection, endPointMin, endPointMax, RadiusSearch, MinNeighborsInRadius, outputDir);
    /*
    //打开第二的点云，对应斜板面
    pcl::PointCloud<PointT>::Ptr cloud2(new pcl::PointCloud<PointT>());
    std::stringstream ss;
    ss << outputDir << "output_ascii_" << std::setw(4) << std::setfill('0') << 1 << ".pcd";
    //pcl::io::loadPCDFile<pcl::PointXYZI>(ss, *cloud2);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(ss.str(), *cloud2) == -1)
    {
        PCL_ERROR("Error while reading the point cloud file!\n");

    }
    //将点云中的点投射到直线上

    //半径滤波，滤去离群点，可能会导致交线长度减少，和RadiusSearch正相关
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
    outrem.setInputCloud(cloud2);
    outrem.setRadiusSearch(RadiusSearch);
    outrem.setMinNeighborsInRadius(MinNeighborsInRadius);
    outrem.setKeepOrganized(true);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    outrem.filter(*cloud_filtered);
    cloud2.swap(cloud_filtered);

    // 投影点云到交线上
    std::vector<Eigen::Vector3f> projectedPoints;
    for (const auto& point : cloud2->points)
    {
        Eigen::Vector3f pointVec(point.x, point.y, point.z);
        // 投影点到直线
        Eigen::Vector3f projectedPoint = linePoint + ((pointVec - linePoint).dot(lineDirection)) * lineDirection;
        projectedPoints.push_back(projectedPoint);
    }

    // 计算投影点的极值
    float minProj = std::numeric_limits<float>::max();
    float maxProj = std::numeric_limits<float>::lowest();
    for (const auto& projPoint : projectedPoints)
    {
        float projLength = (projPoint - linePoint).dot(lineDirection);
        minProj = std::min(minProj, projLength);
        maxProj = std::max(maxProj, projLength);
    }

    // 确定线段端点
    endPointMin = linePoint + minProj * lineDirection;
    endPointMax = linePoint + maxProj * lineDirection;

    std::cout << endPointMin << std::endl << endPointMax << std::endl << std::endl;
    
    //储存结果到txt
    // 写入文本文件
    std::stringstream si;
    std::string filePath = "result.txt"; // 输出文件路径
    si << outputDir << "output_ascii_" << filePath;

    // 使用 ofstream 将向量写入文件
    std::ofstream outputFile(si.str());
    if (outputFile.is_open()) {
        // 写入向量
        outputFile << endPointMin(0) << " " << endPointMin(1) << " " << endPointMin(2) << std::endl;
        outputFile << endPointMax(0) << " " << endPointMax(1) << " " << endPointMax(2) << std::endl;
        outputFile.close();
    }
    else {
        std::cerr << "Unable to open file: " << filePath << std::endl;
    }
    */
}


int main()
{

    // ... fill cloud with data ...
        // Read in the cloud data
    std::ifstream file("path to cloud input");
    if (!file) {
        std::cerr << "Error: Could not open the file." << std::endl;
        return -1;
    }
    std::string outpath = "path to cloud output";
    // 每个点有3个float值表示坐标
    const size_t pointSize = sizeof(CloudData);
    std::vector<char> buffer(pointSize * 100000); // 假设点云数据最多有1000个点

    file.read(&buffer[0], static_cast<long>(buffer.size()));
    int number = 0;
    if (!file) {
        std::cerr << "Error: Could not read the entire file." << std::endl;
        return -1;
    }

    pcl::PointXYZ p;
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
    std::vector<Eigen::Vector4f> planeCoefficients;
    std::vector<Eigen::Vector4f> planeCoefficients_Multi;
    Eigen::Vector3f linePoint, lineDirection;
    Eigen::Vector3f endPointMin;
    Eigen::Vector3f endPointMax;
    //cloud->reserve(100000); // 预留足够的空间以避免多次重新分配内存
    std::string line;
    std::getline(file, line);
    float pass;
//
    int i = 0;
    while (file >> p.x >> p.y >> p.z >> pass) {
        cloud->push_back(p);
        number++;
        //if (i == 0) { cout << p.x << " " << p.y << " " << p.z << endl; i++; }
    }
    std::cout<< "number of points: " << number << std::endl;
    auto start = std::chrono::high_resolution_clock::now();
    PlanesIntersection(cloud, planeCoefficients, linePoint, lineDirection, endPointMin, endPointMax, 2, 9.0,4.0,4, 8000, outpath);

    auto end = std::chrono::high_resolution_clock::now();


    std::chrono::duration<double> elapsed = end - start;
    std::cout << "Time taken: " << elapsed.count() << " s" << std::endl;

    return 0;
}