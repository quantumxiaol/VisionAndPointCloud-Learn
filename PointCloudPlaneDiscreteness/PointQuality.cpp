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


float calculateVariance(const std::vector<float>& numbers) {
    float mean = std::accumulate(numbers.begin(), numbers.end(), 0.0) / numbers.size();
    float variance = 0.0;
    for (float num : numbers) {
        variance += (num - mean) * (num - mean);
    }
    variance /= numbers.size();
    return variance;
}

float calculateStandardDeviation(const std::vector<float>& numbers) {
    return sqrt(calculateVariance(numbers));
}

float calculateMean(const std::vector<float>& numbers) {
    return std::accumulate(numbers.begin(), numbers.end(), 0.0) / numbers.size();
}

void PlanePCA(
    std::string path,
    std::string filename,
    std::vector<Eigen::Vector4f>& planeCoefficients,
    float distanceThreshold = 12.0f,//距离阈值
    int maxIterations = 50000//计算平面最大迭代次数

) {
    // 计算质心
    Eigen::Vector4f centroid;
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
    std::stringstream ss;
    //ss << "C:\\work\\cameratest\\Data\\150Cloud.pcd";
    ss << path << filename;
    //pcl::io::loadPCDFile<pcl::PointXYZI>(ss, *cloud2);
    if (pcl::io::loadPCDFile<PointT>(ss.str(), *cloud) == -1)
    {
        PCL_ERROR("Error while reading the point cloud file!\n");
    }
    std::cout << "start" << std::endl;


    pcl::compute3DCentroid(*cloud, centroid);

    // 构建点云矩阵
    Eigen::MatrixXd points(cloud->size(), 3);
    for (size_t i = 0; i < cloud->size(); ++i)
    {
        points(i, 0) = (*cloud)[i].x - centroid[0];
        points(i, 1) = (*cloud)[i].y - centroid[1];
        points(i, 2) = (*cloud)[i].z - centroid[2];
    }

    // 计算协方差矩阵
    Eigen::MatrixXd centered = points.rowwise() - points.colwise().mean();
    Eigen::Matrix3d covariance = centered.transpose() * centered / (points.rows() - 1);

    // 计算特征值和特征向量
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigensolver(covariance);
    Eigen::Matrix3d eigenVectors = eigensolver.eigenvectors();
    Eigen::Vector3d eigenValues = eigensolver.eigenvalues();

    // 选择主轴
    int index1, index2;
    if (eigenValues[0] > eigenValues[1])
    {
        index1 = 0;
        index2 = 1;
    }
    else
    {
        index1 = 1;
        index2 = 0;
    }
    // 确保 index1 指向最大特征值的索引，index2 指向次大特征值的索引
    int maxIndex = 0, secondMaxIndex = 1;
    if (eigenValues[1] > eigenValues[maxIndex]) {
        maxIndex = 1;
    }
    if (eigenValues[2] > eigenValues[maxIndex]) {
        secondMaxIndex = maxIndex;
        maxIndex = 2;
    }
    else if (eigenValues[2] > eigenValues[secondMaxIndex]) {
        secondMaxIndex = 2;
    }

    index1 = maxIndex;
    index2 = secondMaxIndex;

    // 主轴和法向量
    Eigen::Vector3d axis1 = eigenVectors.col(index1);
    Eigen::Vector3d axis2 = eigenVectors.col(index2);
    Eigen::Vector3d normal = axis1.cross(axis2);

    // 平面方程
    Eigen::Vector3f normal3f(normal.x(), normal.y(), normal.z());
    double d = -normal3f.dot(centroid.head<3>());
    std::cout << "Plane equation: " << normal[0] << "x + " << normal[1] << "y + " << normal[2] << "z + " << d << " = 0" << std::endl;

    planeCoefficients.push_back(Eigen::Vector4f(normal[0], normal[1], normal[2], d));
    std::stringstream si;
    std::string filePath = "resultPCA.txt"; // 输出文件路径
    //si <<  "C:\\work\\cameratest\\Data\\" << filePath;
    si << path << filePath;
    std::vector<Eigen::Vector4f> coefficients;
    // 使用 ofstream 将向量写入文件
    std::ofstream outputFile(si.str());
    if (outputFile.is_open()) {
        for (const auto& coeff : planeCoefficients) {
            outputFile << coeff[0] << "\t" << coeff[1] << "\t" << coeff[2] << "\t" << coeff[3] << std::endl;
        }
        outputFile.close();
    }
}


//(1)点云平面离散程度测算
void PointQuality(
    std::string path,
    std::string filename,
    std::vector<Eigen::Vector4f>& planeCoefficients,
    float distanceThreshold = 12.0f,//距离阈值
    int maxIterations = 50000//计算平面最大迭代次数
)
{

pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
std::stringstream ss;
//ss << "C:\\work\\cameratest\\Data\\150Cloud.pcd";
ss << path << filename;
//pcl::io::loadPCDFile<pcl::PointXYZI>(ss, *cloud2);
if (pcl::io::loadPCDFile<PointT>(ss.str(), *cloud) == -1)
{
    PCL_ERROR("Error while reading the point cloud file!\n");
}
std::cout << "start"<<std::endl;
pcl::SACSegmentation<PointT> seg;
seg.setOptimizeCoefficients(true);
seg.setModelType(pcl::SACMODEL_PLANE);
seg.setMethodType(pcl::SAC_LMEDS);
//随机采样出一部分点。
//使用这些点来估计一个模型，估计的方法一般为最小二乘法
//将所有点带入这个模型，如果该点到当前模型的误差小于阈值，则将该点记为内点。
//用所有内点重新估计一次模型
//比较当前模型和之前推出的最好的模型的内点的数量，记录最大内点数的模型参数和内点数。
//重复1 - ５步，直到迭代结束或者当前模型的内点数大于一定的阈值。
seg.setMaxIterations(maxIterations);
seg.setDistanceThreshold(distanceThreshold);
//auto remainingCloud = std::make_shared<pcl::PointCloud<PointT>>();

pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
pcl::PointCloud<PointT>::Ptr cloud_plane(new pcl::PointCloud<PointT>());
pcl::PointCloud<PointT>::Ptr cloud_filter(new pcl::PointCloud<PointT>());
seg.setInputCloud(cloud);
seg.segment(*inliers, *coefficients);
planeCoefficients.push_back(Eigen::Vector4f(coefficients->values[0], coefficients->values[1], coefficients->values[2], coefficients->values[3]));

if (inliers->indices.size() < 100) {
    std::cerr << "Could not estimate a planar model for the given dataset. Number of inliers: " << inliers->indices.size() << std::endl;
    return;
}
//计算点云中所有三维点到计算出的平面距离分布
//ax+by+cz+d=0
std::cout << "(" << coefficients->values[0] << ")*x + (" << coefficients->values[1] << ")*y + (" << coefficients->values[2] << ")*z + (" << coefficients->values[3] << ")" << std::endl;
std::stringstream si;
std::string filePath = "result.txt"; // 输出文件路径
//si <<  "C:\\work\\cameratest\\Data\\" << filePath;
si << path << filePath;

// 使用 ofstream 将向量写入文件
std::ofstream outputFile(si.str());
if (outputFile.is_open()) {
    // 写入向量
    outputFile << coefficients->values[0] << "\t" << coefficients->values[1] << "\t" << coefficients->values[2] << "\t" << coefficients->values[3] << std::endl;
    outputFile.close();
}

}

int statPointCloudDistancetoPlane(
    std::string path,
    std::string filename,
    std::vector<Eigen::Vector4f>& planeCoefficients,
    std::vector<int> &distribute,
    //  0.05  0.10    0.15    0.20    0.25    0.30    0.35    0.40    0.45    0.5   other
    //  0.025 0.010   0.0225  0.040   0.0625  0.090   0.1225  0.16    0.2025  0.25  other
    float interval=0.05
) {
    int i = 0;
    
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
    std::stringstream ss;
    //ss << "C:\\work\\cameratest\\Data\\150Cloud.pcd";
    ss << path << filename;

    //pcl::io::loadPCDFile<pcl::PointXYZI>(ss, *cloud2);
    if (pcl::io::loadPCDFile<PointT>(ss.str(), *cloud) == -1)
    {
        PCL_ERROR("Error while reading the point cloud file!\n");
    }
    std::cout << "start" << std::endl;

    float a = planeCoefficients[0][0];
    float b = planeCoefficients[0][1];
    float c = planeCoefficients[0][2];
    float d = planeCoefficients[0][3];

    float abc = a * a + b * b + c * c;

    float inv = interval;

    // Squared thresholds for comparison
    std::vector<float> thresholds(distribute.size(), 0);
    for (i = 0; i < distribute.size()-1; i++) {
        thresholds[i] = inv * inv * abc;
        inv += interval;
    }
    thresholds[distribute.size() - 1] = std::numeric_limits<float>::max();


    std::vector<float> dist;
    float avgZ = 0;
    float disMax = 0;
    for (const auto& point : cloud->points) {
        float distSquared = (a * point.x + b * point.y + c * point.z + d) * (a * point.x + b * point.y + c * point.z + d);
        avgZ = (avgZ * (i - 1) + point.z) / i;
        disMax = std::max(disMax, distSquared);
        dist.push_back(std::sqrt(distSquared / abc));
        int binIndex = 0;
        while (distSquared > thresholds[binIndex] && binIndex < distribute.size()) {
            binIndex++;
        }
        distribute[binIndex]++;
    }

    //输出分布信息
    for (i = 0; i < distribute.size(); i++)
        std::cout << std::sqrt(thresholds[i]/abc) << "\t";
    std::cout << std::endl;
    for (i = 0; i < distribute.size(); i++)
        std::cout << distribute[i] << "\t";
    std::cout << std::endl;
    std::cout << "点云平均距离" << avgZ << std::endl;
    std::cout << "最大偏差" << std::sqrt(disMax / abc) << std::endl;
    std::cout << "平均偏差" << calculateMean(dist) << std::endl;
    std::cout << "标准偏差" << calculateStandardDeviation(dist) << std::endl;
    std::stringstream si;
    std::string filePath = "distrbute.txt"; // 输出文件路径
    si << path << filePath;

    // 使用 ofstream 将写入文件
    std::ofstream outputFile(si.str());
    if (outputFile.is_open()) {
        // 写入
        for (i = 0; i < distribute.size(); i++)
            outputFile << distribute[i] << "\t";
        outputFile << std::endl;
        outputFile << avgZ << "\t" << std::sqrt(disMax / abc) << "\t" << calculateMean(dist) << "\t" << calculateStandardDeviation(dist);
        outputFile.close();
    }
    return 0;
}

int main() {
    auto start = std::chrono::high_resolution_clock::now();
    std::vector<Eigen::Vector4f> planeCoefficients;

    auto start = std::chrono::high_resolution_clock::now();
    std::string path = "path to data";
    std::string filename = "testClouddata.pcd";

    //PointQuality(path,filename,planeCoefficients,2.7,600);
    PlanePCA(path, filename, planeCoefficients, 2.7, 600);


    auto endPointQuality = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = endPointQuality - start;
    std::cout << "Time taken: " << elapsed.count() << " s" << std::endl;

    std::vector<int> distrbute(11,0);
    statPointCloudDistancetoPlane(path, filename, planeCoefficients, distrbute,0.3);
    auto endstatPointCloudDistancetoPlane = std::chrono::high_resolution_clock::now();
    elapsed = endstatPointCloudDistancetoPlane - endPointQuality ;
    std::cout << "Time taken: " << elapsed.count() << " s" << std::endl;
    
    return 0;
}
