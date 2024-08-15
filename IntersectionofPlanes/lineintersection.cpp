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
���ȴӸ�����ƽ�淽�̣�����������ʾ������ȡ��������normal1��normal2��
Ȼ�󣬼����������������Ĳ�˵õ����ߵķ�������direction��
������������ķ���Ϊ�㣬����ζ������ƽ��ƽ�л��غϣ����û��Ψһ�Ľ��ߣ���������false��
������������һ��3x3�������а���������������ƽ���ڽ��ߵķ���������
���㳣������constants�����а�����ƽ�淽���еĳ�����Ը��ű�ʾ����һ����Ԫ�أ���Ӧ�ڽ��߷�����������
ʹ��QR�ֽ������󷽳�Ax = b������A�ǰ����������ͷ��������ľ���b�ǳ����������⽫����λ�ڽ����ϵĵ�point��
��������ĵ㸳ֵ��linePoint���������������淶����ֵ��lineDirection��
*/
bool computePlaneIntersection(
    const Eigen::Vector4f& plane1, //ƽ�淽��1
    const Eigen::Vector4f& plane2, //ƽ�淽��2
    Eigen::Vector3f& linePoint, //�����ϵ�һ����
    Eigen::Vector3f& lineDirection//���ߵķ�������
) {
    Eigen::Vector3f normal1(plane1[0], plane1[1], plane1[2]);
    Eigen::Vector3f normal2(plane2[0], plane2[1], plane2[2]);
    // direction ����������ƽ�淨�������Ĳ��������ƽ���ཻʱ���ߵķ���
    Eigen::Vector3f direction = normal1.cross(normal2);
    // ���� direction �ķ����������Ȼ�ģ�����������Ϊ�㣬����ƽ��Ҫô��ȫƽ�У�û�н��ߣ���
    // Ҫôʵ�����غ�Ϊͬһƽ�棨�����������ߣ����޷�����һ��Ψһ�Ľ��߷��򣩣������һ��������Ϣ������ false��
    if (direction.norm() == 0) {
        std::cerr << "Planes are parallel or coincident, no unique intersection line." << std::endl;
        return false;
    }
    /*
    Eigen::Matrix3f matrix;
    matrix << normal1, normal2, direction;
    Eigen::Vector3f constants(-plane1[3], -plane2[3], 0);
    // �����Է����� Ax = b������ A ���� matrix ��ʾ�ľ���b ���� constants ��ʾ��������
    // ʹ�� Eigen ���е� colPivHouseholderQr() ���������� A �� QR �ֽ⣬��������Է����顣
    // ����洢�� point �����У�point ���Ƿ�����Ľ⡣
    Eigen::Vector3f point = matrix.colPivHouseholderQr().solve(constants);
    */

    //����Ѱ�ҽ�����һ���㣬
    //ֱ���ϵĵ�ͬʱ����a1*x + b1*y + c1*z + d1 = 0 �� a2*x + b2*y + c2*z + d2 = 0
    //�����ٶ� x = 0 һ��Ϊ��0 , y0 , z0��
    //y0 , z0ͨ���ⷽ����b1*y + c1*z + d1 = 0 �� b2*y + c2*z + d2 = 0 ���
    //��ʵ�� z = -(b2*d1-b1*d2)/(b2*c1-b1*c2)
    //y = (d1*(c1*b2-b1*c2))/(b1*(b2c1-b1c2))


    // �ҵ�������Ϊ0��������
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

    // ������ѡ���������Է�����
    Eigen::Matrix2f subMatrix;
    Eigen::Vector2f subConstants;

    switch (axisToZero) {
    case 0: // ��� x=0
        subMatrix << normal1[1], normal1[2],
            normal2[1], normal2[2];
        subConstants << -plane1[3], -plane2[3];
        break;
    case 1: // ��� y=0
        subMatrix << normal1[0], normal1[2],
            normal2[0], normal2[2];
        subConstants << -plane1[3], -plane2[3];
        break;
    case 2: // ��� z=0
        subMatrix << normal1[0], normal1[1],
            normal2[0], normal2[1];
        subConstants << -plane1[3], -plane2[3];
        break;
    }

    // ������Է�����
    Eigen::Vector2f solution;
    // QR �ֽ⽫���� �ֽ�Ϊһ�����������һ�������Ǿ���ĳ˻�
    // ִ�д�������Ԫ��column pivoting����Householder QR�ֽ⡣����Ԫ��ָ�ڷֽ�����У�������п��ܻᱻ����������������ֵ�ȶ���
    //Eigen::Vector2f solution = subMatrix.colPivHouseholderQr().solve(subConstants);

    // LU�ֽ⽫���� �ֽ�Ϊһ�������Ǿ����һ�������Ǿ���ĳ˻�
    Eigen::FullPivLU<Eigen::Matrix2f> luDecomposition(subMatrix);

    // �������Ƿ���棨���ֽ��Ƿ�ɹ���
    if (luDecomposition.isInvertible()) {
        // ʹ�� LU �ֽ�������Է�����
        solution = luDecomposition.solve(subConstants);
        // solution ���ڰ����˽ⷽ����Ľ�
    }
    else {
        std::cerr << "The matrix is singular and cannot be inverted." << std::endl;
    }


    // ���������ϵ�һ��
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
* �������һ����
���ȣ�����һ��SACSegmentation����seg������ģ������ΪSACMODEL_PLANE����������ΪSAC_RANSAC������������Ϊ50000��������ֵΪ12��
Ȼ�󣬴���һ��ָ��������Ƶ�ָ��remainingCloud��һ��ָ������е��ڵ��ָ��inliers��һ��ָ��ģ��ϵ����ָ��coefficients��
������������ÿ��ƽ�棬����SACSegmentation::segment()��������ȡ���ڵ��ģ��ϵ����
����ڵ������С��100��˵���޷����Ƹ������ݼ���ƽ��ģ�ͣ��������ء�
��ģ��ϵ���洢��planeCoefficients�У�������һ��ExtractIndices����extract�������������ΪremainingCloud���ڵ�Ϊinliers��
����extract.filter()���������ڵ��remainingCloud����ȡ������
���planeCoefficients�Ĵ�СС��2��˵���ڵ������Ҳ�����������ƽ�棬�������ء�
����computePlaneIntersection()��������������ƽ��Ľ��ߣ�������洢��linePoint��lineDirection�С�
*/

//���ؽ����ڵ����ϵĶ˵�
void IntersectionPoint(
    Eigen::Vector3f& linePoint,//�����ϵ�һ����
    Eigen::Vector3f& lineDirection,//���ߵķ�������
    Eigen::Vector3f endPointMin,//���ߵ����
    Eigen::Vector3f endPointMax,//���ߵ��յ�
    float RadiusSearch = 2.0,//�뾶�˲��İ뾶
    int MinNeighborsInRadius = 8,//�뾶�˲����ھ���
    std::string outputDir = "C:\\work\\test1\\"//���·��
) {
    //�򿪵ڶ��ĵ��ƣ���Ӧб����
    pcl::PointCloud<PointT>::Ptr cloud2(new pcl::PointCloud<PointT>());
    std::stringstream ss;
    ss << outputDir << "output_ascii_" << std::setw(4) << std::setfill('0') << 1 << ".pcd";
    //pcl::io::loadPCDFile<pcl::PointXYZI>(ss, *cloud2);
    if (pcl::io::loadPCDFile<PointT>(ss.str(), *cloud2) == -1)
    {
        PCL_ERROR("Error while reading the point cloud file!\n");

    }
    //�������еĵ�Ͷ�䵽ֱ����

    //�뾶�˲�����ȥ��Ⱥ�㣬���ܻᵼ�½��߳��ȼ��٣���RadiusSearch�����
    pcl::RadiusOutlierRemoval<PointT> outrem;
    outrem.setInputCloud(cloud2);
    outrem.setRadiusSearch(RadiusSearch);
    outrem.setMinNeighborsInRadius(MinNeighborsInRadius);
    outrem.setKeepOrganized(true);

    pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
    outrem.filter(*cloud_filtered);
    cloud2.swap(cloud_filtered);

    // ͶӰ���Ƶ�������
    std::vector<Eigen::Vector3f> projectedPoints;
    for (const auto& point : cloud2->points)
    {
        Eigen::Vector3f pointVec(point.x, point.y, point.z);
        // ͶӰ�㵽ֱ��
        Eigen::Vector3f projectedPoint = linePoint + ((pointVec - linePoint).dot(lineDirection)) * lineDirection;
        projectedPoints.push_back(projectedPoint);
    }

    // ����ͶӰ��ļ�ֵ
    float minProj = std::numeric_limits<float>::max();
    float maxProj = std::numeric_limits<float>::lowest();
    for (const auto& projPoint : projectedPoints)
    {
        float projLength = (projPoint - linePoint).dot(lineDirection);
        minProj = std::min(minProj, projLength);
        maxProj = std::max(maxProj, projLength);
    }

    // ȷ���߶ζ˵�
    endPointMin = linePoint + minProj * lineDirection;
    endPointMax = linePoint + maxProj * lineDirection;

    std::cout << endPointMin << std::endl << endPointMax << std::endl << std::endl;

    //��������txt
    // д���ı��ļ�
    std::stringstream si;
    std::string filePath = "result.txt"; // ����ļ�·��
    si << outputDir << "output_ascii_" << filePath;

    // ʹ�� ofstream ������д���ļ�
    std::ofstream outputFile(si.str());
    if (outputFile.is_open()) {
        // д������
        outputFile << endPointMin(0) << " " << endPointMin(1) << " " << endPointMin(2) << std::endl;
        outputFile << endPointMax(0) << " " << endPointMax(1) << " " << endPointMax(2) << std::endl;
        outputFile.close();
    }
    else {
        std::cerr << "Unable to open file: " << filePath << std::endl;
    }
    
}

//����ƽ�潻��
void PlanesIntersection(
    const pcl::PointCloud<PointT>::Ptr& inputCloud,//���������
    std::vector<Eigen::Vector4f>& planeCoefficients,//ƽ�淽��ϵ��ax+by+cz+d=0
    Eigen::Vector3f& linePoint,//�����ϵ�һ����
    Eigen::Vector3f& lineDirection,//���ߵķ�������
    Eigen::Vector3f endPointMin,//���ߵ����
    Eigen::Vector3f endPointMax,//���ߵ��յ�
    int numPlanes = 2,//Ҫ��ֵ�ƽ����������ƽ����н���
    float distanceThreshold = 12.0f,//������ֵ
    float RadiusSearch=2.0,//�뾶�˲��İ뾶
    int MinNeighborsInRadius=8,//�뾶�˲����ھ���
    int maxIterations = 50000,//����ƽ������������
    std::string outputDir //���·��

)
{
    pcl::SACSegmentation<PointT> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    //���������һ���ֵ㡣
    //ʹ����Щ��������һ��ģ�ͣ����Ƶķ���һ��Ϊ��С���˷�
    //�����е�������ģ�ͣ�����õ㵽��ǰģ�͵����С����ֵ���򽫸õ��Ϊ�ڵ㡣
    //�������ڵ����¹���һ��ģ��
    //�Ƚϵ�ǰģ�ͺ�֮ǰ�Ƴ�����õ�ģ�͵��ڵ����������¼����ڵ�����ģ�Ͳ������ڵ�����
    //�ظ�1 - ������ֱ�������������ߵ�ǰģ�͵��ڵ�������һ������ֵ��
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

        //�������
        std::stringstream ss;
        ss << outputDir << "output_ascii_" << std::setw(4) << std::setfill('0') << i << ".pcd";
        pcl::io::savePCDFileASCII(ss.str(), *cloud_plane);

        //��ȡ����ָ������֮��ĵ������ݡ�Ȼ��extract.filter(*cloud)��Ӧ�ù�������������洢��cloud�����У������˵�������ƽ��֮��ĵ������ݡ�
        extract.setNegative(true);
        extract.filter(*cloud_filter);

        remainingCloud.swap(cloud_filter);
        //�ͷ�cloud_filter
        cloud_filter.reset(new pcl::PointCloud<PointT>());


    }
    //�ͷ��ڴ�
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
    * ������ reset() �����������Զ����� pcl::PointCloud::Ptr ʱ�������ڴ��쳣
    * ��Eigen Memory.h�е�handmade_aligned_free
    * 0x00007FFB52E55BB6 (ntdll.dll)��(λ�� PCL_test.exe ��)�������쳣: 0xC0000005: ��ȡλ�� 0xFFFFFFFFFFFFFFFF ʱ�������ʳ�ͻ
    * ������ĳ��ָ��û�б���ȷ��ʼ��������ָ����һ���Ѿ����ͷŵ��ڴ����򣬻���Խ���������ʱ
    �����ڴ������⣬��һЩ���������
        ��1��VS2019����ͨ����Ŀ����->C/C++->��������->������ǿָ�->ѡ��AVX�������
        ��2��qt ����pro�ļ���� QMAKE_CXXFLAGS += /arch:AVX��
        ��3����������CMake�����ģ�ԭ������ǿ�����-march=native��CPUָ��Ż�����ɸñ������Զ��ͷţ�
        ��share_ptrȥѰ�Ҳ�����������ʱ�򱨴�����������رո��Ż����ɣ�
        ��4������VS2022�У�Ҫ�������ڴ��ͷ����⣬����Ҫ�ĳ� VS��ͨ����Ŀ����->C/C++->��������->������ǿָ�->δ����
    */

    IntersectionPoint(linePoint, lineDirection, endPointMin, endPointMax, RadiusSearch, MinNeighborsInRadius, outputDir);
    /*
    //�򿪵ڶ��ĵ��ƣ���Ӧб����
    pcl::PointCloud<PointT>::Ptr cloud2(new pcl::PointCloud<PointT>());
    std::stringstream ss;
    ss << outputDir << "output_ascii_" << std::setw(4) << std::setfill('0') << 1 << ".pcd";
    //pcl::io::loadPCDFile<pcl::PointXYZI>(ss, *cloud2);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(ss.str(), *cloud2) == -1)
    {
        PCL_ERROR("Error while reading the point cloud file!\n");

    }
    //�������еĵ�Ͷ�䵽ֱ����

    //�뾶�˲�����ȥ��Ⱥ�㣬���ܻᵼ�½��߳��ȼ��٣���RadiusSearch�����
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
    outrem.setInputCloud(cloud2);
    outrem.setRadiusSearch(RadiusSearch);
    outrem.setMinNeighborsInRadius(MinNeighborsInRadius);
    outrem.setKeepOrganized(true);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    outrem.filter(*cloud_filtered);
    cloud2.swap(cloud_filtered);

    // ͶӰ���Ƶ�������
    std::vector<Eigen::Vector3f> projectedPoints;
    for (const auto& point : cloud2->points)
    {
        Eigen::Vector3f pointVec(point.x, point.y, point.z);
        // ͶӰ�㵽ֱ��
        Eigen::Vector3f projectedPoint = linePoint + ((pointVec - linePoint).dot(lineDirection)) * lineDirection;
        projectedPoints.push_back(projectedPoint);
    }

    // ����ͶӰ��ļ�ֵ
    float minProj = std::numeric_limits<float>::max();
    float maxProj = std::numeric_limits<float>::lowest();
    for (const auto& projPoint : projectedPoints)
    {
        float projLength = (projPoint - linePoint).dot(lineDirection);
        minProj = std::min(minProj, projLength);
        maxProj = std::max(maxProj, projLength);
    }

    // ȷ���߶ζ˵�
    endPointMin = linePoint + minProj * lineDirection;
    endPointMax = linePoint + maxProj * lineDirection;

    std::cout << endPointMin << std::endl << endPointMax << std::endl << std::endl;
    
    //��������txt
    // д���ı��ļ�
    std::stringstream si;
    std::string filePath = "result.txt"; // ����ļ�·��
    si << outputDir << "output_ascii_" << filePath;

    // ʹ�� ofstream ������д���ļ�
    std::ofstream outputFile(si.str());
    if (outputFile.is_open()) {
        // д������
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
    // ÿ������3��floatֵ��ʾ����
    const size_t pointSize = sizeof(CloudData);
    std::vector<char> buffer(pointSize * 100000); // ����������������1000����

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
    //cloud->reserve(100000); // Ԥ���㹻�Ŀռ��Ա��������·����ڴ�
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