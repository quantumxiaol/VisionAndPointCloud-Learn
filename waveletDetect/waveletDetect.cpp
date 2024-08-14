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
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/kdtree.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/octree/octree.h>
#include <pcl/octree/octree_pointcloud_voxelcentroid.h>
#include <pcl/octree/octree_search.h>
#include <vector>
#include <fstream>
#include <Eigen/Dense>
#include <iostream>
#include <chrono>
#include <sstream>
#include <chrono>
#include<thread>
#include <omp.h>
#include <opencv.hpp>
#include<mutex>
typedef pcl::PointXYZ PointT;

void PointLocalNExistNDetection(
    std::string path,
    std::string inputfilename,
    std::string outputfilename,
    float radius,
    float threshold,
    int n
) {
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
    pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>());

    std::stringstream ss;
    ss << path << inputfilename;
    if (pcl::io::loadPCDFile<PointT>(ss.str(), *cloud) == -1)
    {
        PCL_ERROR("Error while reading the point cloud file!\n");
        return;
    }

    std::cout << "Loaded " << cloud->points.size() << " data points from " << inputfilename << std::endl;

    // Create a KDTree for the search method of the extraction.
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
    tree->setInputCloud(cloud);

    for (size_t i = 0; i < cloud->points.size(); ++i)
    {
        std::vector<int> indices;
        std::vector<float> square_dists;
        tree->radiusSearch(i, radius, indices, square_dists);

        int outliers_count = 0;
        for (auto index : indices)
        {
            double diff = std::abs(cloud->points[i].z - cloud->points[index].z);
            if (diff > threshold)
            {
                outliers_count++;
                if (outliers_count >= n)
                    break;
            }
        }

        if (outliers_count >= n)
        {
            cloud_filtered->points.push_back(cloud->points[i]);
        }
    }

    std::cout << "Detected " << cloud_filtered->points.size() << " outlier points." << std::endl;
    cloud_filtered->width = cloud_filtered->points.size();
    cloud_filtered->height = 1;

    std::stringstream output_stream;
    output_stream << path << outputfilename;
    if (pcl::io::savePCDFileASCII(output_stream.str(), *cloud_filtered) == -1)
    {
        PCL_ERROR("Error while saving the point cloud file!\n");
        return;
    }

    std::cout << "Saved " << cloud_filtered->points.size() << " data points to " << outputfilename << std::endl;
}

void PointLocalNExistNDetectionMT(
    std::string path,
    std::string inputfilename,
    std::string outputfilename,
    float radius,
    float threshold,
    int n
) {
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
    pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>());

    std::stringstream ss;
    ss << path << inputfilename;
    if (pcl::io::loadPCDFile<PointT>(ss.str(), *cloud) == -1)
    {
        PCL_ERROR("Error while reading the point cloud file!\n");
        return;
    }

    std::cout << "Loaded " << cloud->points.size() << " data points from " << inputfilename << std::endl;

    // Create a KDTree for the search method of the extraction.
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
    tree->setInputCloud(cloud);

    // Get number of hardware threads
    int threadnum = std::thread::hardware_concurrency();

    // Allocate local point clouds for each thread
    std::vector<std::vector<PointT>> local_clouds(threadnum);

#pragma omp parallel num_threads(threadnum)
    {
        size_t thread_id = omp_get_thread_num();
        size_t num_threads = omp_get_num_threads();

        // Calculate the start and end indices for this thread
        size_t start = (cloud->points.size() / num_threads) * thread_id;
        size_t end = (thread_id != num_threads - 1) ?
            (cloud->points.size() / num_threads) * (thread_id + 1) :
            cloud->points.size();

        for (size_t i = start; i < end; ++i)
        {
            std::vector<int> indices;
            std::vector<float> square_dists;
            tree->radiusSearch(i, radius, indices, square_dists);

            int outliers_count = 0;
            for (auto index : indices)
            {
                double diff = std::abs(cloud->points[i].z - cloud->points[index].z);
                if (diff > threshold)
                {
                    outliers_count++;
                    if (outliers_count >= n)
                        break;
                }
            }

            if (outliers_count >= n)
            {
                local_clouds[thread_id].push_back(cloud->points[i]);
            }
        }
    }

    // Merge local clouds into the main cloud_filtered
    for (const auto& local_cloud : local_clouds)
    {
        cloud_filtered->points.insert(cloud_filtered->points.end(), local_cloud.begin(), local_cloud.end());
    }

    std::cout << "Detected " << cloud_filtered->points.size() << " outlier points." << std::endl;
    cloud_filtered->width = cloud_filtered->points.size();
    cloud_filtered->height = 1;

    std::stringstream output_stream;
    output_stream << path << outputfilename;
    if (pcl::io::savePCDFileASCII(output_stream.str(), *cloud_filtered) == -1)
    {
        PCL_ERROR("Error while saving the point cloud file!\n");
        return;
    }

    std::cout << "Saved " << cloud_filtered->points.size() << " data points to " << outputfilename << std::endl;
}

//经过降采样
void PointLocalNExistNDetectionMTv1(
    std::string path,
    std::string inputfilename,
    std::string outputfilename,
    float radius,
    float threshold,
    int n,
    float voxel_size
) {
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
    pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>());
    pcl::PointCloud<PointT>::Ptr cloud_downsampled(new pcl::PointCloud<PointT>());

    std::stringstream ss;
    ss << path << inputfilename;
    if (pcl::io::loadPCDFile<PointT>(ss.str(), *cloud) == -1)
    {
        PCL_ERROR("Error while reading the point cloud file!\n");
        return;
    }

    std::cout << "Loaded " << cloud->points.size() << " data points from " << inputfilename << std::endl;

    // Create a VoxelGrid filter for down-sampling
    pcl::VoxelGrid<PointT> voxel_filter;
    voxel_filter.setInputCloud(cloud);
    voxel_filter.setLeafSize(voxel_size, voxel_size, voxel_size);
    voxel_filter.filter(*cloud_downsampled);

    std::cout << "Downsampled to " << cloud_downsampled->points.size() << " data points." << std::endl;

    // Create a KDTree for the search method of the extraction.
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
    tree->setInputCloud(cloud_downsampled);

    // Get number of hardware threads
    int threadnum = std::thread::hardware_concurrency();

    // Allocate local point clouds for each thread
    std::vector<std::vector<PointT>> local_clouds(threadnum);

#pragma omp parallel num_threads(threadnum)
    {
        size_t thread_id = omp_get_thread_num();
        size_t num_threads = omp_get_num_threads();

        // Calculate the start and end indices for this thread
        size_t start = (cloud_downsampled->points.size() / num_threads) * thread_id;
        size_t end = (thread_id != num_threads - 1) ?
            (cloud_downsampled->points.size() / num_threads) * (thread_id + 1) :
            cloud_downsampled->points.size();

        for (size_t i = start; i < end; ++i)
        {
            std::vector<int> indices;
            std::vector<float> square_dists;
            tree->radiusSearch(i, radius, indices, square_dists);

            int outliers_count = 0;
            for (auto index : indices)
            {
                double diff = std::abs(cloud_downsampled->points[i].z - cloud_downsampled->points[index].z);
                if (diff > threshold)
                {
                    outliers_count++;
                    if (outliers_count >= n)
                        break;
                }
            }

            if (outliers_count >= n)
            {
                local_clouds[thread_id].push_back(cloud_downsampled->points[i]);
            }
        }
    }

    // Merge local clouds into the main cloud_filtered
    for (const auto& local_cloud : local_clouds)
    {
        cloud_filtered->points.insert(cloud_filtered->points.end(), local_cloud.begin(), local_cloud.end());
    }

    std::cout << "Detected " << cloud_filtered->points.size() << " outlier points." << std::endl;
    cloud_filtered->width = cloud_filtered->points.size();
    cloud_filtered->height = 1;

    std::stringstream output_stream;
    output_stream << path << outputfilename;
    if (pcl::io::savePCDFileASCII(output_stream.str(), *cloud_filtered) == -1)
    {
        PCL_ERROR("Error while saving the point cloud file!\n");
        return;
    }

    std::cout << "Saved " << cloud_filtered->points.size() << " data points to " << outputfilename << std::endl;
}


int main() {
    auto start = std::chrono::high_resolution_clock::now();
    std::string path = "path to the file";
    std::string inputfilename = "Scan.pcd";
    std::string outputfilename = "result.pcd";

    PointLocalNExistNDetection(path,inputfilename, outputfilename, 20.0, 3.0, 5);
    auto endPointLocalMeanDeviationDetection = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = endPointLocalMeanDeviationDetection - start;
    std::cout << "Time taken: " << elapsed.count() << " s" << std::endl;

    PointLocalNExistNDetectionMT(path, inputfilename, outputfilename, 20.0, 4.0, 5);
    auto endPointLocalMeanDeviationDetectionMT = std::chrono::high_resolution_clock::now();
    elapsed = endPointLocalMeanDeviationDetectionMT - endPointLocalMeanDeviationDetection;
    std::cout << "Time taken: " << elapsed.count() << " s" << std::endl;

    PointLocalNExistNDetectionMTv1(path, inputfilename, "result-4.pcd", 20.0, 4.0, 5, 2.0);
    auto endPointLocalMeanDeviationDetectionOT = std::chrono::high_resolution_clock::now();
    elapsed = endPointLocalMeanDeviationDetectionOT - endPointLocalMeanDeviationDetectionMT;
    std::cout << "Time taken: " << elapsed.count() << " s" << std::endl;
    
    return 0;
}
