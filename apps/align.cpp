#include <iostream>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <ros/ros.h>

#include <pclomp/ndt_omp.h>

// align point clouds and measure processing time
pcl::PointCloud<pcl::PointXYZ>::Ptr
align(pcl::Registration<pcl::PointXYZ, pcl::PointXYZ>::Ptr registration,
      const pcl::PointCloud<pcl::PointXYZ>::Ptr& target_cloud,
      const pcl::PointCloud<pcl::PointXYZ>::Ptr& source_cloud) { //TF: target ---> souce
    registration->setInputTarget(target_cloud);
    registration->setInputSource(source_cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr aligned(new pcl::PointCloud<pcl::PointXYZ>());

    auto t1 = ros::WallTime::now();
    registration->align(*aligned);
    auto t2 = ros::WallTime::now();
    std::cout << "single : " << (t2 - t1).toSec() * 1000 << "[msec]" << std::endl;
    std::cout << "fitness: " << registration->getFitnessScore() << std::endl
              << std::endl;

    return aligned;
}

int main(int argc, char** argv) {
    if (argc != 3) {
        std::cout << "usage: align target.pcd source.pcd" << std::endl;
        return 0;
    }

    std::string target_pcd = argv[1];
    std::string source_pcd = argv[2];

    pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>());

    if (pcl::io::loadPCDFile(target_pcd, *target_cloud)) {
        std::cerr << "failed to load " << target_pcd << std::endl;
        return 0;
    }
    if (pcl::io::loadPCDFile(source_pcd, *source_cloud)) {
        std::cerr << "failed to load " << source_pcd << std::endl;
        return 0;
    }

    // downsampling
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZ>());

    pcl::VoxelGrid<pcl::PointXYZ> voxelgrid;
    voxelgrid.setLeafSize(0.1f, 0.1f, 0.1f);

    voxelgrid.setInputCloud(target_cloud);
    voxelgrid.filter(*downsampled);
    *target_cloud = *downsampled;

    voxelgrid.setInputCloud(source_cloud);
    voxelgrid.filter(*downsampled);
    source_cloud = downsampled;

    ros::Time::init();

    // benchmark
    std::cout << "--- pcl::NDT ---" << std::endl;
    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>::Ptr ndt(
        new pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>());
    ndt->setResolution(1.0);
    ndt->setTransformationEpsilon(0.01);
    ndt->setStepSize(0.1);
    ndt->setMaximumIterations(35);
    pcl::PointCloud<pcl::PointXYZ>::Ptr aligned = align(ndt, target_cloud, source_cloud);

    std::vector<int> num_threads = {1, omp_get_max_threads()};
    std::vector<std::pair<std::string, pclomp::NeighborSearchMethod>> search_methods = {
        {"KDTREE", pclomp::KDTREE},
        {"DIRECT7", pclomp::DIRECT7},
        {"DIRECT1", pclomp::DIRECT1}};

    pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>::Ptr ndt_omp(
        new pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>());

    ndt_omp->setResolution(1.0);
    ndt_omp->setStepSize(0.1);

    for (int n : num_threads) {
        for (const auto& search_method : search_methods) {
            std::cout << "--- pclomp::NDT (" << search_method.first << ", " << n << " threads) ---"
                      << std::endl;
            ndt_omp->setNumThreads(n);
            ndt_omp->setNeighborhoodSearchMethod(search_method.second);
            aligned = align(ndt_omp, target_cloud, source_cloud);
        }
    }

    // visulization
    pcl::visualization::PCLVisualizer vis("vis");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_handler(target_cloud, 255.0, 0.0, 0.0); // r,g,b  红色
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_handler(source_cloud, 0.0, 255.0, 0.0); //绿色
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> aligned_handler(aligned, 0.0, 0.0, 255.0);//蓝色
    vis.addPointCloud(target_cloud, target_handler, "target");
    vis.addPointCloud(source_cloud, source_handler, "source");
    vis.addPointCloud(aligned, aligned_handler, "aligned");
    vis.spin();

    return 0;
}
