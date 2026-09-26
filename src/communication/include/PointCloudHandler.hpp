#pragma once

#include "PointCloud2.hpp"

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/kdtree/kdtree_flann.h"

using namespace sensor_msgs::msg::dds_;

void convertPointCloud2ToPCL(const PointCloud2_ &msg, pcl::PointCloud<pcl::PointXYZ> &cloud) {
    // Set up the PointCloud size
    cloud.width = static_cast<uint32_t>(msg.width());
    cloud.height = static_cast<uint32_t>(msg.height());
    cloud.is_dense = static_cast<bool>(msg.is_dense());  // Indicates whether there are any invalid points in the cloud

    // Resize the pcl cloud to match the number of points
    cloud.points.resize(cloud.width * cloud.height);

    // Set the point step (how many bytes per point)
    size_t point_step = static_cast<uint32_t>(msg.point_step());

    // Loop over all the points in the PointCloud2 message
    for (size_t i = 0; i < cloud.height * cloud.width; ++i) {
        // Pointer to the current point in the raw data
        const uint8_t* point_data = &msg.data()[i * point_step];

        // Extract x, y, z values (assuming the PointCloud2 message uses `x`, `y`, `z` fields)
        float x, y, z;
        memcpy(&x, &point_data[0], sizeof(float));  // Read X (float)
        memcpy(&y, &point_data[4], sizeof(float));  // Read Y (float)
        memcpy(&z, &point_data[8], sizeof(float));  // Read Z (float)

        // Store the point into pcl::PointCloud<pcl::PointXYZ>
        cloud.points[i].x = x;
        cloud.points[i].y = y;
        cloud.points[i].z = z;
    }
}

pcl::PointXYZ getAverageDistanceWithinRadius(const pcl::PointCloud<pcl::PointXYZ>& cloud, const double& search_radius) {
    std::shared_ptr<const pcl::PointCloud<pcl::PointXYZ>> cloud_ptr = std::make_shared<const pcl::PointCloud<pcl::PointXYZ>>(cloud);

    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud (cloud_ptr);
    pcl::PointXYZ searchPoint;
    searchPoint.x = -0.1;
    searchPoint.y = 0.0;
    searchPoint.z = 0;

    // Neighbors within radius search

    double mean_radius_search_distance = 0.0;
    int num_points_radius = 0;

    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;

    float radius = search_radius;

    pcl::PointXYZ nearest_point;
    nearest_point.x = 0;
    nearest_point.y = 0;
    nearest_point.z = 0;

    int num_points = 0;

    double min_distance = 1000;

    if ( kdtree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
    {
        for (std::size_t i = 0; i < pointIdxRadiusSearch.size (); ++i) {
            if (pointRadiusSquaredDistance[i] < min_distance) {
                nearest_point.x = (*cloud_ptr)[ pointIdxRadiusSearch[i] ].x;
                nearest_point.y = (*cloud_ptr)[ pointIdxRadiusSearch[i] ].y;
                nearest_point.z = (*cloud_ptr)[ pointIdxRadiusSearch[i] ].z;
            }
            num_points++;
        }
    } else {
        nearest_point.x = 1000;
        nearest_point.y = 1000;
        nearest_point.z = 1000;
    }
    // if (num_points > 0) {
    //     nearest_point.x /= num_points;
    //     nearest_point.y /= num_points;
    //     nearest_point.z /= num_points;
    // }
    return nearest_point;
}

pcl::PointXYZ getNearestNeighbour(const pcl::PointCloud<pcl::PointXYZ>& cloud, const int& num_neighbours) {
    std::shared_ptr<const pcl::PointCloud<pcl::PointXYZ>> cloud_ptr = std::make_shared<const pcl::PointCloud<pcl::PointXYZ>>(cloud);

    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud (cloud_ptr);
    pcl::PointXYZ searchPoint;
    searchPoint.x = 0;
    searchPoint.y = 0;
    searchPoint.z = 0;

    // K nearest neighbor search

    int K = num_neighbours;

    std::vector<int> pointIdxKNNSearch(K);
    std::vector<float> pointKNNSquaredDistance(K);

    double neighbour_x = 0;
    double neighbour_y = 0;
    double neighbour_z = 0;
    double mean_distance = 0.0;
    if ( kdtree.nearestKSearch (searchPoint, K, pointIdxKNNSearch, pointKNNSquaredDistance) > 0 )
    {
        for (std::size_t i = 0; i < pointIdxKNNSearch.size (); ++i) {
            neighbour_x = (*cloud_ptr)[ pointIdxKNNSearch[i] ].x;
            neighbour_y = (*cloud_ptr)[ pointIdxKNNSearch[i] ].y;
            neighbour_z = (*cloud_ptr)[ pointIdxKNNSearch[i] ].z;
            mean_distance += std::sqrt(pointKNNSquaredDistance[i]);
        }
    }
    pcl::PointXYZ neighbour_point;

    neighbour_point.x = neighbour_x;
    neighbour_point.y = neighbour_y;
    neighbour_point.z = neighbour_z;

    // return std::tuple<double, double, double, double>(neighbour_x, neighbour_y, neighbour_z, mean_distance / K);
    return neighbour_point;
}