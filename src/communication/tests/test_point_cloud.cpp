#include "dds_subscriber.hpp"
#include "timer.hpp"

#include "PointCloudHandler.hpp"
#include "kinodynamics.hpp"

PointCloud2_ lidar_point_cloud;
pcl::PointCloud<pcl::PointXYZ> pcl_cloud;

void get_data_cb(const PointCloud2_& msg) {
    lidar_point_cloud = msg;
    // Get the nearest point to the origin
    convertPointCloud2ToPCL(lidar_point_cloud, pcl_cloud);

    double obstacle_x = 0.0;
    double obstacle_y = 0.0;
    double obstacle_z = 0.0;
    double distance = 0.0;
    std::tie(obstacle_x, obstacle_y, obstacle_z, distance) = getDistanceFromNearestNeighbour(pcl_cloud, 1);

    std::cout << "x, y, z: " << obstacle_x << ", " << obstacle_y << ", " << obstacle_z << "\n";
    std::cout << "Mean distance form NN search: " << distance << "\n";
    // std::cout << "Mean distance from raidus search: " << getAverageDistanceWithinRadius(pcl_cloud, 0.2) << "\n";

}

int main(int argc, char** argv) {
    std::string robot_name = "go2";
    std::string urdf_filepath = "/home/dev/ql/src/robots/" + robot_name + "_description/urdf/" + robot_name + "_description.urdf";
    // pinocchio::Model model;
    // pinocchio::Data data;
    // pinocchio::urdf::buildModel(urdf_filepath, pinocchio::JointModelFreeFlyer(), model);
    // data = pinocchio::Data(model);
    // pinocchio::forwardKinematics(model, data, pinocchio::neutral(model));
    // Quadruped robot(urdf_filepath);

    std::shared_ptr<DDSSubscriber<PointCloud2_>> m_lidar_data_sub_ptr = NULL;
    lidar_point_cloud = PointCloud2_();
    

    m_lidar_data_sub_ptr.reset(new DDSSubscriber<PointCloud2_>(
        "rt/go2/utlidar_pcl",
        std::bind(get_data_cb, std::placeholders::_1),
        0
    ));
    
    {
        Timer timer("Wait time");
        wait_us(500 * 1e6);
    }

    return 0;
}