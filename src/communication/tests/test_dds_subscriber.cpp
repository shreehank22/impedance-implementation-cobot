#include "dds_subscriber.hpp"
#include "timer.hpp"

#include "JointData.hpp"
#include "SensorData.hpp"
#include "QuadLog.hpp"
#include "JoyData.hpp"
#include "SolverStats.hpp"
#include "PointCloud2.hpp"

using namespace xterra::msg::dds_;
using namespace sensor_msgs::msg::dds_;

double t_curr = 0;
double t_last = 0;

void get_data_cb(const PointCloud2_& msg) {
    std::cout << "Receiving point cloud...\n";
    // lidar_point_cloud = msg;
    std::cout << "Point cloud received...\n";
    // Get the nearest point to the origin
}

void get_sensor_data_cb(const SensorData_& msg) {
	static std::chrono::time_point<std::chrono::high_resolution_clock> startTimePoint = std::chrono::high_resolution_clock::now();
	t_last = t_curr;
	t_curr = get_wall_time_seconds(startTimePoint);

    std::cout << "Sensor data rate :" << 1 / (t_curr - t_last) << "\n";
}

int main(int argc, char** argv) {
    std::shared_ptr<DDSSubscriber<PointCloud2_>> m_lidar_data_sub_ptr = NULL;
    PointCloud2_ lidar_point_cloud;

    // m_lidar_data_sub_ptr.reset(new DDSSubscriber<PointCloud2_>(
    //     "rt/go2/utlidar_pcl",
    //     std::bind(get_data_cb, std::placeholders::_1),
    //     0
    // ));
    // lidar_point_cloud = PointCloud2_();

    std::shared_ptr<DDSSubscriber<SensorData_>> m_sensor_data_sub_ptr = NULL;
    SensorData_ sensor_data;

    m_sensor_data_sub_ptr.reset(new DDSSubscriber<SensorData_>(
        "rt/go2/sensor_data",
        std::bind(get_sensor_data_cb, std::placeholders::_1),
        0
    ));
    sensor_data = SensorData_();
    
    {
        Timer timer("Wait time");
        wait_us(50 * 1e6);
    }

    return 0;
}
