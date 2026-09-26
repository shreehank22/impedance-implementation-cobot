#include "dds_subscriber.hpp"
#include "timer.hpp"

#include "Point.hpp"
#include "JoyData.hpp"
#include "JointData.hpp"

using namespace xterra::msg::dds_;

void get_joy_data_cb(const JoyData_& msg) {
	std::cout << "Joy subscriber triggered...\n";
}
void get_point_data_cb(const Point_& msg) {
	std::cout << "Point subscriber triggered...\n";
}
void get_joint_data_cb(const JointData_& msg) {
	std::cout << "Joint subscriber triggered...\n";
}

int main(int argc, char** argv) {
	std::shared_ptr<DDSSubscriber<Point_>> m_point_sub_ptr = NULL;
	std::shared_ptr<DDSSubscriber<JoyData_>> m_joy_sub_ptr = NULL;
	std::shared_ptr<DDSSubscriber<JointData_>> m_joint_sub_ptr = NULL;

	m_point_sub_ptr.reset(new DDSSubscriber<Point_>("test/point", std::bind(get_point_data_cb, std::placeholders::_1), 0));
	m_point_sub_ptr->setBestEffortQoS();
	m_joy_sub_ptr.reset(new DDSSubscriber<JoyData_>("test/joy_data", std::bind(get_joy_data_cb, std::placeholders::_1), 0));
	m_joy_sub_ptr->setBestEffortQoS();
	m_joint_sub_ptr.reset(new DDSSubscriber<JointData_>("test/joint_data", std::bind(get_joint_data_cb, std::placeholders::_1), 0));
	m_joint_sub_ptr->setBestEffortQoS();

	wait_us(1e8);

	return 0;
}
