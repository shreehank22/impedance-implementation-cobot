#include "dds_publisher.hpp"
#include "timer.hpp"
#include "Point.hpp"
#include "JoyData.hpp"
#include "JointData.hpp"

using namespace xterra::msg::dds_;

int main(int argc, char** argv) {
	std::shared_ptr<DDSPublisher<Point_>> m_point_pub_ptr = NULL;
	std::shared_ptr<DDSPublisher<JoyData_>> m_joy_pub_ptr = NULL;
	std::shared_ptr<DDSPublisher<JointData_>> m_joint_pub_ptr = NULL;

	Point_ point_data;
	point_data.x() = 1.0;
	point_data.y() = 2.0;
	point_data.z() = 3.0;

	JoyData_ joy_data;
	joy_data.axes()[0] = 0;

	JointData_ joint_data;

	m_point_pub_ptr.reset(new DDSPublisher<Point_>("test/point"));
	m_joy_pub_ptr.reset(new DDSPublisher<JoyData_>("test/joy_data"));
	m_joint_pub_ptr.reset(new DDSPublisher<JointData_>("test/joint_data"));

	while (true) {
		m_point_pub_ptr->publish(point_data);
		m_joy_pub_ptr->publish(joy_data);
		m_joint_pub_ptr->publish(joint_data);
		wait_us(1e6 * 1);
	}

	return 0;
}
