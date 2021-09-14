#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/Twist.h>
#include <thread>
#include <boost/filesystem.hpp>

#include "rang.hpp"

using namespace std::chrono;

namespace
{
  geometry_msgs::Twist twist(const double linear_x, const double angular_z)
  {
    geometry_msgs::Twist twist;
    
    twist.linear.x = linear_x;
    twist.angular.z = angular_z;
    return twist;
  }

  ros::Publisher cmd_vel_pub;

  template<typename T, typename P>
  void twist_for(const double linear_x, const double angular_z, const duration<T, P> &duration)
  {
    const auto start = system_clock::now();
    
    ros::Rate rate(5.0);
    while (ros::ok() && (system_clock::now() - start) < duration)
    {
      cmd_vel_pub.publish(twist(linear_x, angular_z));
      ros::spinOnce();
      rate.sleep();
    }

    cmd_vel_pub.publish(twist(0.0, 0.0));
    ros::spinOnce();
  }
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "calibrate_node");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  using namespace std;
  using namespace rang;

  string cmd_vel_topic;
  pnh.param<string>("topic_cmd_vel", cmd_vel_topic, "/jetbot/base_controller/cmd_vel");

  cmd_vel_pub = nh.advertise<geometry_msgs::Twist>(cmd_vel_topic, 1);

  cout
    << style::bold
    << "Waiting for "
    << fg::cyan
    << cmd_vel_topic
    << fg::reset
    << " to be subscribed to..."
    << endl;
  while (cmd_vel_pub.getNumSubscribers() == 0)
  {
    this_thread::sleep_for(milliseconds(100));
  }

  cout << endl;
  cout
    << style::bold
    << "Welcome to the "
    << fg::green
    << "jetbot_controller Calibration Tool"
    << fg::reset
    << "! Press any key to move to the next step."
    << style::reset
    << endl
    << endl;

  cout
    << style::bold << fg::yellow
    << "Step 1: "
    << fg::reset << style::reset
    << "Place the jetbot on the floor. It should have ~2 meters (6 feet) of unobstructed space around it.";
  cout.flush();
  std::cin.get();

  cout
    << style::bold
    << fg::yellow
    << "Step 2: "
    << fg::reset
    << style::reset
    << "Mark the current position of the robot's left and right wheel with tape.";
  cout.flush();
  std::cin.get();

  cout
    << style::bold
    << fg::yellow
    << "Step 3: "
    << fg::reset
    << style::reset
    << "The robot will now move forward.";
  cout.flush();
  std::cin.get();
  twist_for(0.5, 0.0, seconds(2));

  cout
    << style::bold
    << fg::yellow
    << "Step 4: "
    << fg::reset
    << style::reset
    << "Measure the distance each wheel has traveled in centimeters. Mark the new locations with tape.";
  cout.flush();
  std::cin.get();

  cout
    << fg::blue
    << "  Left wheel travel distance (cm): "
    << fg::reset;
  double left_wheel_travel_distance_05 = 0.0;
  std::cin >> left_wheel_travel_distance_05;
  std::cin.get();

  cout
    << fg::blue
    << "  Right wheel travel distance (cm): "
    << fg::reset;
  double right_wheel_travel_distance_05 = 0.0;
  std::cin >> right_wheel_travel_distance_05;
  std::cin.get();

    // Convert to meters/second
  left_wheel_travel_distance_05 *= 2.0;
  right_wheel_travel_distance_05 *= 2.0;

  cout
    << style::bold
    << fg::yellow
    << "Step 5: "
    << fg::reset
    << style::reset
    << "The robot will now move backward.";
  cout.flush();
  std::cin.get();

  twist_for(-1.0, 0.0, seconds(1));

  cout
    << style::bold
    << fg::yellow
    << "Step 6: "
    << fg::reset
    << style::reset
    << "Measure the distance each wheel has traveled in centimeters.";
  cout.flush();
  std::cin.get();

  cout
    << fg::blue
    << "  Left wheel travel distance (cm): "
    << fg::reset;
  double left_wheel_travel_distance_10 = 0.0;
  std::cin >> left_wheel_travel_distance_10;
  std::cin.get();

  cout
    << fg::blue
    << "  Right wheel travel distance (cm): "
    << fg::reset;
  double right_wheel_travel_distance_10 = 0.0;
  std::cin >> right_wheel_travel_distance_10;
  std::cin.get();

  const auto avg_left_distance = (left_wheel_travel_distance_05 + right_wheel_travel_distance_10) / 2.0;
  const auto avg_right_distance = (right_wheel_travel_distance_05 + right_wheel_travel_distance_10) / 2.0;
  const boost::filesystem::path package_path = ros::package::getPath("jetbot_controller");
  const auto calibration_yaml_path = package_path / "config" / "calibration.yaml";

  std::ofstream o(calibration_yaml_path.string());
  if (!o)
  {
    std::cerr << fg::red << "Failed to open " << calibration_yaml_path << " for writing" << fg::reset << std::endl;
    return EXIT_FAILURE;
  }

  o << "meters_per_second_10: " << (avg_left_distance + avg_right_distance) / 2.0 / 100.0 << endl;
  o.close();

  std::cout << fg::green << "Done!" << fg::reset << std::endl;

  return EXIT_SUCCESS;
}
