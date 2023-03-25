#include "ros/ros.h"
#include "trajectory/trajectory.h"
#include "utilies/params.h"
#include "trajectory/dispatch.h"
class Node
{
public:
    Node() : sensor_queues(&tra)
    {
        laser_sub = nh.subscribe<sensor_msgs::LaserScan>(PARAM(laser_topic), 1000,
                                                         std::bind(&Node::laser_call_back, this, std::placeholders::_1));
        imu_sub = nh.subscribe<sensor_msgs::Imu>(PARAM(imu_topic), 1000,
                                                 std::bind(&Node::imu_call_back, this, std::placeholders::_1));
        wheel_odom_sub = nh.subscribe<nav_msgs::Odometry>(PARAM(wheel_odom_topic), 1000,
                                                          std::bind(&Node::wheel_odom_call_back, this, std::placeholders::_1));
        image_transport::ImageTransport it(nh);

        camera_sub = it.subscribe(PARAM(camera_topic), 1,
                                  std::bind(&Node::image_callback, this, std::placeholders::_1));
    }

private:
    ros::NodeHandle nh;
    lvio_2d::trajectory tra;
    ros::Subscriber laser_sub;
    ros::Subscriber imu_sub;
    ros::Subscriber wheel_odom_sub;
    image_transport::Subscriber camera_sub;

    lvio_2d::dispatch_queue sensor_queues;
    void laser_call_back(const sensor_msgs::LaserScan::ConstPtr &msg)
    {
        if (!PARAM(enable_laser))
            return;
        sensor_queues.add_laser_msg(msg);
    }
    void imu_call_back(const sensor_msgs::Imu::ConstPtr &msg)
    {
        sensor_queues.add_imu_msg(msg);
    }
    void wheel_odom_call_back(const nav_msgs::Odometry::ConstPtr &msg)
    {
        sensor_queues.add_wheel_odom_msg(msg);
    }
    void image_callback(const sensor_msgs::ImageConstPtr &msg)
    {
        if (!PARAM(enable_camera))
            return;
        sensor_queues.add_camera_msg(msg);
    }
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "lvio_2d");
    Node n;
    ros::spin();
    return 0;
}