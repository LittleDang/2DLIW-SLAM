#pragma once
#include "utilies/utilies.h"
#include <Eigen/Dense>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <nav_msgs/Odometry.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
namespace lvio_2d
{
    namespace sensor
    {
        struct imu
        {
            using u_ptr = std::unique_ptr<imu>;
            Eigen::Vector3d acc;
            Eigen::Vector3d gyro;
            double time_stamp;
            imu() : acc(0, 0, 0),
                    gyro(0, 0, 0),
                    time_stamp(TIME_MIN)
            {
            }
            imu(const sensor_msgs::Imu::ConstPtr &msg) : acc(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z),
                                                         gyro(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z),
                                                         time_stamp(msg->header.stamp.toSec())
            {
            }
            ~imu()
            {
                // std::cout << "release imu" << std::endl;
            }
        };

        struct laser
        {
            using u_ptr = std::unique_ptr<laser>;
            double time_stamp;
            std::shared_ptr<std::vector<Eigen::Vector3d>> points_ptr;
            std::shared_ptr<std::vector<double>> times_ptr;

            laser(const sensor_msgs::LaserScan::ConstPtr &msg) : time_stamp(msg->header.stamp.toSec())
            {
                std::tie(points_ptr, times_ptr) = convert::laser_to_point_times(msg);
            }
            ~laser()
            {
                // std::cout << "release laser" << std::endl;
            }
            void correct(const Eigen::Vector3d &linear, const Eigen::Vector3d &angular)
            {
                // cv::Mat board(800, 1600, CV_8UC3);
                // cv::rectangle(board, cv::Point(0, 0), cv::Point(1599, 799), cv::Scalar(0, 0, 0), -1);

                // Eigen::Vector3d origin(400, 400, 0);
                // double scalar = 60;

                // std::cout << "liner:" << linear.transpose() << "angluar:" << angular.transpose() << std::endl;
                for (int i = 0; i < points_ptr->size(); i++)
                {
                    double dt = (*times_ptr)[i] - time_stamp;
                    Eigen::Isometry3d T_i_j = lie::make_tf<double>(dt * linear, dt * angular);
                    // (*points_ptr)[i] = T_i_j.inverse() * ((*points_ptr)[i]);
                    // Eigen::Vector3d tmp = T_i_j.inverse() * ((*points_ptr)[i]);
                    Eigen::Vector3d tmp2 = T_i_j * ((*points_ptr)[i]);

                    //g
                    // {
                    //     Eigen::Vector3d draw_point = ((*points_ptr)[i]) * scalar + origin;
                    //     if (draw_point(0) >= 0 && draw_point(0) < 1600 &&
                    //         draw_point(1) >= 0 && draw_point(1) < 800)
                    //         board.at<cv::Vec3b>(draw_point(1), draw_point(0)) = {0, 255, 0};
                    // }
                    //b
                    // {

                    //     Eigen::Vector3d draw_point = tmp2 * scalar + origin;
                    //     if (draw_point(0) >= 0 && draw_point(0) < 1600 &&
                    //         draw_point(1) >= 0 && draw_point(1) < 800)
                    //         board.at<cv::Vec3b>(draw_point(1), draw_point(0)) = {255, 0, 0};
                    // }
                    //r
                    // {
                    //     Eigen::Vector3d draw_point = tmp * scalar + origin;
                    //     if (draw_point(0) >= 0 && draw_point(0) < 1600 &&
                    //         draw_point(1) >= 0 && draw_point(1) < 800)
                    //         board.at<cv::Vec3b>(draw_point(1), draw_point(0)) = {0, 0, 255};
                    // }
                    (*points_ptr)[i] = tmp2;
                }
                // cv::imshow("scan", board);
                // cv::waitKey(1);
            }
        };

        struct wheel_odom
        {
            using u_ptr = std::unique_ptr<wheel_odom>;

            Eigen::Isometry3d pose;
            double time_stamp;
            wheel_odom() : pose(Eigen::Isometry3d::Identity()) {}
            wheel_odom(const nav_msgs::Odometry::ConstPtr &msg)
            {
                pose = Eigen::Isometry3d::Identity();
                Eigen::Quaterniond q(
                    msg->pose.pose.orientation.w,
                    msg->pose.pose.orientation.x,
                    msg->pose.pose.orientation.y,
                    msg->pose.pose.orientation.z);
                q.normalize();
                Eigen::Vector3d t(
                    msg->pose.pose.position.x,
                    msg->pose.pose.position.y,
                    msg->pose.pose.position.z);
                pose.matrix().template block<3, 3>(0, 0) = q.toRotationMatrix();
                pose.matrix().template block<3, 1>(0, 3) = t;
                time_stamp = msg->header.stamp.toSec();
            }
            ~wheel_odom()
            {
                // std::cout << "release wheel odom" << std::endl;
            }
        };
        struct camera
        {
            using u_ptr = std::unique_ptr<camera>;

            cv::Mat image;
            cv::Mat color_image;
            double time_stamp;
            camera(const sensor_msgs::ImageConstPtr &msg)
            {

                image = cv_bridge::toCvShare(msg, "mono8")->image;
                if (PARAM(enable_camera_vis))
                    color_image = cv_bridge::toCvShare(msg, "bgr8")->image;

                time_stamp = msg->header.stamp.toSec();
            }
            ~camera()
            {
                // std::cout << "release camera" << std::endl;
            }
        };
    } // namespace sensor

} // namespace lvio_2d