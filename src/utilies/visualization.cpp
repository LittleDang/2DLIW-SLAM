#include "utilies/visualization.h"
namespace lvio_2d
{
    geometry_msgs::Point point_eigen_to_ros(const Eigen::Vector3d &p)
    {
        geometry_msgs::Point rp;
        rp.x = p(0);
        rp.y = p(1);
        rp.z = p(2);
        return rp;
    }
    geometry_msgs::Transform eigen_tf_to_ros_geometry(const Eigen::Isometry3d &tf)
    {
        geometry_msgs::Transform ros_geometry;
        Eigen::Quaterniond q(tf.matrix().block<3, 3>(0, 0));
        ros_geometry.rotation.w = q.w();
        ros_geometry.rotation.x = q.x();
        ros_geometry.rotation.y = q.y();
        ros_geometry.rotation.z = q.z();

        ros_geometry.translation.x = tf.matrix()(0, 3);
        ros_geometry.translation.y = tf.matrix()(1, 3);
        ros_geometry.translation.z = tf.matrix()(2, 3);
        return ros_geometry;
    }
    visualization::ptr visualization::get_ptr()
    {
        static ptr ret = nullptr;
        if (!ret)
            ret = std::make_shared<visualization>();
        return ret;
    }
    int check_map_and_calc_index(const nav_msgs::OccupancyGrid &map, const Eigen::Vector3d &target)
    {
        Eigen::Vector3d origin(0, 0, 0);
        origin(0) = map.info.origin.position.x;
        origin(1) = map.info.origin.position.y;

        double res = map.info.resolution;
        Eigen::Vector3d tmp = target - origin;
        tmp /= res;
        int x = tmp(0);
        int y = tmp(1);
        int w = map.info.width;
        int h = map.info.height;
        if (x < 0 || x >= w || y < 0 || y >= h)
            return -1;
        return y * w + x;
    }
    void update_occupancy_grid(nav_msgs::OccupancyGrid &map, const Eigen::Vector3d &emit_origin,
                               const Eigen::Vector3d &target)
    {
        double len = (target - emit_origin).norm();
        Eigen::Vector3d unit = (target - emit_origin) / len;
        double step = map.info.resolution / 2;

        for (double tr = 0; tr <= len; tr += step)
        {
            Eigen::Vector3d cp = emit_origin + unit * tr;
            int index = check_map_and_calc_index(map, cp);
            if (index > -1 && map.data[index] == -1)
                map.data[index] = 0;
        }

        {
            int index = check_map_and_calc_index(map, target);
            if (index > -1)
            {
                if (map.data[index] == -1 || map.data[index] == 0)
                    map.data[index] = 50;
                else
                    map.data[index] = 100;
            }
        }
    }
    visualization::visualization() : it(nh), T_imu_to_camera(PARAM(T_imu_to_camera)),
                                     T_imu_to_laser(PARAM(T_imu_to_laser)),
                                     T_imu_to_wheel(PARAM(T_imu_to_wheel))
    {
        marker_pub = nh.advertise<visualization_msgs::Marker>("vis", 10);
        map_pub = nh.advertise<nav_msgs::OccupancyGrid>("map", 10);
        quit = false;
        visualization_thread = std::thread(std::bind(&visualization::do_visualization, this));
    }
    visualization::~visualization()
    {
        {
            std::unique_lock<std::mutex> lc(task_mutex);
            quit = true;
            task_cv.notify_one();
        }
        visualization_thread.join();
    }

    void visualization::do_image_show(const std::string &topic, const cv::Mat &src)
    {
        auto iter = image_pubs.find(topic);
        bool is_first = false;
        if (iter == image_pubs.end())
        {
            is_first = true;
            image_pubs[topic] = it.advertise(topic, 10);
        }
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", src).toImageMsg();

        if (is_first)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }
        image_pubs[topic].publish(msg);
    }

    void visualization::do_status_show(const Eigen::Isometry3d &tf, const TRAJECTORY_STATUS &status, const double &time)
    {
        ros::Time ros_t = ros::Time().fromSec(time);
        Eigen::Isometry3d imu_pose = tf;

        geometry_msgs::TransformStamped tf_msg;
        tf_msg.header.frame_id = "world";
        tf_msg.header.stamp = ros_t;

        tf_msg.child_frame_id = "imu";
        tf_msg.transform = eigen_tf_to_ros_geometry(imu_pose);
        tf_br.sendTransform(tf_msg);

        tf_msg.header.frame_id = "imu";
        tf_msg.child_frame_id = "camera";
        tf_msg.transform = eigen_tf_to_ros_geometry(T_imu_to_camera);
        tf_br.sendTransform(tf_msg);

        tf_msg.header.frame_id = "imu";
        tf_msg.child_frame_id = "wheel";
        tf_msg.transform = eigen_tf_to_ros_geometry(T_imu_to_wheel);
        tf_br.sendTransform(tf_msg);

        tf_msg.header.frame_id = "imu";
        tf_msg.child_frame_id = "laser";
        tf_msg.transform = eigen_tf_to_ros_geometry(T_imu_to_laser);
        tf_br.sendTransform(tf_msg);

        visualization_msgs::Marker camera_marker;
        camera_marker.header.frame_id = "camera";
        camera_marker.header.stamp = ros_t;
        camera_marker.ns = "lines";
        camera_marker.action = visualization_msgs::Marker::ADD;
        camera_marker.pose.orientation.w = 1.0;
        camera_marker.id = 2;
        camera_marker.type = visualization_msgs::Marker::LINE_LIST;
        camera_marker.scale.x = 0.01;
        camera_marker.scale.y = 0.01;
        camera_marker.color.a = 1.0;
        if (status == INITIALIZING)
            camera_marker.color.r = 1.0;
        else
            camera_marker.color.g = 1.0;

        // top
        geometry_msgs::Point p0;
        geometry_msgs::Point p1;
        geometry_msgs::Point p2;
        p1.x = -0.3;
        p1.y = -0.15;
        p1.z = 0.2;

        p2.x = 0.3;
        p2.y = -0.15;
        p2.z = 0.2;
        camera_marker.points.push_back(p1);
        camera_marker.points.push_back(p2);

        // right
        geometry_msgs::Point p3;
        p3.x = 0.3;
        p3.y = 0.15;
        p3.z = 0.2;
        camera_marker.points.push_back(p2);
        camera_marker.points.push_back(p3);

        // bottom
        geometry_msgs::Point p4;
        p4.x = -0.3;
        p4.y = 0.15;
        p4.z = 0.2;
        camera_marker.points.push_back(p3);
        camera_marker.points.push_back(p4);

        // left
        camera_marker.points.push_back(p4);
        camera_marker.points.push_back(p1);

        geometry_msgs::Point p5;
        geometry_msgs::Point p6;
        geometry_msgs::Point p7;

        p5.x = -0.25;
        p5.y = -0.15;
        p5.z = 0.2;

        p6.x = -0.25;
        p6.y = -0.1;
        p6.z = 0.2;

        p7.x = -0.3;
        p7.y = -0.1;
        p7.z = 0.2;

        camera_marker.points.push_back(p5);
        camera_marker.points.push_back(p6);

        camera_marker.points.push_back(p6);
        camera_marker.points.push_back(p7);

        camera_marker.points.push_back(p1);
        camera_marker.points.push_back(p0);

        camera_marker.points.push_back(p2);
        camera_marker.points.push_back(p0);

        camera_marker.points.push_back(p3);
        camera_marker.points.push_back(p0);

        camera_marker.points.push_back(p4);
        camera_marker.points.push_back(p0);

        marker_pub.publish(camera_marker);
    }

    void visualization::do_scan_to_show(const scan::ptr &scan_ptr)
    {

        visualization_msgs::Marker lines_marker;
        lines_marker.header.frame_id = "laser";
        lines_marker.header.stamp = ros::Time().fromSec(scan_ptr->time);
        lines_marker.ns = "laser_lines";
        lines_marker.action = visualization_msgs::Marker::ADD;
        lines_marker.pose.orientation.w = 1.0;
        lines_marker.id = 2;
        lines_marker.type = visualization_msgs::Marker::LINE_LIST;
        lines_marker.scale.x = 0.15;
        lines_marker.scale.y = 0.15;
        lines_marker.scale.z = 0.03;
        lines_marker.color.a = 0.5;
        lines_marker.color.r = 1.0;
        lines_marker.color.b = 1.0;

        visualization_msgs::Marker concer_marker;
        concer_marker.header.frame_id = "laser";
        concer_marker.header.stamp = ros::Time().fromSec(scan_ptr->time);
        concer_marker.ns = "laser_concer";
        concer_marker.action = visualization_msgs::Marker::ADD;
        concer_marker.pose.orientation.w = 1.0;
        concer_marker.id = 2;
        concer_marker.type = visualization_msgs::Marker::POINTS;
        concer_marker.scale.x = 0.15;
        concer_marker.scale.y = 0.15;
        concer_marker.scale.z = 0.15;
        concer_marker.color.a = 1.0;
        concer_marker.color.r = 1.0;

        int w = scan_ptr->w;
        int h = scan_ptr->h;

        for (int i = 0; i < scan_ptr->concers.size(); i++)
        {
            concer_marker.points.push_back(point_eigen_to_ros(
                scan_ptr->concers[i]));
        }

        for (int i = 0; i < scan_ptr->lines.size(); i++)
        {
            lines_marker.points.push_back(point_eigen_to_ros(
                scan_ptr->lines[i]->p1));
            lines_marker.points.push_back(point_eigen_to_ros(
                scan_ptr->lines[i]->p2));
        }

        visualization_msgs::Marker points_marker;
        points_marker.header.frame_id = "laser";
        points_marker.header.stamp = ros::Time().fromSec(scan_ptr->time);
        points_marker.ns = "laser_points";
        points_marker.action = visualization_msgs::Marker::ADD;
        points_marker.pose.orientation.w = 1.0;
        points_marker.id = 2;
        points_marker.type = visualization_msgs::Marker::POINTS;
        points_marker.scale.x = 0.01;
        points_marker.scale.y = 0.01;
        points_marker.scale.z = 0.01;
        points_marker.color.a = 0.5;
        points_marker.color.b = 1.0;
        points_marker.color.g = 1.0;
        for (int i = 0; i < scan_ptr->points.size(); i++)
        {
            points_marker.points.push_back(point_eigen_to_ros(scan_ptr->points[i]));
        }
        marker_pub.publish(points_marker);
        marker_pub.publish(concer_marker);
        marker_pub.publish(lines_marker);
    }
    void visualization::do_path_show(const std::string &topic, const std::vector<Eigen::Isometry3d> &path)
    {
        auto iter = path_pubs.find(topic);
        bool is_first = false;
        if (iter == path_pubs.end())
        {
            is_first = true;
            path_pubs[topic] = nh.advertise<nav_msgs::Path>(topic, 10);
        }
        if (is_first)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }
        nav_msgs::Path msg;
        msg.header.frame_id = "world";
        msg.header.stamp = ros::Time::now();
        for (int i = 0; i < path.size(); i++)
        {
            geometry_msgs::PoseStamped posestamp;
            Eigen::Quaterniond q(path[i].matrix().block<3, 3>(0, 0));

            posestamp.pose.position.x = path[i].matrix()(0, 3);
            posestamp.pose.position.y = path[i].matrix()(1, 3);
            posestamp.pose.position.z = path[i].matrix()(2, 3);

            posestamp.pose.orientation.w = q.w();
            posestamp.pose.orientation.x = q.x();
            posestamp.pose.orientation.y = q.y();
            posestamp.pose.orientation.z = q.z();

            msg.poses.push_back(posestamp);
        }
        path_pubs[topic].publish(msg);
    }
    void visualization::do_odom_to_show(const std::string &topic, const Eigen::Isometry3d &pose, const Eigen::Vector3d &linear, const Eigen::Vector3d &angular)
    {
        auto iter = odom_pubs.find(topic);
        bool is_first = false;
        if (iter == odom_pubs.end())
        {
            is_first = true;
            odom_pubs[topic] = nh.advertise<nav_msgs::Odometry>(topic, 10);
        }
        if (is_first)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }
        nav_msgs::Odometry msg;
        msg.header.frame_id = "world";
        msg.header.stamp = ros::Time::now();
        Eigen::Quaterniond q(pose.matrix().block<3, 3>(0, 0));

        msg.pose.pose.orientation.w = q.w();
        msg.pose.pose.orientation.x = q.x();
        msg.pose.pose.orientation.y = q.y();
        msg.pose.pose.orientation.z = q.z();

        msg.pose.pose.position.x = pose.matrix()(0, 3);
        msg.pose.pose.position.y = pose.matrix()(1, 3);
        msg.pose.pose.position.z = pose.matrix()(2, 3);

        msg.twist.twist.angular.x = angular(0);
        msg.twist.twist.angular.y = angular(1);
        msg.twist.twist.angular.z = angular(2);

        msg.twist.twist.linear.x = linear(0);
        msg.twist.twist.linear.y = linear(1);
        msg.twist.twist.linear.z = linear(2);
        odom_pubs[topic].publish(msg);
    }
    void visualization::do_laser_map_to_show(const laser_map::ptr &laser_map_ptr)
    {

        visualization_msgs::Marker points_marker;
        points_marker.header.frame_id = "world";
        points_marker.header.stamp = ros::Time::now();
        points_marker.ns = "laser_map";
        points_marker.action = visualization_msgs::Marker::ADD;
        points_marker.pose.orientation.w = 1.0;
        points_marker.id = 2;
        points_marker.type = visualization_msgs::Marker::POINTS;
        points_marker.scale.x = 0.02;
        points_marker.scale.y = 0.02;
        points_marker.color.a = 1.0;
        points_marker.color.b = 0.0;
        points_marker.color.g = 0.0;
        points_marker.color.r = 0.0;

        double max_x = TIME_MIN, max_y = TIME_MIN;
        double min_x = TIME_MAX, min_y = TIME_MAX;
        double map_res = 0.05;
        for (int i = 0; i < laser_map_ptr->submaps.size(); i++)
        {
            Eigen::Isometry3d T_w_l = lie::make_tf(laser_map_ptr->submaps[i]->current_p, laser_map_ptr->submaps[i]->current_q) *
                                      PARAM(T_imu_to_laser);
            for (int j = 0; j < laser_map_ptr->submaps[i]->scan_ptr->points.size(); j++)
            {
                Eigen::Vector3d p = T_w_l *
                                    laser_map_ptr->submaps[i]->scan_ptr->points[j];
                points_marker.points.push_back(point_eigen_to_ros(p));
                if (p(0) > max_x)
                    max_x = p(0);
                if (p(0) < min_x)
                    min_x = p(0);
                if (p(1) > max_y)
                    max_y = p(1);
                if (p(1) < min_y)
                    min_y = p(1);
            }
        }

        Eigen::Vector3d origin((max_x + min_x) / 2, (max_y + min_y) / 2, 0);

        int w = (max_x - min_x) / map_res + 1;
        int h = (max_y - min_y) / map_res + 1;

        nav_msgs::OccupancyGrid map;
        map.header.frame_id = "world";
        map.header.stamp = ros::Time::now();
        map.info.origin.position.x = min_x;
        map.info.origin.position.y = min_y;
        map.info.origin.position.z = 0;

        map.info.origin.orientation.x = 0;
        map.info.origin.orientation.y = 0;
        map.info.origin.orientation.z = 0;
        map.info.origin.orientation.w = 1;

        map.info.width = w;
        map.info.height = h;
        map.info.resolution = map_res;
        map.data.resize(w * h);
        for (int i = 0; i < w * h; i++)
        {
            map.data[i] = -1;
        }

        for (int i = 0; i < laser_map_ptr->submaps.size(); i++)
        {
            Eigen::Isometry3d T_w_l = lie::make_tf(laser_map_ptr->submaps[i]->current_p, laser_map_ptr->submaps[i]->current_q) *
                                      PARAM(T_imu_to_laser);
            for (int j = 0; j < laser_map_ptr->submaps[i]->scan_ptr->points.size(); j++)
            {
                Eigen::Vector3d p = T_w_l *
                                    laser_map_ptr->submaps[i]->scan_ptr->points[j];
                Eigen::Vector3d emit_origin = T_w_l.matrix().block<3, 1>(0, 3);
                update_occupancy_grid(map, emit_origin, p);
            }
        }

        map_pub.publish(map);
        marker_pub.publish(points_marker);
    }

    void visualization::do_feature_map_to_show(const feature_map::ptr &feature_map_ptr)
    {
        visualization_msgs::Marker frozen_points_marker;
        frozen_points_marker.header.frame_id = "world";
        frozen_points_marker.header.stamp = ros::Time::now();
        frozen_points_marker.ns = "feature_map_frozen";
        frozen_points_marker.action = visualization_msgs::Marker::ADD;
        frozen_points_marker.pose.orientation.w = 1.0;
        frozen_points_marker.id = 2;
        frozen_points_marker.type = visualization_msgs::Marker::POINTS;
        frozen_points_marker.scale.x = 0.03;
        frozen_points_marker.scale.y = 0.03;
        frozen_points_marker.scale.z = 0.03;
        frozen_points_marker.color.a = 1.0;
        frozen_points_marker.color.b = 1.0;
        frozen_points_marker.color.g = 0.0;
        frozen_points_marker.color.r = 0.0;

        visualization_msgs::Marker opting_points_marker;
        opting_points_marker.header.frame_id = "world";
        opting_points_marker.header.stamp = ros::Time::now();
        opting_points_marker.ns = "feature_map_opting";
        opting_points_marker.action = visualization_msgs::Marker::ADD;
        opting_points_marker.pose.orientation.w = 1.0;
        opting_points_marker.id = 2;
        opting_points_marker.type = visualization_msgs::Marker::POINTS;
        opting_points_marker.scale.x = 0.03;
        opting_points_marker.scale.y = 0.03;
        opting_points_marker.scale.z = 0.03;
        opting_points_marker.color.a = 1.0;
        opting_points_marker.color.b = 0.0;
        opting_points_marker.color.g = 0.0;
        opting_points_marker.color.r = 1.0;

        for (auto &[id, point] : feature_map_ptr->world_points)
        {
            if (point.has_frozen)
            {
                frozen_points_marker.points.push_back(point_eigen_to_ros(point.world_point));
            }
            else
            {
                opting_points_marker.points.push_back(point_eigen_to_ros(point.world_point));
            }
        }

        marker_pub.publish(frozen_points_marker);
        marker_pub.publish(opting_points_marker);
    }
    void visualization::do_laser_match_to_show(const laser_match::ptr &laser_match_ptr)
    {
        visualization_msgs::Marker lines_marker1;

        lines_marker1.header.frame_id = "world";
        lines_marker1.header.stamp = ros::Time().now();
        lines_marker1.ns = "match_line1";
        lines_marker1.action = visualization_msgs::Marker::ADD;
        lines_marker1.pose.orientation.w = 1.0;
        lines_marker1.id = 2;
        lines_marker1.type = visualization_msgs::Marker::LINE_LIST;
        lines_marker1.scale.x = 0.06;
        lines_marker1.scale.y = 0.06;
        lines_marker1.scale.z = 0.06;
        lines_marker1.color.a = 0.5;

        lines_marker1.color.r = 1.0;

        visualization_msgs::Marker lines_marker2;

        lines_marker2.header.frame_id = "world";
        lines_marker2.header.stamp = ros::Time().now();
        lines_marker2.ns = "match_line2";
        lines_marker2.action = visualization_msgs::Marker::ADD;
        lines_marker2.pose.orientation.w = 1.0;
        lines_marker2.id = 2;
        lines_marker2.type = visualization_msgs::Marker::LINE_LIST;
        lines_marker2.scale.x = 0.03;
        lines_marker2.scale.y = 0.03;
        lines_marker2.scale.z = 0.03;
        lines_marker2.color.a = 0.5;
        lines_marker2.color.b = 1.0;

        if (laser_match_ptr)
        {
            Eigen::Isometry3d T_w_l1 = lie::make_tf(laser_match_ptr->p1, laser_match_ptr->q1) * PARAM(T_imu_to_laser);
            Eigen::Isometry3d T_w_l2 = lie::make_tf(laser_match_ptr->p2, laser_match_ptr->q2) * PARAM(T_imu_to_laser);

            for (int i = 0; i < laser_match_ptr->lines1.size(); i++)
            {
                lines_marker1.points.push_back(point_eigen_to_ros(
                    T_w_l1 * laser_match_ptr->lines1[i]->p1));
                lines_marker1.points.push_back(point_eigen_to_ros(
                    T_w_l1 * laser_match_ptr->lines1[i]->p2));

                lines_marker2.points.push_back(point_eigen_to_ros(
                    T_w_l2 * laser_match_ptr->lines2[i]->p1));
                lines_marker2.points.push_back(point_eigen_to_ros(
                    T_w_l2 * laser_match_ptr->lines2[i]->p2));
            }
        }

        marker_pub.publish(lines_marker1);
        marker_pub.publish(lines_marker2);
    }
    void visualization::do_visualization()
    {
        while (1)
        {
            std::string im_topic;
            cv::Mat im_src;
            bool has_im_task = false;

            Eigen::Isometry3d status_tf;
            TRAJECTORY_STATUS status_statu;
            double status_time;
            bool has_status_task = false;

            std::string path_topic;
            std::vector<Eigen::Isometry3d> path;
            bool has_path_task = false;

            scan::ptr scan_ptr;
            bool has_scan_task = false;

            laser_map::ptr laser_map_ptr;
            bool has_laser_map_taks = false;

            std::string odom_topic;
            Eigen::Isometry3d odom_pose;
            Eigen::Vector3d odom_linear;
            Eigen::Vector3d odom_angular;
            bool has_odom_tasks = false;

            feature_map::ptr feature_map_ptr;
            bool has_feature_map_tasks = false;

            laser_match::ptr laser_match_ptr;
            bool has_laser_match_tasks = false;

            {
                std::unique_lock<std::mutex> lc(task_mutex);
                while (status_tasks.empty() &&
                       image_tasks.empty() &&
                       path_tasks.empty() &&
                       scan_tasks.empty() &&
                       laser_map_tasks.empty() &&
                       odom_tasks.empty() &&
                       feature_map_tasks.empty() &&
                       laser_match_tasks.empty())
                {
                    if (quit)
                        return;
                    task_cv.wait(lc);
                }
                if (!image_tasks.empty())
                {
                    std::tie(im_topic, im_src) = image_tasks.front();
                    image_tasks.pop_front();
                    has_im_task = true;
                }
                if (!status_tasks.empty())
                {
                    std::tie(status_tf, status_statu, status_time) = status_tasks.front();
                    status_tasks.pop_front();
                    has_status_task = true;
                }
                if (!path_tasks.empty())
                {
                    std::tie(path_topic, path) = path_tasks.front();
                    path_tasks.pop_front();
                    has_path_task = true;
                }
                if (!scan_tasks.empty())
                {
                    scan_ptr = scan_tasks.front();
                    scan_tasks.pop_front();
                    has_scan_task = true;
                }
                if (!laser_map_tasks.empty())
                {
                    laser_map_ptr = laser_map_tasks.front();
                    laser_map_tasks.pop_front();
                    has_laser_map_taks = true;
                }
                if (!odom_tasks.empty())
                {
                    std::tie(odom_topic, odom_pose, odom_linear, odom_angular) = odom_tasks.front();
                    odom_tasks.pop_front();
                    has_odom_tasks = true;
                }
                if (!feature_map_tasks.empty())
                {
                    std::tie(feature_map_ptr) = feature_map_tasks.front();
                    feature_map_tasks.pop_front();
                    has_feature_map_tasks = true;
                }
                if (!laser_match_tasks.empty())
                {
                    std::tie(laser_match_ptr) = laser_match_tasks.front();
                    laser_match_tasks.pop_front();
                    has_laser_match_tasks = true;
                }
            }
            if (has_im_task)
            {
                do_image_show(im_topic, im_src);
            }
            if (has_status_task)
            {
                do_status_show(status_tf, status_statu, status_time);
            }
            if (has_path_task)
            {
                do_path_show(path_topic, path);
            }
            if (has_scan_task)
            {
                do_scan_to_show(scan_ptr);
            }
            if (has_laser_map_taks)
            {
                do_laser_map_to_show(laser_map_ptr);
            }
            if (has_odom_tasks)
            {
                do_odom_to_show(odom_topic, odom_pose, odom_linear, odom_angular);
            }
            if (has_feature_map_tasks)
            {
                do_feature_map_to_show(feature_map_ptr);
            }
            if (has_laser_match_tasks)
            {
                do_laser_match_to_show(laser_match_ptr);
            }
            if (quit)
                break;
        }
    }

    void visualization::add_image_to_show(const std::string &topic, const cv::Mat &src)
    {
        {
            std::unique_lock<std::mutex> lc(task_mutex);
            image_tasks.emplace_back(topic, src);
        }
        task_cv.notify_one();
    }
    void visualization::add_status_to_show(const Eigen::Isometry3d &tf, const TRAJECTORY_STATUS &status, const double &time)
    {
        {
            std::unique_lock<std::mutex> lc(task_mutex);
            status_tasks.emplace_back(tf, status, time);
        }
        task_cv.notify_one();
    }
    void visualization::add_path_to_show(const std::string &topic, const std::vector<Eigen::Isometry3d> &path)
    {
        {
            std::unique_lock<std::mutex> lc(task_mutex);
            path_tasks.emplace_back(topic, path);
        }
        task_cv.notify_one();
    }
    void visualization::add_scan_to_show(const scan::ptr &scan_ptr)
    {
        {
            std::unique_lock<std::mutex> lc(task_mutex);
            scan_tasks.emplace_back(scan_ptr);
        }
        task_cv.notify_one();
    }
    void visualization::add_laser_map_to_show(const laser_map::ptr &laser_map_ptr)
    {
        {
            std::unique_lock<std::mutex> lc(task_mutex);
            laser_map_tasks.emplace_back(laser_map_ptr);
        }
        task_cv.notify_one();
    }
    void visualization::add_odom_to_show(const std::string &topic, const Eigen::Isometry3d &pose, const Eigen::Vector3d &linear, const Eigen::Vector3d &angular)
    {
        {
            std::unique_lock<std::mutex> lc(task_mutex);
            odom_tasks.emplace_back(topic, pose, linear, angular);
        }
        task_cv.notify_one();
    }
    void visualization::add_feature_map_to_show(const feature_map::ptr &feature_map_ptr)
    {
        {
            std::unique_lock<std::mutex> lc(task_mutex);
            feature_map_tasks.emplace_back(feature_map_ptr);
        }
        task_cv.notify_one();
    }
    void visualization::add_laser_match_to_show(const laser_match::ptr &laser_match_ptr)
    {
        {
            std::unique_lock<std::mutex> lc(task_mutex);
            laser_match_tasks.emplace_back(laser_match_ptr);
        }
        task_cv.notify_one();
    }

} // namespace lvio_2d