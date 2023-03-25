#pragma once
#include "trajectory/camera_type.h"
#include "utilies/utilies.h"
#include <Eigen/Dense>
#include <deque>
#include <opencv2/opencv.hpp>
#include <trajectory/sensor.h>
#include <tuple>
#include <unordered_map>

namespace lvio_2d
{
    struct camera_match
    {
        using ptr = std::shared_ptr<camera_match>;

        std::vector<Eigen::Vector3d> pts1;
        std::vector<Eigen::Vector3d> pts2;

        std::vector<long long> id;
        void update_scale(const Eigen::Isometry3d &T_1_2);
        std::vector<double> scales2;
        cv::Mat m2;
    };

    class camera_manager
    {
    public:
        camera_manager();
        camera_match::ptr add_frame(sensor::camera::u_ptr &src);

        void abort_id(const long long &id);

    private:
        cv::Mat last_frame;

        Eigen::Matrix3d camera_k_inverse;
        Eigen::Matrix3d camera_k;

        long long feature_id;
        std::vector<long long> last_ids;
        std::vector<cv::Point2f> last_features;
    };

    // 每个world point对应一个结构
    struct feature_info
    {
        using ptr = std::shared_ptr<feature_info>;

        long long id;      // 唯一id
        bool has_estimate; // 用来判断是否已经有初值了，如果观测次数多于一定次数，则会给初值，有初值之后不再使用tri_tfs和tri_cam_points

        bool has_ready_for_opt; // 是否允许进行优化
        int linearized_times;

        int ob_times; // 观测到的次数
        double error; // 暂时不用

        // use for optimization
        std::deque<int> frame_indexs;              // 按顺序记录再哪一个帧被观测到，需要和trajectory里面的frame infos的index对应起来
        std::deque<Eigen::Vector3d> camera_points; // 对应的观测值，其他需求和上面一样
        Eigen::Vector3d world_point;               // 在世界坐标下的值

        // use for triangulate
        std::vector<Eigen::Isometry3d> tri_tfs;
        std::vector<Eigen::Vector3d> tri_cam_points;

        feature_info(const long long &id_);
        void push_frame(const int &frame_index,
                        const Eigen::Vector3d &camera_point,
                        const Eigen::Isometry3d &current_tf);

        bool pop_frame(const int &k);
        bool estimate_initial_value();
    };

    class feature_manger
    {
    public:
        feature_manger();
        void add_match(const camera_match::ptr &match_ptr, const long long &current_index,
                       const Eigen::Vector3d &current_p,
                       const Eigen::Vector3d &current_q);

        std::vector<long long> pop_frame(const int &k);

        std::vector<long long> remove_all_features_without_lastest_frame();

        long long remove_feature(const long long &id);
        std::map<long long, feature_info::ptr> &get_feature_infos();
        std::vector<feature_info::ptr> &get_lastest_frame_features();
        feature_map::ptr &get_feature_map_ptr();

    private:
        std::map<long long, feature_info::ptr> feature_infos;
        std::vector<feature_info::ptr> lastest_frame_features;
        feature_map::ptr feature_map_ptr;
    };
}; // namespace lvio_2d