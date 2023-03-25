#pragma once
#include "trajectory/keyframe_type.h"
#include "trajectory/trajectory_type.h"
#include "utilies/utilies.h"
#include <fstream>
#include "utilies/record.h"
namespace lvio_2d
{
    class keyframe_manager
    {
    public:
        keyframe_manager();
        ~keyframe_manager();

        // 暂时不考虑相机帧
        void add_keyframe(const frame_info::ptr &frame);

        void update_other_frame(const std::deque<frame_info::ptr> &frame_infos);

        void show_laser_map();

    private:
        std::deque<frame_info::ptr> cache_keyframes;
        std::deque<frame_info::ptr> keyframe_queue;
        std::deque<Eigen::Isometry3d> tfs_tracking;
        std::deque<frame_info::ptr> other_frame;
        std::vector<edge::ptr> seq_edges;
        std::vector<edge::ptr> loop_edges;
        std::vector<laser_map_feature::ptr> laser_map_features;
        // modify_delta_tf=new*old.inverse();
        Eigen::Isometry3d modify_delta_tf; // 记录最后一帧keyframe的修正tf

        int laser_frame_count;

        std::thread backend_thread;
        std::thread show_thread;
        std::mutex mu;
        std::condition_variable task_cv;

        std::mutex show_mu;
        bool quit;
        bool has_loop_wait_for_solve;
        double last_show_time;
        double last_solve_time;
        int last_loop_index;
        edge::ptr laser_loop_detect();
        edge::ptr camera_loop_detect();

        void main_loop();
        void show_loop();

        void do_add_keyframe(const frame_info::ptr &frame_ptr);
        void get_init();
        laser_map_feature::ptr spawn_laser_map_feature();
        void solve();
        bool is_time_to_solve();
        bool is_time_to_show();
    };
} // namespace lvio_2d