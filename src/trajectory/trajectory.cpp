#include "trajectory/trajectory.h"
#include "timerAndColor/timer.h"
#include "utilies/visualization.h"

static bool is_static(const Eigen::Isometry3d &delta_tf,
                      const double &p_motion_threshold,
                      const double &q_motion_threshold)
{
    auto [delta_p, delta_q] =
        lie::log_SE3(delta_tf);
    return delta_p.norm() < p_motion_threshold && delta_q.norm() < q_motion_threshold;
}
static Eigen::Isometry3d wheel_delta_to_imu_delta(const Eigen::Isometry3d &wheel_delta)
{
    return PARAM(T_imu_to_wheel) * wheel_delta * PARAM(T_imu_to_wheel).inverse();
};

static Eigen::Isometry3d wheel_delta_to_camera_delta(const Eigen::Isometry3d &wheel_delta)
{
    auto T_camera_to_wheel = PARAM(T_imu_to_camera).inverse() * PARAM(T_imu_to_wheel);
    return T_camera_to_wheel * wheel_delta * T_camera_to_wheel.inverse();
};
static Eigen::Isometry3d wheel_delta_to_laser_delta(const Eigen::Isometry3d &wheel_delta)
{
    auto T_laser_to_wheel = PARAM(T_imu_to_laser).inverse() * PARAM(T_imu_to_wheel);
    return T_laser_to_wheel * wheel_delta * T_laser_to_wheel.inverse();
}

namespace lvio_2d
{
    trajectory::trajectory() : recorder("traj")
    {
        init_current_status();
    }
    trajectory::~trajectory()
    {
        o_fstream.close();
    }
    void trajectory::init_current_status()
    {
        status = INITIALIZING;
        last_keyframe_tf = Eigen::Isometry3d::Identity();
        last_time = TIME_MIN;
        current_time = TIME_MIN;

        std::tie(current_p, current_q) = lie::log_SE3(PARAM(T_imu_to_wheel).inverse());

        current_v = Eigen::Matrix<double, 3, 1>::Zero();
        current_bs = Eigen::Matrix<double, 6, 1>::Zero();
        current_angular_local = Eigen::Matrix<double, 3, 1>::Zero();

        wheel_odom_inited = false;
        imu_inited = false;

        current_index = -1;
        last_laser_index = -1;
        last_camera_index = -1;

        if (PARAM(output_tum))
        {
            o_path = PARAM(output_dir) + "fornt_end.txt";
            o_fstream.open(o_path, std::ios_base::out);
            std::cout << "open " << o_path << ":" << o_fstream.is_open() << std::endl;
            o_fstream << "#Time px py pz qx qy qz qw\n";
            o_fstream << std::fixed;
            o_fstream << std::setprecision(10);
        }
    }
    void trajectory::add_sensor_data(sensor::wheel_odom::u_ptr &wheel_odom_data_ptr)
    {
        double time = wheel_odom_data_ptr->time_stamp;
        if (wheel_odom_preintegration_.add_wheel_odom_measure(wheel_odom_data_ptr))
            wheel_odom_inited = true;
    }
    void trajectory::add_sensor_data(sensor::imu::u_ptr &imu_data_ptr)
    {
        double time = imu_data_ptr->time_stamp;
        if (imu_preintegraption_.add_imu_measure(imu_data_ptr))
            imu_inited = true;
    }

    void trajectory::update_current_status(const Eigen::Isometry3d &delta_tf, const double &time)
    {
        // if (status == INITIALIZING)
        {
            Eigen::Isometry3d new_tf = lie::make_tf(current_p, current_q) * delta_tf;
            auto [new_p, new_q] = lie::log_SE3(new_tf);
            current_p = new_p;
            current_q = new_q;
            current_time = time;
            return;
        }

        double dt = time - current_time;
        current_p += current_v * dt;
        current_q = lie::log_SO3<double>(lie::exp_so3(current_q) * lie::exp_so3<double>(current_angular_local * dt));
        current_time = time;
    }

    void trajectory::update_path_and_pub(const Eigen::Isometry3d &delta_tf)
    {
        // debug
        tracking_path.push_back(lie::make_tf(frame_infos.back()->p, frame_infos.back()->q));
        if (odom_path.empty())
        {
            odom_path.push_back(tracking_path[0]);
        }
        else
        {
            odom_path.push_back(odom_path.back() * delta_tf);
        }
        visualization::get_ptr()->add_path_to_show("tracking_path", tracking_path);
        visualization::get_ptr()->add_path_to_show("odom_path", odom_path);

        if (frame_infos.size() > 0 && odom_path.size() > 1)
        {
            int n = frame_infos.size();
            double dt = current_time - last_time;
            Eigen::Isometry3d pose_from_odom_i = odom_path[odom_path.size() - 2];
            Eigen::Isometry3d pose_from_odom_j = odom_path.back();

            auto [delta_p, delta_q] = lie::log_SE3(pose_from_odom_i.inverse() * pose_from_odom_j);

            Eigen::Vector3d linear_from_odom = delta_p / dt;
            Eigen::Vector3d angular_from_odom = delta_q / dt;

            Eigen::Isometry3d pose_from_tracking_j = tracking_path.back();
            Eigen::Vector3d wv_from_tracking_j = frame_infos.back()->v;
            Eigen::Vector3d v_from_tracking_j = pose_from_tracking_j.inverse().matrix().block<3, 3>(0, 0) * wv_from_tracking_j;

            visualization::get_ptr()->add_odom_to_show("ob_wheel_odom", pose_from_odom_j, linear_from_odom, angular_from_odom);
            visualization::get_ptr()->add_odom_to_show("ob_tracking_odom", pose_from_tracking_j, v_from_tracking_j, current_angular_local);
        }
        last_time = current_time;
    }

    void trajectory::add_sensor_data(sensor::laser::u_ptr &laser_data_ptr)
    {
        double time = laser_data_ptr->time_stamp;
        if (status == TRACKING)
        {
            Eigen::Isometry3d T_w_laser = lie::make_tf(current_p, current_q) * PARAM(T_imu_to_laser);
            Eigen::Matrix3d R_w_laser = T_w_laser.matrix().block<3, 3>(0, 0);
            Eigen::Matrix3d R_i_l = PARAM(T_imu_to_laser).matrix().block<3, 3>(0, 0);
            Eigen::Vector3d tmp_angular = lie::log_SO3<double>(R_i_l.transpose() * lie::exp_so3(current_angular_local) * R_i_l);

            laser_data_ptr->correct(R_w_laser.transpose() * current_v, tmp_angular);
        }

        if (!imu_inited)
        {
            ROS_WARN("abort camera msg.wait for imu  init.");
            return;
        }
        if (!wheel_odom_inited)
        {
            ROS_WARN("abort camera msg.wait for wheel odom init.");
            return;
        }
        auto wheel_result_filter = wheel_odom_preintegration_.get_preintegraption_result();
        auto laser_delta_filter = wheel_delta_to_laser_delta(wheel_result_filter->delta_Tij);

        if (status == INITIALIZING && is_static(laser_delta_filter, PARAM(p_motion_threshold), PARAM(q_motion_threshold)))
            return;

        if (PARAM(enable_camera))
            if (!frame_infos.empty() && frame_infos.back()->type == frame_info::laser)
                return;

        if (status == TRACKING && imu_preintegraption_.Dt < PARAM(min_delta_t))
            return;

        current_index++;

        // 对齐时间戳
        wheel_odom_preintegration_.update_only_t(time);
        imu_preintegraption_.update_only_t(time);

        auto wheel_result = wheel_odom_preintegration_.get_preintegraption_result();
        auto imu_reuslt = imu_preintegraption_.get_preintegraption_result();
        wheel_odom_preintegration_.reset_wheel_odom_measure(time);
        imu_preintegraption_.reset_imu_measure(time,
                                               current_bs.block<3, 1>(0, 0),
                                               current_bs.block<3, 1>(3, 0));

        current_angular_local = imu_reuslt->X.block<3, 1>(gamma_index, 0) /
                                imu_reuslt->Dt;

        Eigen::Isometry3d delta_tf = wheel_delta_to_imu_delta(wheel_result->delta_Tij);

        // 获取当前的角速度
        update_current_status(delta_tf, time);

        recorder.begin_record();
        auto scan_ptr = laser_manger_.spawn_scan(laser_data_ptr);
        recorder.end_record("spawn_scan");

        recorder.add_record("lines each frame", scan_ptr->lines.size());

        laser_match::ptr laser_match = nullptr;
        if (status == INITIALIZING)
        {
            laser_match = laser_manger_.match_with_front(scan_ptr, current_p, current_q);
            laser_manger_.add_scan(scan_ptr, current_p, current_q);
        }
        else
        {
            // laser_match = laser_manger_.match_with_back(scan_ptr, current_p, current_q);
            recorder.begin_record();
            laser_match = laser_manger_.match_with_ref(scan_ptr, current_p, current_q);
            recorder.end_record("match_line");
        }
        auto current_frame = frame_info::create(current_time,
                                                current_p,
                                                current_q,
                                                current_v,
                                                current_bs,
                                                imu_reuslt,
                                                wheel_result);
        current_frame->add_laser_match(laser_match);
        if (laser_match)
        {
            if (PARAM(enable_laser_vis))
                visualization::get_ptr()->add_scan_to_show(laser_match->scan2);
        }
        assert(current_frame->type == frame_info::laser);
        last_laser_index = current_index;
        frame_infos.push_back(current_frame);
        if (status == INITIALIZING)
        {
            if (check_and_processing_initialize())
            {
                status = TRACKING;
            }
            return;
        }

        do_tracking();
        {
            Eigen::Isometry3d tf_w_l = lie::make_tf(current_p, current_q) * PARAM(T_imu_to_laser);
            for (int i = 0; i < laser_match->scan2->concers.size(); i++)
                acc_concers.push_back(tf_w_l * laser_match->scan2->concers[i]);
        }
        if (PARAM(enable_laser_vis))
            visualization::get_ptr()->add_laser_match_to_show(laser_match);

        Eigen::Isometry3d current_laser_tf = lie::make_tf(current_p, current_q) * PARAM(T_imu_to_laser);
        Eigen::Isometry3d delta_laser_tf = last_keyframe_tf.inverse() * current_laser_tf;

        int n_match_size = 0;
        if (laser_match)
            n_match_size = laser_match->lines1.size();
        recorder.add_record("match line size", n_match_size);

        int n_no_match_size = scan_ptr->lines.size() - n_match_size;

        if (!is_static(delta_laser_tf,
                       PARAM(key_frame_p_motion_threshold),
                       PARAM(key_frame_q_motion_threshold)) ||
            n_match_size < n_no_match_size)
        {
            // laser_manger_.add_scan(scan_ptr, current_p, current_q);
            recorder.add_record("corner each keyframe", acc_concers.size());
            frame_infos.back()->set_acc_concers(acc_concers);

            frame_infos.back()->set_key_frame();
            acc_concers.clear();

            last_keyframe_tf = current_laser_tf;
        }
        recorder.begin_record();
        laser_manger_.add_scan(scan_ptr, current_p, current_q);
        recorder.end_record("add scan to ref");

        update_path_and_pub(wheel_delta_to_imu_delta(wheel_result->delta_Tij));
        return;
    }

    void trajectory::add_sensor_data(sensor::camera::u_ptr &camera_image_ptr)
    {
        double time = camera_image_ptr->time_stamp;
        if (!imu_inited)
        {
            ROS_WARN("abort camera msg.wait for imu  init.");
            return;
        }
        if (!wheel_odom_inited)
        {
            ROS_WARN("abort camera msg.wait for wheel odom init.");
            return;
        }
        auto wheel_result_filter = wheel_odom_preintegration_.get_preintegraption_result();
        auto camera_delta_filter = wheel_delta_to_camera_delta(wheel_result_filter->delta_Tij);
        auto camera_match = camera_manger_.add_frame(camera_image_ptr);

        if (status == INITIALIZING && is_static(camera_delta_filter, PARAM(p_motion_threshold),
                                                PARAM(q_motion_threshold)))
            return;
        if (PARAM(enable_laser))
        {
            if (!frame_infos.empty() && frame_infos.back()->type == frame_info::camera)
                return;
            if (frame_infos.empty()) // the first frame must be laser
                return;
        }
        if (status == TRACKING)
        {
            if (imu_preintegraption_.Dt < PARAM(min_delta_t))
            {
                // ROS_WARN("too short Dt:%lf.abort", imu_preintegraption_.Dt);
                return;
            }
        }
        current_index++;

        // 对齐时间戳
        wheel_odom_preintegration_.update_only_t(time);
        imu_preintegraption_.update_only_t(time);

        auto wheel_result = wheel_odom_preintegration_.get_preintegraption_result();
        auto imu_reuslt = imu_preintegraption_.get_preintegraption_result();
        wheel_odom_preintegration_.reset_wheel_odom_measure(time);
        imu_preintegraption_.reset_imu_measure(time,
                                               current_bs.block<3, 1>(0, 0),
                                               current_bs.block<3, 1>(3, 0));

        current_angular_local = imu_reuslt->X.block<3, 1>(gamma_index, 0) /
                                imu_reuslt->Dt;

        Eigen::Isometry3d delta_tf = wheel_delta_to_imu_delta(wheel_result->delta_Tij);

        // 获取当前的角速度
        update_current_status(delta_tf, time);

        feature_manger_.add_match(camera_match, current_index, current_p, current_q);

        auto current_frame = frame_info::create(current_time,
                                                current_p,
                                                current_q,
                                                current_v,
                                                current_bs,
                                                imu_reuslt,
                                                wheel_result);

        Eigen::Isometry3d T_1_2_camera = Eigen::Isometry3d::Identity();
        if (last_camera_index > -1)
        {
            Eigen::Isometry3d t1 = lie::make_tf(frame_infos[last_camera_index]->p, frame_infos[last_camera_index]->q);
            Eigen::Isometry3d t2 = lie::make_tf(current_p, current_q);
            T_1_2_camera = PARAM(T_imu_to_camera).inverse() * t1.inverse() * t2 * PARAM(T_imu_to_camera);
        }

        current_frame->type = frame_info::camera;
        if (camera_match)
            current_frame->image = camera_match->m2;
        assert(current_frame->type == frame_info::camera);
        last_camera_index = current_index;
        frame_infos.push_back(current_frame);

        if (status == INITIALIZING)
        {
            if (camera_match)
            {
                show_lastest_image("tracking");
            }
            if (check_and_processing_initialize())
            {
                status = TRACKING;
            }
            return;
        }
        do_tracking();
        if (camera_match)
        {
            show_lastest_image("tracking");
        }

        update_path_and_pub(wheel_delta_to_imu_delta(wheel_result->delta_Tij));
        return;
    }
    bool trajectory::check_and_processing_initialize()
    {
        std::cout << "check_and_processing_initialize" << std::endl;
        // check about camera
        if (frame_infos.size() < PARAM(slide_window_size))
            return false;
        int k = 0;

        // check about laser
        int is_first_laser = true;
        for (int i = 0; i < frame_infos.size(); i++)
        {
            if (frame_infos[i]->type == frame_info::laser)
            {
                if (is_first_laser)
                {
                    is_first_laser = false;
                    continue;
                }
                if (frame_infos[i]->laser_match_ptr == nullptr)
                {
                    k = i + 1;
                    break;
                }
                if (frame_infos[i]->laser_match_ptr->lines2.size() < 2)
                {
                    k = i + 1;
                    break;
                }
            }
        }
        if (k > 0)
        {
            pop_frame(frame_infos.size());
            laser_manger_.clear_all_scan();
            init_current_status();
            return false;
        }
        estimate_features(true);
        auto all_features = feature_manger_.get_feature_infos();
        for (auto &[id, feature_ptr] : all_features)
            feature_ptr->has_ready_for_opt = true;
        // frame infos
        for (int i = 0; i < frame_infos.size(); i++)
        {
            if (frame_infos[i]->type == frame_info::camera)
            {
                std::cout << "frame : " << i << "\t"
                          << "camera" << std::endl;
            }
            if (frame_infos[i]->type == frame_info::laser)
            {
                std::cout << "frame : " << i << "\t"
                          << "laser" << std::endl;
                std::cout << " lines:\t";
                if (!frame_infos[i]->laser_match_ptr)
                    std::cout << 0 << std::endl;
                else
                    std::cout << frame_infos[i]->laser_match_ptr->lines1.size() << std::endl;
            }
            if (frame_infos[i]->type == frame_info::unknow)
                std::cout << "frame : " << i << "\t"
                          << "unkonow" << std::endl;
        }
        show_lastest_image("before_initilization");
        opt_solver.init_solve(frame_infos, feature_manger_);

        int n_laser = 0;
        auto &key_frame = laser_manger_.get_keyframs();
        for (int i = 0; i < frame_infos.size(); i++)
        {
            if (frame_infos[i]->type == frame_info::laser)
            {
                key_frame[n_laser]->current_p = frame_infos[i]->p;
                key_frame[n_laser]->current_q = frame_infos[i]->q;
                n_laser++;
            }
        }
        filter_outlier_world_point();
        show_lastest_image("after_initilization");
        current_p = frame_infos.back()->p;
        current_q = frame_infos.back()->q;
        current_v = frame_infos.back()->v;
        current_bs = frame_infos.back()->bs;

        laser_manger_.clear_all_scan();
        for (int i = 0; i < frame_infos.size(); i++)
        {

            if (frame_infos[i]->type == frame_info::laser)
            {
                if (frame_infos[i]->laser_match_ptr)
                    if (frame_infos[i]->laser_match_ptr->scan2)
                        laser_manger_.add_scan(frame_infos[i]->laser_match_ptr->scan2,
                                               frame_infos[i]->p,
                                               frame_infos[i]->q);
            }
        }
        opt_solver.marginalization(frame_infos, feature_manger_);

        acc_concers.clear();
        pop_frame_for_tracking();

        last_keyframe_tf = lie::make_tf(current_p, current_q);

        return true;
    }
    void trajectory::pop_frame(int k)
    {
        if (k <= 0)
            return;

        std::deque<frame_info::ptr> tmp_frame_infos;
        for (int i = 0; i < k; i++)
        {
            auto tmp_frame = frame_infos.front();
            frame_infos.pop_front();
            if (tmp_frame->is_key_frame)
            {
                keyframe_manager_.add_keyframe(tmp_frame);
                tmp_frame_infos.clear();
            }
            else
            {
                tmp_frame_infos.push_back(tmp_frame);
            }
        }
        for (int i = 0; i < frame_infos.size(); i++)
        {
            tmp_frame_infos.push_back(frame_infos[i]);
        }
        keyframe_manager_.update_other_frame(tmp_frame_infos);

        if (last_camera_index > -1)
            last_camera_index -= k;

        if (last_laser_index > -1)
            last_laser_index -= k;
        if (current_index > -1)
            current_index -= k;
        assert(current_index >= 0);

        auto remove_ids = feature_manger_.pop_frame(k);
    }
    void trajectory::do_tracking()
    {

        if (status != TRACKING)
            return;

        static double last_outpu_time = TIME_MIN;
        estimate_features(false);
        recorder.begin_record();
        opt_solver.solve(frame_infos, feature_manger_);
        recorder.end_record("solve");

        current_p = frame_infos.back()->p;
        current_q = frame_infos.back()->q;
        current_v = frame_infos.back()->v;
        current_bs = frame_infos.back()->bs;

        filter_outlier_world_point();
        recorder.begin_record();
        opt_solver.marginalization(frame_infos, feature_manger_);
        recorder.end_record("marginalization");
        pop_frame_for_tracking();

        // output
        Eigen::Isometry3d current_tf = lie::make_tf(current_p, current_q);
        Eigen::Isometry3d current_tf_base = current_tf * PARAM(T_imu_to_wheel);
        Eigen::Quaterniond q(current_tf_base.matrix().block<3, 3>(0, 0));

        double x = current_tf_base.matrix()(0, 3);
        double y = current_tf_base.matrix()(1, 3);
        double z = current_tf_base.matrix()(2, 3);
        if (last_outpu_time >= frame_infos.back()->time)
            ROS_ERROR("error output time %lf", last_outpu_time - frame_infos.back()->time);
        o_fstream << frame_infos.back()->time << " " << x << " " << y << " " << z << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
        last_outpu_time = frame_infos.back()->time;
    }
    
    void trajectory::estimate_features(const bool &remove)
    {
        // lmicroTimer("estimate_features");
        // std::cout << "feature infos" << std::endl;
        auto features = feature_manger_.get_feature_infos();
        std::vector<long long> wait_for_remove;
        for (auto &[id, ptr] : features)
        {
            if (ptr->estimate_initial_value())
            {
                // std::cout << "features id: " << ptr->id << " | [ " << ptr->world_point.transpose() << " ]" << std::endl;
                // std::cout << "error:" << ptr->error << std::endl;
                // std::cout << "\tob in frame : ";

                // for (int i : ptr->frame_indexs)
                //     std::cout << i << " ";
                // std::cout << std::endl;
            }
            else
                wait_for_remove.push_back(id);
        }
        if (remove)
            for (int i = 0; i < wait_for_remove.size(); i++)
            {
                auto remove_id = feature_manger_.remove_feature(wait_for_remove[i]);
                camera_manger_.abort_id(remove_id);
            }
    }
    void trajectory::pop_frame_for_tracking()
    {

        int n = frame_infos.size();
        int k = n - 1;
        if (PARAM(enable_laser))
            for (int i = n - 1; i > -1; i--)
            {
                if (frame_infos[i]->type == frame_info::laser)
                {
                    k = i;
                    break;
                }
            }
        if (PARAM(enable_camera))
            for (int i = n - 1; i > -1; i--)
            {
                if (frame_infos[i]->type == frame_info::camera)
                {
                    if (i < k)
                        k = i;
                    break;
                }
            }
        pop_frame(k);
        auto &laser_keyframes = laser_manger_.get_keyframs();
        while (laser_keyframes.size() > 1)
            laser_manger_.pop_scan();

        if (PARAM(enable_camera_vis))
        {
            visualization::get_ptr()->add_feature_map_to_show(feature_manger_.get_feature_map_ptr());
        }
    }

    void trajectory::show_lastest_image(const std::string &win_name)
    {
        if (!PARAM(enable_camera_vis))
            return;
        int lastest_camera_index = -1;
        for (int i = frame_infos.size() - 1; i > -1; i--)
        {
            if (frame_infos[i]->type == frame_info::camera && !frame_infos[i]->image.empty())
            {
                lastest_camera_index = i;
                break;
            }
        }
        if (lastest_camera_index == -1)
            return;

        auto frame_ptr = frame_infos[lastest_camera_index];
        cv::Mat mat_to_publish = frame_ptr->image.clone();
        auto lastest_features = feature_manger_.get_lastest_frame_features();
        for (int i = 0; i < lastest_features.size(); i++)
        {
            double s = -0.01;
            auto current_feature = lastest_features[i];
            assert(current_feature->frame_indexs.back() == lastest_camera_index);
            Eigen::Vector3d pts_in_pixel = PARAM(camera_K) * current_feature->camera_points.back();
            int ob_times = lastest_features[i]->ob_times;
            if (ob_times > 30)
                ob_times = 30;
            int blue = 255 - (double(ob_times) / 30) * 255;

            cv::Point draw_xy(pts_in_pixel(0), pts_in_pixel(1));
            cv::Scalar color(blue, 0, 255 - blue);

            if (!lastest_features[i]->has_estimate)
            {
                cv::circle(mat_to_publish, draw_xy, 4, color, -1);
                continue;
            }
            Eigen::Isometry3d tf = lie::make_tf(frame_ptr->p, frame_ptr->q) * PARAM(T_imu_to_camera);
            Eigen::Vector3d world_point = current_feature->world_point;
            Eigen::Vector3d local_point = tf.inverse() * world_point;
            // std::cout << "world point id:" << std::endl
            //           << lastest_features[i]->id << "|" << world_point.transpose() << std::endl;
            s = local_point(2);
            local_point = local_point / s;

            Eigen::Vector3d repoject_point_in_pixel = PARAM(camera_K) * local_point;
            cv::Scalar repoject_color(0, 255, 0);
            cv::Point repoject_draw_xy(repoject_point_in_pixel(0), repoject_point_in_pixel(1));

            cv::circle(mat_to_publish, repoject_draw_xy, 2, repoject_color, -1);

            std::string text = "(" + std::to_string(lastest_features[i]->id) + "," + std::to_string(int(s * 100)) + ")";
            cv::putText(mat_to_publish, text,
                        repoject_draw_xy,
                        cv::FONT_HERSHEY_SIMPLEX, 0.3, repoject_color, 1, 8);
            cv::circle(mat_to_publish, draw_xy, 4, color, -1);
        }
        cv::putText(mat_to_publish, win_name,
                    cv::Point(10, 10),
                    cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(0, 0, 255), 1, 8);
        visualization::get_ptr()->add_image_to_show(win_name, mat_to_publish);
    }

    void trajectory::filter_outlier_world_point()
    {
        if (!PARAM(enable_camera))
            return;
        auto &lastest_frame_world_point = feature_manger_.get_lastest_frame_features();
        int n = lastest_frame_world_point.size();
        std::vector<long long> wait_for_abort_id;
        for (int i = 0; i < n; i++)
        {
            if (!lastest_frame_world_point[i]->has_estimate)
                continue;
            if (!lastest_frame_world_point[i]->has_ready_for_opt)
                continue;
            auto world_point = lastest_frame_world_point[i]->world_point;
            int index = lastest_frame_world_point[i]->frame_indexs.back();
            Eigen::Isometry3d tmp_tf = lie::make_tf(frame_infos[index]->p, frame_infos[index]->q);
            Eigen::Isometry3d T_w_c = tmp_tf * PARAM(T_imu_to_camera);
            Eigen::Vector3d cam_p = T_w_c.inverse() * world_point;
            double z = cam_p(2);
            cam_p /= z;
            double error = (cam_p - lastest_frame_world_point[i]->camera_points.back()).norm();

            if (error > PARAM(max_camera_reproject_error) ||
                z > PARAM(max_camera_feature_dis) ||
                z < 0.1)
            {

                wait_for_abort_id.push_back(lastest_frame_world_point[i]->id);
            }
        }
        for (int i = 0; i < wait_for_abort_id.size(); i++)
        {
            // std::cout << "remove id:" << wait_for_abort_id[i] << std::endl;
            long long remove_id = feature_manger_.remove_feature(wait_for_abort_id[i]);
            camera_manger_.abort_id(remove_id);
        }
        auto remove_ids = feature_manger_.remove_all_features_without_lastest_frame();
        for (int i = 0; i < remove_ids.size(); i++)
        {
            camera_manger_.abort_id(remove_ids[i]);
        }
    }

}; // namespace lvio_2d