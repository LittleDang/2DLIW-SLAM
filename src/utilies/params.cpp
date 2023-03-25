#include "params.h"
#include "utilies/common.h"
#define ECHO_PARAM(name) GREEN_INFO(true, #name << ":\t" << name << std::endl)
#define LOAD_PARAM(nh, name)      \
    do                            \
    {                             \
        nh.getParam(#name, name); \
        ECHO_PARAM(name);         \
    } while (0)

#define ECHO_TF(name) GREEN_INFO(true, #name << ":\n" \
                                             << name.matrix() << std::endl)
#define LOAD_TF(nh, name)                 \
    do                                    \
    {                                     \
        name = load_transform(#name, nh); \
        ECHO_TF(name);                    \
    } while (0)

#define ECHO_VEC(name) GREEN_INFO(true, #name << ":\t" \
                                              << name.transpose() << std::endl)
#define LOAD_VEC(nh, name, N)              \
    do                                     \
    {                                      \
        name = load_vectorX<N>(#name, nh); \
        ECHO_VEC(name);                    \
    } while (0)

#define ECHO_MAT(name) GREEN_INFO(true, #name << ":\n" \
                                              << name << std::endl)
#define LOAD_MAT(nh, name, N)              \
    do                                     \
    {                                      \
        name = load_matrixX<N>(#name, nh); \
        ECHO_MAT(name);                    \
    } while (0)

#define LOAD_MAT3(nh, name) LOAD_MAT(nh, name, 3)
#define LOAD_MAT2(nh, name) LOAD_MAT(nh, name, 2)

#define LOAD_VEC3(nh, name) LOAD_VEC(nh, name, 3)
#define LOAD_VEC2(nh, name) LOAD_VEC(nh, name, 2)

Eigen::Isometry3d load_transform(const std::string &param_name, const ros::NodeHandle &nh)
{
    XmlRpc::XmlRpcValue param_list;
    Eigen::Isometry3d ret;
    nh.getParam(param_name, param_list);
    for (size_t r = 0; r < 4; r++)
        for (size_t c = 0; c < 4; c++)
            ret.matrix().data()[c * 4 + r] = param_list[r * 4 + c];
    lie::normalize_tf(ret);
    return ret;
}
template <size_t N>
Eigen::Matrix<double, N, 1> load_vectorX(const std::string &param_name, const ros::NodeHandle &nh)
{
    XmlRpc::XmlRpcValue param_list;
    Eigen::Matrix<double, N, 1> ret;
    nh.getParam(param_name, param_list);
    for (size_t i = 0; i < N; i++)
        ret(i) = param_list[i];
    return ret;
}

template <size_t N>
Eigen::Matrix<double, N, N> load_matrixX(const std::string &param_name, const ros::NodeHandle &nh)
{
    XmlRpc::XmlRpcValue param_list;
    Eigen::Matrix<double, N, N> ret;
    nh.getParam(param_name, param_list);
    for (size_t r = 0; r < N; r++)
        for (size_t c = 0; c < N; c++)
            ret(r, c) = param_list[r * N + c];
    return ret;
}

namespace lvio_2d
{
    namespace param
    {
        manager::ptr manager::get_param_manager()
        {
            static manager::ptr ptr = nullptr;
            if (!ptr)
            {
                static ros::NodeHandle nh("~");
                ptr = manager::ptr(new manager(nh));
            }
            return ptr;
        }
        manager::manager(const ros::NodeHandle &nh)
        {
            LOAD_PARAM(nh, wheel_odom_topic);
            LOAD_PARAM(nh, laser_topic);
            LOAD_PARAM(nh, imu_topic);
            LOAD_PARAM(nh, camera_topic);
            LOAD_PARAM(nh, g);

            LOAD_PARAM(nh, manifold_p_sigma);
            LOAD_PARAM(nh, manifold_q_sigma);

            LOAD_TF(nh, T_imu_to_camera);
            LOAD_TF(nh, T_imu_to_laser);
            LOAD_TF(nh, T_imu_to_wheel);

            LOAD_VEC3(nh, imu_noise_acc_sigma);
            LOAD_VEC3(nh, imu_bias_acc_sigma);
            LOAD_VEC3(nh, imu_noise_gyro_sigma);
            LOAD_VEC3(nh, imu_bias_gyro_sigma);
            LOAD_VEC3(nh, wheel_sigma);

            LOAD_VEC2(nh, camera_sigma);

            LOAD_PARAM(nh, max_feature_num);

            LOAD_PARAM(nh, slide_window_size);
            LOAD_PARAM(nh, p_motion_threshold);
            LOAD_PARAM(nh, q_motion_threshold);

            LOAD_PARAM(nh, w_laser_each_scan);
            LOAD_PARAM(nh, h_laser_each_scan);
            LOAD_PARAM(nh, laser_resolution);

            LOAD_PARAM(nh, line_continuous_threshold);
            LOAD_PARAM(nh, line_max_tolerance_angle);
            LOAD_PARAM(nh, line_min_len);
            LOAD_PARAM(nh, line_max_dis);
            LOAD_PARAM(nh, line_to_line_sigma);

            LOAD_PARAM(nh, enable_camera);
            LOAD_PARAM(nh, enable_laser);
            LOAD_PARAM(nh, enable_camera_vis);
            LOAD_PARAM(nh, enable_laser_vis);

            LOAD_PARAM(nh, max_camera_reproject_error);
            LOAD_PARAM(nh, max_camera_feature_dis);

            LOAD_MAT3(nh, camera_K);

            LOAD_PARAM(nh, feature_min_dis);

            LOAD_PARAM(nh, FPS);

            LOAD_PARAM(nh, key_frame_p_motion_threshold);
            LOAD_PARAM(nh, key_frame_q_motion_threshold);

            LOAD_PARAM(nh, a_res);
            LOAD_PARAM(nh, d_res);
            LOAD_PARAM(nh, submap_count);
            LOAD_PARAM(nh, laser_loop_min_match_threshold);
            LOAD_PARAM(nh, loop_detect_min_interval);
            LOAD_PARAM(nh, output_dir);
            LOAD_PARAM(nh, output_tum);
            LOAD_PARAM(nh, use_ground_p_factor);
            LOAD_PARAM(nh, use_ground_q_factor);
            LOAD_PARAM(nh, verify_loop_rate);
            LOAD_PARAM(nh, loop_max_dis);

            LOAD_PARAM(nh, loop_edge_k);
            LOAD_PARAM(nh, ref_motion_filter_p);
            LOAD_PARAM(nh, ref_motion_filter_q);
            LOAD_PARAM(nh, ref_n_accumulation);

            LOAD_PARAM(nh, loop_max_tf_p);
            LOAD_PARAM(nh, loop_max_tf_q);

            LOAD_VEC3(nh, loop_sigma_p);
            LOAD_VEC3(nh, loop_sigma_q);

            LOAD_PARAM(nh, fast_mode);

            if (!check_param())
                exit(-1);
        }
        bool manager::check_param()
        {
            if (!enable_laser)
                enable_laser_vis = false;
            if (!enable_camera)
                enable_camera_vis = false;
            if (!enable_camera && !enable_laser)
            {
                RED_INFO(true, "ERROR: disable camera and disable laser");
                return false;
            }
            max_camera_reproject_error = max_camera_reproject_error / camera_K(0, 0);
            min_delta_t = 1.0 / double(FPS);
            return true;
        }

    } // namespace param
} // namespace lvio_2d