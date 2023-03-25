#pragma once
#include <Eigen/Dense>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <limits>
#include <memory>
#include <sensor_msgs/LaserScan.h>
#include <vector>

static inline constexpr double TIME_MIN = std::numeric_limits<double>::min();
static inline constexpr double TIME_MAX = std::numeric_limits<double>::max();
namespace convert
{
    std::tuple<std::shared_ptr<std::vector<Eigen::Vector3d>>, std::shared_ptr<std::vector<double>>>
    laser_to_point_times(const sensor_msgs::LaserScan::ConstPtr &msg);
    template <typename T>
    Eigen::Matrix<T, 3, 3> cross_matrix(const Eigen::Matrix<T, 3, 1> &vec)
    {
        Eigen::Matrix<T, 3, 3> ret = Eigen::Matrix<T, 3, 3>::Zero();
        ret(0, 1) = -vec(2);
        ret(1, 0) = vec(2);

        ret(0, 2) = vec(1);
        ret(2, 0) = -vec(1);

        ret(1, 2) = -vec(0);
        ret(2, 1) = vec(0);
        return ret;
    }

    template <typename T, size_t N>
    Eigen::Matrix<T, N, N> disdiagonal(const Eigen::Matrix<T, N, 1> &vec)
    {
        Eigen::Matrix<T, N, N> ret = Eigen::Matrix<T, N, N>::Zero();
        for (int i = 0; i < N; i++)
            ret(i, i) = vec(i);
        return ret;
    }

    template <typename T, size_t N>
    Eigen::Matrix<T, N, N> disdiagonal2(const Eigen::Matrix<T, N, 1> &vec)
    {
        Eigen::Matrix<T, N, N> ret = Eigen::Matrix<T, N, N>::Zero();
        for (int i = 0; i < N; i++)
            ret(i, i) = vec(i) * vec(i);
        return ret;
    }
    inline double angle_to_rad(const double &angle) { return angle / 180.0 * M_PI; }
    inline double rad_to_angle(const double &rad) { return rad / M_PI * 180.0; }

    template <typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
    void remove_rows(Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols> &mat, const int &begin_index,
                     const int &len)
    {
        unsigned int n_rows = mat.rows() - len;
        unsigned int n_cols = mat.cols();
        assert(len >= 0);
        assert(begin_index + len <= mat.rows());
        if (begin_index < n_rows)
        {
            mat.block(begin_index, 0, n_rows - begin_index, n_cols) =
                mat.block(begin_index + len, 0, n_rows - begin_index, n_cols);
        }
        mat.conservativeResize(n_rows, n_cols);
    }

    template <typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
    void remove_cols(Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols> &mat, const int &begin_index,
                     const int &len)
    {
        unsigned int n_rows = mat.rows();
        unsigned int n_cols = mat.cols() - len;
        assert(len >= 0);
        assert(begin_index + len <= mat.cols());
        if (begin_index < n_cols)
        {
            mat.block(0, begin_index, n_rows, n_cols - begin_index) =
                mat.block(0, begin_index + len, n_rows, n_cols - begin_index);
        }
        mat.conservativeResize(n_rows, n_cols);
    }

} // namespace convert
namespace e_laser
{
    template <typename T>
    T dis_from_line(const Eigen::Matrix<T, 3, 1> &p,
                    const Eigen::Matrix<T, 3, 1> &p1,
                    const Eigen::Matrix<T, 3, 1> &p2)
    {

        Eigen::Matrix<T, 3, 1> line = (p2 - p1).normalized();
        Eigen::Matrix<T, 3, 1> p2p = p - p2;
        return (p2p - (line.normalized().dot(p2p)) * line).norm();
    }
    Eigen::Isometry3d ICP_solve(const std::vector<Eigen::Vector3d> &p1,const std::vector<Eigen::Vector3d> &p2);
    
} // namespace e_laser
namespace e_cv
{
    double triangulate(const Eigen::Vector3d &cam_point1, const Eigen::Vector3d &cam_point2, const Eigen::Isometry3d &tf_1_2);
    std::tuple<double, double> triangulate_SVD(Eigen::Vector3d cam_point1, Eigen::Vector3d cam_point2, Eigen::Isometry3d &tf_1_2);

    std::tuple<Eigen::Vector3d, double> triangulate_points_SVD(const std::vector<Eigen::Vector3d> &cam_points, const std::vector<Eigen::Isometry3d> &tfs);

    template <typename T>
    T reproject_error(const Eigen::Matrix<T, 3, 1> &cam_point1,
                      const Eigen::Matrix<T, 3, 1> &cam_point2,
                      const T &s1,
                      const T &s2,
                      const Eigen::Transform<T, 3, Eigen::Isometry> &tf_12)
    {
        Eigen::Matrix<T, 3, 1> tmp2 = tf_12 * (s2 * cam_point2);
        tmp2 = tmp2 / tmp2(2);
        return (tmp2 - cam_point1).norm();
    }
} // namespace e_cv

namespace lie
{
    template <typename T>
    void normalize_so3(Eigen::Matrix<T, 3, 1> &so3)
    {
        // Use ceres::floor because it is specialized for double and Jet types.
        T angle = so3.norm();
        T normalize_angle = angle;
        T two_pi(2.0 * M_PI);
        T pi(M_PI);
        if (angle > pi)
            normalize_angle -= two_pi * ceres::floor((angle + pi) / two_pi);
        else
            return;
        so3 /= angle;
        so3 *= normalize_angle;
    }

    template <typename T>
    Eigen::Matrix<T, 3, 3> exp_so3(const Eigen::Matrix<T, 3, 1> &so3)
    {
        const Eigen::Matrix<T, 3, 1> &axis = so3;
        T angleAxis_[3] = {axis.data()[0], axis.data()[1], axis.data()[2]};
        T q_[4];
        ceres::AngleAxisToQuaternion(angleAxis_, q_);
        Eigen::Quaternion<T> q(q_[0], q_[1], q_[2], q_[3]);
        return q.toRotationMatrix();
    }

    template <typename T>
    Eigen::Matrix<T, 3, 1> log_SO3(const Eigen::Matrix<T, 3, 3> &SO3)
    {
        Eigen::Matrix<T, 3, 1> ret;
        Eigen::Quaternion<T> q(SO3);
        q.normalize();
        T angleAxis_[3];
        T q_[4]{
            q.w(), q.x(), q.y(), q.z()};
        ceres::QuaternionToAngleAxis(q_, angleAxis_);
        ret(0, 0) = angleAxis_[0];
        ret(1, 0) = angleAxis_[1];
        ret(2, 0) = angleAxis_[2];
        normalize_so3<T>(ret);
        return ret;
    }

    template <typename T>
    std::tuple<Eigen::Matrix<T, 3, 1>, Eigen::Matrix<T, 3, 1>> log_SE3(const Eigen::Transform<T, 3, Eigen::Isometry> &SE3)
    {
        Eigen::Matrix<T, 3, 1> p = SE3.matrix().template block<3, 1>(0, 3);
        Eigen::Matrix<T, 3, 1> so3 = log_SO3<T>(SE3.matrix().template block<3, 3>(0, 0));
        return {p, so3};
    }

    template <typename T>
    Eigen::Transform<T, 3, Eigen::Isometry> make_tf(const Eigen::Matrix<T, 3, 1> &p,
                                                    const Eigen::Matrix<T, 3, 1> &so3)
    {
        Eigen::Transform<T, 3, Eigen::Isometry> tf = Eigen::Transform<T, 3, Eigen::Isometry>::Identity();
        tf.matrix().template block<3, 3>(0, 0) = exp_so3(so3);
        tf.matrix().template block<3, 1>(0, 3) = p;
        return tf;
    }

    template <typename T>
    void normalize_tf(Eigen::Transform<T, 3, Eigen::Isometry> &SE3)
    {
        SE3.matrix().template block<3, 3>(0, 0) = Eigen::Quaternion<T>(
                                                      SE3.matrix().template block<3, 3>(0, 0))
                                                      .toRotationMatrix();
    }

    template <typename T>
    Eigen::Matrix<T, 3, 1> get_p_from_tf(const Eigen::Transform<T, 3, Eigen::Isometry> &tf)
    {
        return tf.matrix().template block<3, 1>(0, 3);
    }
} // namespace lie

namespace auto_diff
{
    using rMatrix = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
    template <typename FACTOR, size_t n_r, size_t... n_xs>
    void compute_res_and_jacobi(FACTOR *f, std::vector<double *> &parameters,
                                Eigen::Matrix<double, n_r, 1> &res,
                                std::vector<rMatrix> &jacobians)
    {
        auto cost_function = new ceres::AutoDiffCostFunction<FACTOR, n_r, n_xs...>(f);
        std::vector<size_t> n_block = {n_xs...};
        std::vector<double *> jacobians_double;
        jacobians.clear();
        for (int i = 0; i < n_block.size(); i++)
        {
            jacobians.push_back(rMatrix::Zero(n_r, n_block[i]));
            jacobians_double.push_back(jacobians.back().data());
        }
        cost_function->Evaluate(parameters.data(), res.data(), jacobians_double.data());
        delete cost_function;
    }

    // new and no delete by yourself
    template <typename FACTOR>
    void compute_res_and_jacobi_dynamic(FACTOR *f, std::vector<double *> &parmaters, const std::vector<int> &n_block,
                                        Eigen::VectorXd &res, const int &n_r, std::vector<rMatrix> &jacobians)
    {

        auto cost_function = new ceres::DynamicAutoDiffCostFunction<FACTOR>(f);
        for (int i = 0; i < n_block.size(); i++)
            cost_function->AddParameterBlock(n_block[i]);
        cost_function->SetNumResiduals(n_r);
        jacobians.clear();
        std::vector<double *> jacobians_double;
        for (int i = 0; i < n_block.size(); i++)
        {
            jacobians.push_back(rMatrix(n_r, n_block[i]));
            jacobians_double.push_back(jacobians.back().data());
        }
        cost_function->Evaluate(parmaters.data(), res.data(), jacobians_double.data());
        delete cost_function;
    }
} // namespace auto_diff
namespace BUG
{
    template <typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
    bool maxtirx_is_valid(const Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols> &mat)
    {
        for (int r = 0; r < mat.rows(); r++)
            for (int c = 0; c < mat.cols(); c++)
                if (std::isnan(mat(r, c)) || std::isinf(mat(r, c)))
                    return false;
        return true;
    }
    template <typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
    bool maxtirx_is_zero(const Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols> &mat)
    {
        for (int r = 0; r < mat.rows(); r++)
            for (int c = 0; c < mat.cols(); c++)
                if (mat(r, c) != 0)
                    return false;
        return true;
    }
} // namespace BUG