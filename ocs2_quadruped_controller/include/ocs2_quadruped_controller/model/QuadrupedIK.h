#pragma once
#include <Eigen/Dense>
#include <cmath>
#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <ocs2_core/misc/LoadData.h>

namespace ocs2::legged_robot
{
    using Eigen::Vector3d;
    using Matrix34d = Eigen::Matrix<double, 3, 4>;

    class QuadrupedIK
    {
    public:
        QuadrupedIK();
        Vector3d single_leg_ik(const Vector3d &pos_foot_rel_abd, bool is_left_leg);
        Matrix34d IK(const Matrix34d &pos_foot_rel_body);
        void loadSettings(const std::string &task_file, bool verbose);

    private:
        double abd_link_length_ = 0.1065; // hip关节偏移
        double upper_link_length_ = 0.26; // 大腿长度
        double lower_link_length_ = 0.26; // 小腿长度
        double body_length_ = 0.603;      // 身体长度
        double body_width_ = 0.16;        // 身体宽度

        Matrix34d pos_abd_rel_body_;
        Matrix34d pos_foot_rel_abd_;
    };
} // namespace ocs2::legged_robot