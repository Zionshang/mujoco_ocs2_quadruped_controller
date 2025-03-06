#include <ocs2_quadruped_controller/model/QuadrupedIK.h>

namespace ocs2::legged_robot
{
    QuadrupedIK::QuadrupedIK()
    {
        pos_abd_rel_body_ << body_length_ / 2, body_length_ / 2, -body_length_ / 2, -body_length_ / 2,
            body_width_ / 2, -body_width_ / 2, body_width_ / 2, -body_width_ / 2,
            0, 0, 0, 0;
    }

    Vector3d QuadrupedIK::single_leg_ik(const Vector3d &p, bool is_left_leg)
    {
        Vector3d q;
        const double L = std::sqrt(std::pow(p(1), 2) + std::pow(p(2), 2));
        const double L1 = std::sqrt(std::pow(L, 2) - std::pow(abd_link_length_, 2));
        const double L2 = std::sqrt(std::pow(L1, 2) + std::pow(p(0), 2));

        if (is_left_leg)
        {
            q(0) = (p(1) > 0) ? std::acos(abd_link_length_ / L) + std::asin(p(2) / L)
                              : -(M_PI - (std::acos(abd_link_length_ / L) - std::asin(p(2) / L)));
        }
        else
        {
            q(0) = (p(1) < 0) ? -std::acos(abd_link_length_ / L) - std::asin(p(2) / L)
                              : M_PI - (std::acos(abd_link_length_ / L) - std::asin(p(2) / L));
        }

        q(1) = -(std::atan(p(0) / L1) - std::acos(L2 / (2 * upper_link_length_)));
        q(2) = -2 * std::acos(L2 / (2 * upper_link_length_));

        return q;
    }

    Matrix34d QuadrupedIK::IK(const Matrix34d &pos_foot_rel_body)
    {
        Matrix34d q;
        pos_foot_rel_abd_ = pos_foot_rel_body - pos_abd_rel_body_;
        q.col(0) = single_leg_ik(pos_foot_rel_abd_.col(0), true);
        q.col(1) = single_leg_ik(pos_foot_rel_abd_.col(1), false);
        q.col(2) = single_leg_ik(pos_foot_rel_abd_.col(2), true);
        q.col(3) = single_leg_ik(pos_foot_rel_abd_.col(3), false);
        return q;
    }

    void QuadrupedIK::loadSettings(const std::string &task_file, const bool verbose)
    {
        boost::property_tree::ptree pt;
        read_info(task_file, pt);
        const std::string prefix = "kinematics.";
        if (verbose)
        {
            std::cerr << "\n #### Model Kinematics:";
            std::cerr << "\n #### =============================================================================\n";
        }

        loadData::loadPtreeValue(pt, abd_link_length_, prefix + "abd_link_length", verbose);
        loadData::loadPtreeValue(pt, upper_link_length_, prefix + "upper_link_length", verbose);
        loadData::loadPtreeValue(pt, lower_link_length_, prefix + "lower_link_length", verbose);
        loadData::loadPtreeValue(pt, body_length_, prefix + "body_length", verbose);
        loadData::loadPtreeValue(pt, body_width_, prefix + "body_width", verbose);
    }
} // namespace ocs2::legged_robot