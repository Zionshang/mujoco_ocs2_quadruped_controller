#pragma once
#include <ocs2_legged_robot/common/Types.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>
#include <ocs2_quadruped_controller/estimator/LinearKalmanFilter.h>

namespace ocs2::legged_robot
{
    class TerrainEstimator
    {
    public:
        TerrainEstimator();

        void update(const vector3_t &body_eular_angle,
                    const std::vector<vector3_t> &pos_feet2body,
                    const contact_flag_t &contact_flag);
        // expressed in BODY frame
        vector3_t getGroundEulerAngleWrtBody() const { return ground_euler_angle; }

    private:
        matrix3_t rotmat_body;
        std::vector<vector3_t> pos_feet2body_body = std::vector<vector3_t>(4); // position of foot w.r.t body, expressed in BODY frame
        vector3_t ground_euler_angle;                                          // expressed in BODY frame
    };
} // namespace ocs2::legged_robot
