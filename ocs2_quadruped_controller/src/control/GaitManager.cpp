#include <utility>

#include "ocs2_quadruped_controller/control/GaitManager.h"

#include <ocs2_core/misc/LoadData.h>

namespace ocs2::legged_robot
{
    GaitManager::GaitManager(CtrlComponent &ctrl_component,
                             std::shared_ptr<GaitSchedule> gait_schedule_ptr)
        : ctrl_component_(ctrl_component),
          gait_schedule_ptr_(std::move(gait_schedule_ptr)),
          target_gait_({0.0, 1.0}, {STANCE})
    {
    }

    void GaitManager::preSolverRun(const scalar_t initTime, const scalar_t finalTime,
                                   const vector_t &currentState,
                                   const ReferenceManagerInterface &referenceManager)
    {
        getTargetGait();
        if (gait_updated_)
        {
            const auto timeHorizon = finalTime - initTime;
            gait_schedule_ptr_->insertModeSequenceTemplate(target_gait_, finalTime, timeHorizon);
            gait_updated_ = false;
        }
    }

    void GaitManager::init(const std::string &gait_file)
    {
        gait_name_list_.clear();
        loadData::loadStdVector(gait_file, "list", gait_name_list_, verbose_);

        gait_list_.clear();
        for (const auto &name : gait_name_list_)
        {
            gait_list_.push_back(loadModeSequenceTemplate(gait_file, name, verbose_));
        }

        RCLCPP_INFO(rclcpp::get_logger("gait_manager"), "GaitManager is ready.");
    }

    void GaitManager::getTargetGait()
    {
        const std::string &current_gait_name = ctrl_component_.user_cmds_.gait_name;

        if (current_gait_name == last_gait_name_)
            return;

        int gait_index = findGaitIndex(current_gait_name);
        if (gait_index == -1)
        {
            RCLCPP_INFO(rclcpp::get_logger("GaitManager"), "Unknown gait name: %s, automatically set to gait",
                        current_gait_name.c_str());
            return;
        }
        target_gait_ = gait_list_[gait_index];
        RCLCPP_INFO(rclcpp::get_logger("GaitManager"), "Switch to gait: %s",
                    gait_name_list_[gait_index].c_str());
        gait_updated_ = true;
        last_gait_name_ = current_gait_name;
    }

    int GaitManager::findGaitIndex(const std::string &gait_name) const
    {
        auto itr = std::find(gait_name_list_.begin(), gait_name_list_.end(), gait_name);

        if (itr != gait_name_list_.end())
            return std::distance(gait_name_list_.begin(), itr);
        else
            return -1;
    }
}
