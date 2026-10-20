/**
 * @file workspace_sampling.hpp
 * @brief simple analysis structure of robotic workspace using moveit
 * 
 * @author Daan Krijnen
 * @date October 13, 2026
 */

#pragma once

#include <moveit/robot_model/robot_model.hpp>
#include <moveit/robot_model/joint_model_group.hpp>

#include <Eigen/Geometry>

#include <cstddef>
#include <string>
#include <vector>

namespace annin_ar4_teleoperator
{

    
    struct WorkspacePoint
    {
        Eigen::Vector3d position;               //Hypothetical end-effector position
        std::vector<double> joint_positions;    //Joint configuration to create that position
    };

    struct WorkspaceBounds
    {
        Eigen::Vector3d minimum;                // min sampled bound of the workspace (not a guarantee all points inside are reachable)
        Eigen::Vector3d maximum;                // max sampled bound of the workspace (not a guarantee...)
        std::size_t samples_used{0};            // samples used

        Eigen::Vector3d dimensions() const
        {
            return maximum - minimum;
        }
    };

    class WorkspaceSampling
    {
    public:

        WorkspaceSampling(
            const moveit::core::RobotModelPtr & robot_model,
            const std::string & planning_group,
            const std::string & end_effector_link);
        /*
        Samples the joint configurations and conputes the corresponding end-effector pose with forward kinematics.
        If points is not null, the individual samples are also returned.
        Only modifies in-memory robotic state.
        */

        WorkspaceBounds sample(
            std::size_t number_of_samples,
            std::vector<WorkspacePoint> * points = nullptr) const;

    private:
        moveit::core::RobotModelPtr robot_model_;

        const moveit::core::JointModelGroup * joint_group_{nullptr};
        const moveit::core::LinkModel * ee_link_{nullptr};

    };
}