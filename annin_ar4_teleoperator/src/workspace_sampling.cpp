/**
 * @file workspace_sampling.cpp
 * @brief constaints the logic for analyzing the workspace of the robotic arm. Does not check for collisions etc.
 * 
 * @author Daan Krijnen
 * @date October 13, 2026
 */

#include <annin_ar4_teleoperator/workspace_sampling.hpp>

#include <moveit/robot_state/robot_state.hpp>

#include <algorithm>
#include <limits>
#include <stdexcept>
#include <utility>

namespace annin_ar4_teleoperator
{
    WorkspaceSampling::WorkspaceSampling(
        const moveit::core::RobotModelPtr & robot_model,
        const std::string & planning_group,
        const std::string & end_effector_link)
        : robot_model_(robot_model)
    {
        if (!robot_model_)
        {
            throw std::invalid_argument("WorkspaceSampling received a null robot model.");
        }

        joint_group_ = robot_model_->getJointModelGroup(planning_group);

        if (!joint_group_)
        {
            throw std::invalid_argument("Planning group not found: " + planning_group);
        }

        ee_link_ = robot_model_->getLinkModel(end_effector_link);

        if (!ee_link_)
        {
            throw std::invalid_argument("End-effector link not found: " + end_effector_link);
        }
    }

    WorkspaceBounds WorkspaceSampling::sample(
        const std::size_t number_of_samples,
        std::vector<WorkspacePoint> * points) const
    {
        if (number_of_samples == 0)
        {
            throw std::invalid_argument("number_of_samples must be greater than zero.");
        }

        const double inf = std::numeric_limits<double>::infinity();

        WorkspaceBounds bounds;
        bounds.minimum = Eigen::Vector3d::Constant(inf);
        bounds.maximum = Eigen::Vector3d::Constant(-inf);
        bounds.samples_used = 0;

        if (points != nullptr)
        {
            points->clear();
            points->reserve(number_of_samples);
        }

        moveit::core::RobotState state(robot_model_);
        state.setToDefaultValues();

        const auto & variable_names = joint_group_->getVariableNames();

        for (std::size_t i = 0; i < number_of_samples; ++i)
        {
            state.setToRandomPositions(joint_group_);
            state.update();

            const Eigen::Isometry3d & transform = state.getGlobalLinkTransform(ee_link_);

            const Eigen::Vector3d position = transform.translation();

            if (!position.allFinite())
            {
                continue;
            }

            bounds.minimum = bounds.minimum.cwiseMin(position);
            bounds.maximum = bounds.maximum.cwiseMax(position);
            ++bounds.samples_used;

            if (points != nullptr)
            {
                WorkspacePoint point;
                point.position = position;
                point.joint_positions.reserve(variable_names.size());

                for (const auto & name : variable_names)
                {
                    point.joint_positions.push_back(state.getVariablePosition(name));
                }

                points->push_back(std::move(point));
            }
        }

        if (bounds.samples_used == 0)
        {
            throw std::runtime_error("Workspace sampling produced no valid positions");
        }

        return bounds;
    }
}