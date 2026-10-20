/**
 * @file workspace_sampling_node.cpp
 * @brief constains the actual executable logic to get the workspace of a robotic system.
 * 
 * It loads the model, reads the parameters, invokes the analyzer and writes the results to a .CSV file.
 * 
 * @author Daan Krijnen
 * @date October 13, 2026
 */

#include <rclcpp/rclcpp.hpp>

#include <moveit/robot_model_loader/robot_model_loader.hpp>
#include <moveit/robot_model/robot_model.hpp>

#include <annin_ar4_teleoperator/workspace_sampling.hpp>

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>


int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    try
    {
        auto node = std::make_shared<rclcpp::Node>("workspace_sampling");

        const int samples = node->declare_parameter<int>("samples", 100000);
        
        const std::string planning_group = node->declare_parameter<std::string>("planning_group", "ar_manipulator");

        const std::string end_effector_link = node->declare_parameter<std::string>("end_effector_link", "ee_link");

        const std::string configured_output = node->declare_parameter<std::string>("output_csv", "~/.config/annin_ar4_teleoperator/workspace.csv");

        if (samples <= 0)
        {
            throw std::runtime_error("Parameter 'samples' must be positive.");
        }

        std::filesystem::path output_path(configured_output);

        if (configured_output == "~" || configured_output.rfind("~/", 0) == 0)
        {
            const char * home_dir = std::getenv("HOME");

            if (home_dir == nullptr)
            {
                throw std::runtime_error("Could not find the HOME directory.");
            }

            if (configured_output == "~")
            {
                output_path = home_dir;
            }
            else
            {
                output_path = std::filesystem::path(home_dir) / configured_output.substr(2);
            }
        }

        const auto output_directory = output_path.parent_path();

        if (!output_directory.empty())
        {
            std::filesystem::create_directories(output_directory);
        }

        
        RCLCPP_INFO(node->get_logger(),
            "Loading Moveit robot model...");

        robot_model_loader::RobotModelLoader model_loader(node, "robot_description");

        moveit::core::RobotModelPtr robot_model = model_loader.getModel();

        if (!robot_model)
        {
            throw std::runtime_error("Failed to load robot_description. "
                "Launch this node with the MoveIt robot model parameters.");
        }


        RCLCPP_INFO(node->get_logger(),
            "Planning group: %s",
            planning_group.c_str());

        RCLCPP_INFO(node->get_logger(),
            "End-effector link: %s",
            end_effector_link.c_str());

        RCLCPP_INFO(node->get_logger(),
            "Sampling %d joint configurations...",
            samples);

        annin_ar4_teleoperator::WorkspaceSampling analyzer(
            robot_model,
            planning_group,
            end_effector_link);

        std::vector<annin_ar4_teleoperator::WorkspacePoint> points;

        const auto bounds = analyzer.sample(static_cast<std::size_t>(samples), &points);


        std::ofstream csv(output_path);

        if (!csv.is_open())
        {
            throw std::runtime_error("Could not open CSV file: " + output_path.string());
        }

        const auto * joint_group = robot_model->getJointModelGroup(planning_group);

        if (joint_group == nullptr)
        {
            throw std::runtime_error("Planning group was not found in robot model.");
        }

        const auto & variable_names = joint_group->getVariableNames();

        csv << "x,y,z";

        for (const auto & name : variable_names)
        {
            csv << ",q" << name;
        }

        csv << '\n';

        for (const auto & point : points)
        {
            csv << point.position.x() << ","
                << point.position.y() << ","
                << point.position.z();

            for (const double joint_position : point.joint_positions)
            {
                csv << "," << joint_position;
            }

            csv << "\n";
        }

        csv.close();

        RCLCPP_INFO(node->get_logger(),
            "Workspace sampling complete");

        RCLCPP_INFO(node->get_logger(),
            "Smaples written: %zu",
            points.size());

        RCLCPP_INFO(node->get_logger(),
            "X bounds: %.4f m to %.4f m",
            bounds.minimum.x(),
            bounds.maximum.x());

        RCLCPP_INFO(node->get_logger(),
            "Y bounds: %.4f m to %.4f m",
            bounds.minimum.y(),
            bounds.maximum.y());

        RCLCPP_INFO(node->get_logger(),
            "Z bounds: %.4f m to %.4f m",
            bounds.minimum.z(),
            bounds.maximum.z());

        RCLCPP_INFO(node->get_logger(),
            "CSV saved to: %s",
            output_path.string().c_str());
    }
    catch(const std::exception& e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("workspace_sampling"),
            "Workspace sampling failed: %s",
            e.what());

        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
    
}