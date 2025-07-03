#include <memory>
#include <vector>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

// <-- include your service header from application_layer
#include "application_layer/srv/ompl_plan.hpp"

// OMPL headers
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;
using OMPLPlan = application_layer::srv::OMPLPlan;

class OMPLPlannerNode : public rclcpp::Node
{
public:
  OMPLPlannerNode()
  : Node("ompl_planner_node")
  {
    // Advertise the service on /ompl_plan
    service_ = this->create_service<OMPLPlan>(
      "ompl_plan",
      std::bind(&OMPLPlannerNode::planCallback, this,
                std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(this->get_logger(), "OMPLPlanner service ready on /ompl_plan");
  }

private:
  void planCallback(
    const std::shared_ptr<OMPLPlan::Request>  req,
    std::shared_ptr<OMPLPlan::Response>       res)
  {
    const auto &start_js = req->start;
    const auto &goal_js  = req->goal;
    double time_allowed = req->time_allowed;

    size_t dim = start_js.position.size();
    if (goal_js.position.size() != dim || start_js.name != goal_js.name) {
      RCLCPP_ERROR(get_logger(), "Start/goal joint mismatch");
      res->success = false;
      return;
    }

    // 1) Define OMPL state space
    auto space = std::make_shared<ob::RealVectorStateSpace>(dim);
    ob::RealVectorBounds bounds(dim);
    // TODO: pull actual limits from parameters or URDF
    for (size_t i = 0; i < dim; ++i) {
      bounds.setLow(i, -M_PI);
      bounds.setHigh(i, M_PI);
    }
    space->setBounds(bounds);

    // 2) Setup SimpleSetup
    og::SimpleSetup ss(space);
    ss.setPlanner(std::make_shared<og::RRTConnect>(ss.getSpaceInformation()));

    // 3) Set start & goal states
    ob::ScopedState<> start(space), goal(space);
    for (size_t i = 0; i < dim; ++i) {
      start[i] = start_js.position[i];
      goal[i]  = goal_js.position[i];
    }
    ss.setStartAndGoalStates(start, goal);

    // 4) Solve within the allowed time
    ss.setup();
    ob::PlannerStatus solved = ss.solve(time_allowed);
    if (!solved) {
      RCLCPP_ERROR(get_logger(), "OMPL planning failed");
      res->success = false;
      return;
    }

    // 5) Extract the path and convert to JointTrajectory
    auto path = ss.getSolutionPath()->as<og::PathGeometric>();
    auto &traj = res->trajectory;
    traj.joint_names = start_js.name;
    traj.header.stamp = this->get_clock()->now();

    // Distribute timing along the path by arc‐length
    double total_length = path->length();
    double scale = (total_length > 0.0) ? (time_allowed / total_length) : 0.0;
    double t = 0.0;

    for (size_t i = 0; i < path->getStateCount(); ++i) {
      const auto *state = path->getState(i)->as<ob::RealVectorStateSpace::StateType>();
      trajectory_msgs::msg::JointTrajectoryPoint pt;
      pt.positions.resize(dim);
      for (size_t j = 0; j < dim; ++j) {
        pt.positions[j] = (*state)[j];
      }
      if (i + 1 < path->getStateCount()) {
        double dist = ss.getSpaceInformation()->distance(
          path->getState(i), path->getState(i + 1));
        t += dist * scale;
      }
      pt.time_from_start = rclcpp::Duration::from_seconds(t).to_msg();
      traj.points.push_back(pt);
    }

    res->success = true;
    RCLCPP_INFO(get_logger(), "OMPL plan succeeded (%zu points)", traj.points.size());
  }

  rclcpp::Service<OMPLPlan>::SharedPtr service_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OMPLPlannerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
