#include <ros/ros.h>
#include <ros/callback_queue.h>

#include "distributed_randomized_sequential_partitions/DistributedPlanner.h"

// Message
#include <distributed_randomized_sequential_partitions/SimpleDistributedPlannerMsg.h>

namespace dist = distributed_randomized_sequential_partitions;

using Message = dist::SimpleDistributedPlannerMsg;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "distributed_planner");
  ros::NodeHandle n;

  dist::RandomizedDistributedPlanner<Message> distributed_planner;

  if (!distributed_planner.initialize(n))
  {
    ROS_ERROR("%s: failed to initialize distributed distributed_planner",
        ros::this_node::getName().c_str());
    return EXIT_FAILURE;
  }

  // A minimal callback
  // * This is where your anytime planner goes in a fully realized distributed
  //   planner
  auto planner_callback = [](double planning_time,
      const std::vector<Message>& prior_decisions, Message&)
  {
    auto end_time = ros::Time::now() + ros::Duration(planning_time);

    ROS_INFO_STREAM("Planning callback, duration: " << planning_time);
    ROS_INFO_STREAM("Received: " << prior_decisions.size() << " messages");

    // Sleep until the end of the planning round
    (end_time - ros::Time::now()).sleep();
  };

  distributed_planner.setPlannerCallback(planner_callback);

  distributed_planner.start();

  ros::spin();

  return EXIT_SUCCESS;
}
