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

  // I probably do not need an asynchronous queue at first, and passing the queue
  // in on the node should suffice.

  // auto asynchronous_callback_queue = std::make_shared<ros::CallbackQueue>();
  // auto async_spinner =
    // std::make_shared<ros::AsyncSpinner>(4, asynchronous_callback_queue.get());


  dist::RandomizedDistributedPlanner<Message> distributed_planner;

  if (!distributed_planner.initialize(n))
  {
    ROS_ERROR("%s: failed to initialize distributed distributed_planner",
        ros::this_node::getName().c_str());
    return EXIT_FAILURE;
  }

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

  // spinners are basically singletons
  //if(!async_spinner->canStart())
  //{
    //return EXIT_FAILURE;
  //}
  //async_spinner->start();

  distributed_planner.start();

  ros::spin();

  return EXIT_SUCCESS;
}
