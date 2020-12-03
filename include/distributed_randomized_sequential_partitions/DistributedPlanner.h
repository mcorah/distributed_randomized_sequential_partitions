#ifndef DISTRIBUTEDPLANNER_H
#define DISTRIBUTEDPLANNER_H

// This file implements anytime distributed planning/scheduling for multi-robot
// teams.
//
// In effect, this amounts to scheduling planning planning times for each robot
// and communicating decisions. In fact, this implementation is independent of
// the planner, planning process and results aside from the message that
// communicates this data.
//
// Terminology:
// * Epoch: The time period during which the planning processes collectively
//   solve a single joint planning problem
//   (e.g. one instance of submodular maximization)
// * Planning round (or round): One of possibly several scheduled time period
//   during which individual robots plan during an epoch e.g. an epoch may
//   consist of three rounds

#include <thread>
#include <functional>
#include <cmath>
#include <random>
#include <mutex>

#include <ros/ros.h>

// Message (signature)
#include <distributed_randomized_sequential_partitions/DistributedPlannerSignature.h>
// Statistics
#include <distributed_randomized_sequential_partitions/DistributedPlannerStatistics.h>

namespace distributed_randomized_sequential_partitions
{
  // A simple parameter loader
  template<class T>
  static bool loadParam(const ros::NodeHandle& n,
      const std::string& query, T& value)
  {
    ros::NodeHandle node(n);

    std::string param_path;
    if(!ros::param::search(query, param_path))
    {
      ROS_WARN_STREAM("Failed to search for param: " << query);
      return false;
    }

    node.getParam(param_path, value);

    return true;
  }

  template<class Message>
  class AbstractDistributedPlanner
  {
    public:

      using Ptr = std::shared_ptr<AbstractDistributedPlanner<Message>>;
      using ConstPtr =
          std::shared_ptr<const AbstractDistributedPlanner<Message>>;

      // Args
      // (planning_time, prior_decisions, current_decision)
      using PlannerCallback =
          std::function<void (double, const std::vector<Message>&, Message&)>;

      virtual ~AbstractDistributedPlanner()
      {
        stop();
      }

      virtual bool initialize(const ros::NodeHandle& n)
      {
        ros::NodeHandle node(n);

        if (!loadParam(node, "distributed_planning/epoch_duration",
                       epoch_duration))
          return false;

        if (!loadParam(node, "planner_id", reinterpret_cast<int&>(planner_id)))
          return false;

        return true;
      }

      // start and stop the planner thread
      void start()
      {
        // Check that the planner is not running, and start the thread
        if(!planner_thread.joinable())
        {
          continue_planning = true;
          planner_thread = std::thread([this](){ plannerThread(); });
        }
      }
      void stop()
      {
        if(planner_thread.joinable())
        {
          // Signal the planner to stop and join
          continue_planning = false;
          planner_thread.join();
        }
      }

      double epochDuration() const
      {
        return epoch_duration;
      }

      void plannerThread()
      {
        ROS_INFO("Running planner_thread");
        auto now = ros::Time::now();

        // Sleep until the start of the first epoch
        sleepToEndOfEpoch(now);
        unsigned epoch = getNextEpoch(now);

        while(continue_planning)
        {
          runPlanningEpoch(epoch);

          // sleep till the end of the epoch if necessary
          auto now = ros::Time::now();
          unsigned next_epoch = getNextEpoch(now);
          if(next_epoch != epoch + 1)
          {
            ROS_WARN_STREAM("Planner " << getId()
                                       << ": Planning exceeded available time, "
                                          "skipping the next epoch");
          }

          epoch = next_epoch;
          sleepToEndOfEpoch(now);
        }
      }

      // Plan for the current epoch, depends on the specific planner
      // implementation
      // Note: I may want to break this into components that can be executed
      // sequentially and in parallel.
      virtual void runPlanningEpoch(unsigned planning_epoch)
      {
        ROS_INFO_STREAM("Planner "
                        << getId()
                        << ": Running abstract planning epoch ("
                        << planning_epoch
                        << ") (which does nothing but print this string)");

        Message message;
        signMessage(planning_epoch, message);
      }

      // Returns the (should be unique) id of the given planner
      unsigned getId() const
      {
        return planner_id;
      }

      // Form of the planner callback:
      // Input:
      // * Available planning time
      // * A vector of messages
      //   (typically describing prior decisions in a planning sequence)
      // Output: A Message describing the decision
      // (The distributed planner will be responsible for filling in the
      // signature fields before sending the message)
      //
      // Note: See "plannerCallback" for access
      void setPlannerCallback(PlannerCallback planner_callback)
      {
        this->planner_callback = planner_callback;
      }

    protected:
      // Compute the number of the current epoch.
      // (epoch zero starts at time zero)
      unsigned getEpoch(ros::Time time) const
      {
        // round down
        return unsigned(time.toSec() / epoch_duration);
      }
      unsigned getNextEpoch(ros::Time time) const
      {
        return getEpoch(time) + 1;
      }
      // How far are we into the current epoch
      double timeInEpoch(ros::Time time) const
      {
        return fmod(time.toSec(), epoch_duration);
      }
      double remainingTime(ros::Time time) const
      {
        return epoch_duration - timeInEpoch(time);
      }

      void sleepToEndOfEpoch(ros::Time time) const
      {
        ros::Duration(remainingTime(time)).sleep();
      }

      // Sign the signature field of the message
      void signMessage(unsigned planning_epoch, Message& message)
      {
        // Ensure that the message has a signature field which fits the
        // prescribed format
        DistributedPlannerSignature& signature = message.signature;

        signature.planning_epoch = planning_epoch;
        signature.planner_id = getId();
      }

      // wrap the callback
      void plannerCallback(double planning_duration,
                           const std::vector<Message>& prior_decisions,
                           Message& current_decision) const
      {
        planner_callback(planning_duration, prior_decisions, current_decision);
      }

    private:
      double epoch_duration;

      unsigned planner_id;

      bool continue_planning = false;
      std::thread planner_thread;

      PlannerCallback planner_callback;
  };

  template<class Message>
  class MyopicDistributedPlanner : public AbstractDistributedPlanner<Message>
  {
    public:
      virtual void runPlanningEpoch(unsigned planning_epoch)
      {
        ROS_INFO_STREAM("Planner "
            << this->getId()
            << ": Running myopic planning epoch ("
            << planning_epoch
            << ")");

        Message message;
        std::vector<Message> empty;

        this->plannerCallback(this->epochDuration(), empty, message);

        this->signMessage(planning_epoch, message);
      }
  };

  template<class Message>
  class RandomizedDistributedPlanner : public AbstractDistributedPlanner<Message>
  {
    public:

      virtual ~RandomizedDistributedPlanner()
      {
        printStatistics();

        decision_sub.shutdown();
      }

      virtual bool initialize(const ros::NodeHandle& n)
      {
        if (!AbstractDistributedPlanner<Message>::initialize(n))
          return false;

        ros::NodeHandle node(n);

        std::random_device random_device;
        rng.seed(random_device());

        if (!loadParam(node, "distributed_planning/num_rounds",
                       reinterpret_cast<int&>(this->num_rounds)))
          return false;

        ROS_INFO_STREAM("Planner " << this->getId()
                                   << ": num_rounds = " << num_rounds);

        if (!loadParam(node, "distributed_planning/message_latency",
                       message_latency))
          return false;

        std::string planner_topic;
        if (!loadParam(node, "distributed_planning/planner_topic",
                       planner_topic))
          return false;

        decision_pub = node.advertise<Message>(planner_topic, 100);

        statistics_pub = node.advertise<DistributedPlannerStatistics>(
            "distributed_planner_statistics", 1);

        // Hopefully, we should not have to buffer here.
        // However, if we do buffer, we want to process new messages
        decision_sub = node.subscribe(
            planner_topic, 100,
            &RandomizedDistributedPlanner<Message>::bufferDecision,
            this, ros::TransportHints().tcpNoDelay());

        return true;
      }

      virtual void runPlanningEpoch(unsigned planning_epoch)
      {
        //
        // Schedule planning round
        //

        // Schedule in [0, num_rounds-1]
        unsigned planning_round = samplePlanningRound();

        // Planning within the epoch
        double round_duration = getRoundDuration();

        // Sleep till the start of planning time
        ros::Duration(round_duration * planning_round).sleep();

        //
        // Run planning and pre/post
        //

        ros::Time desired_end_time =
            ros::Time::now() + ros::Duration(round_duration - message_latency);

        ROS_INFO_STREAM("Planner "
            << this->getId()
            << ": Distributed planning epoch "
            << planning_epoch
            << ", round " << planning_round);

        std::vector<Message> prior_decisions;
        filterDecisionsAndClearBuffer(planning_epoch, prior_decisions);

        // Compute the available time for this round, starting now
        auto duration = desired_end_time - ros::Time::now();

        Message message;
        this->plannerCallback(duration.toSec(), prior_decisions, message);

        ros::Time end_time = ros::Time::now();

        this->signMessage(planning_epoch, message);
        decision_pub.publish(message);

        if(end_time > desired_end_time + ros::Duration(1e-3))
        {
          ROS_INFO("Planner %d: overran end time (%10.2f) by %2.2f seconds",
              this->getId(), desired_end_time.toSec(),
              (end_time - desired_end_time).toSec());
        }
      }

      // Buffer all decisions from other robots. Further processing will occur
      // at planning time.
      void bufferDecision(const Message& decision)
      {
        std::lock_guard<std::mutex> lock(decision_mutex);

        auto planner_id = decision.signature.planner_id;

        if(planner_id != this->getId())
        {
          prior_decision_buffer.push_back(decision);
        }

        ROS_INFO_STREAM("Planner " << this->getId()
                                   << " received decision from " << planner_id);
      }

      void printStatistics() const
      {
        double expected_acceptance_rate = 0.5 * (1.0 - 1.0 / num_rounds);
        double actual_acceptance_rate =
            double(num_accepted) / (num_accepted + num_rejected);

        printf("Planner %u: Acceptance rate %0.3f, Expected %0.3f\n",
                 this->getId(), actual_acceptance_rate,
                 expected_acceptance_rate);
      }

    private:

      unsigned samplePlanningRound()
      {
        std::uniform_int_distribution<unsigned> distribution(0, num_rounds-1);

        return distribution(rng);
      }

      double getRoundDuration()
      {
        return this->epochDuration() / num_rounds;
      }

      // Collect decisions for the current epoch.
      // Clear the rest (which should be old).
      void filterDecisionsAndClearBuffer(unsigned planning_epoch,
                                         std::vector<Message>& prior_decisions)
      {
        std::lock_guard<std::mutex> lock(decision_mutex);

        unsigned round_accepted = 0, round_rejected = 0;

        for(auto& decision : prior_decision_buffer)
        {
          auto message_epoch = decision.signature.planning_epoch;

          if(message_epoch == planning_epoch)
          {
            prior_decisions.push_back(decision);

            ++round_accepted;
          }
          else
          {
            ROS_INFO_STREAM("Planner "
                            << this->getId() << ": Filtering out decision from "
                            << decision.signature.planner_id << " in epoch "
                            << message_epoch << " (current: " << planning_epoch
                            << ")");

            ++round_rejected;
          }
        }

        num_rejected += round_rejected;
        num_accepted += round_accepted;

        ROS_INFO_STREAM("Planner " << this->getId() << " Accepted "
                                   << prior_decisions.size() << " decisions");

        DistributedPlannerStatistics msg;
        msg.total_accepted = num_accepted;
        msg.total_rejected = num_rejected;
        msg.round_accepted = round_accepted;
        msg.round_rejected = round_rejected;

        statistics_pub.publish(msg);

        prior_decision_buffer.clear();
      }

      // Member variables

      std::mt19937 rng;

      // Amount of time in each round to allocate for messaging
      double message_latency;

      // number of sequential steps per epoch
      unsigned num_rounds;

      std::mutex decision_mutex;
      std::vector<Message> prior_decision_buffer;

      // The planner tracks accepted and rejected messages to verify that rates
      // match expectations
      unsigned num_accepted = 0;
      unsigned num_rejected = 0;

      ros::Publisher decision_pub;
      ros::Subscriber decision_sub;

      ros::Publisher statistics_pub;
  };

  // Automatically construct the distributed planner
  template <class Message>
  typename AbstractDistributedPlanner<Message>::Ptr makeDistributedPlanner(
      const ros::NodeHandle& node)
  {
    std::string coordination_method;
    if (!loadParam(node, "coordination_method", coordination_method))
    {
      ROS_ERROR(
          "Available coordination methods are: "
          "myopic, "
          "randomized_partitioning");
      return NULL;
    }

    typename AbstractDistributedPlanner<Message>::Ptr ret;

    ROS_INFO_STREAM(
        node.getNamespace()
        << " initializing distributed planner with coordination method: "
        << coordination_method);

    if(coordination_method == "myopic")
    {
      ret = std::make_shared<MyopicDistributedPlanner<Message>>();
    }
    else if(coordination_method == "random_partitioning")
    {
      ret = std::make_shared<RandomizedDistributedPlanner<Message>>();
    }
    else
    {
      ROS_ERROR_STREAM(node.getNamespace()
                       << ": " << coordination_method
                       << " is not a recognized coordination method for any "
                          "AbstractDistributedPlanner");
      return NULL;
    }

    if(!ret->initialize(node))
    {
      return NULL;
    }

    return ret;
  }
};

#endif /* DISTRIBUTEDPLANNER_H */
