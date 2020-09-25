# Distributed Randomized Sequential Partitions (RSP)

This package implements distributed submodular maximization via Randomized
Sequential Partitions (RSP) for online receding-horizon sensor planning.
This scheduling solver was initially designed for use in a system for
distributed exploration, described in the referenced work.

## Examples

`src/simple_distributed_planning_node.cpp` implements a minimal example of
scheduling via RSP.
Actual implementations of sensor planning processes should use the callback to
implement the local robot's planning process.

Run this node via `launch/distributed_planning_test.launch`

## Designing your own planner

This implementation of RSP is intended to be easy to incorporate into existing
single-robot receding-horizon planners.
RSP simply manages scheduling and communication and calls your planner at the
appropriate times.
To integrate RSP you should:
* Design a message to package your plans based on the instructions in
  `msg/DistributedPlannerSignature.msg`
* Pack and unpack decisions in these messages
* Plan (possibly approximately) conditionally on prior decisions in the callback
  you provide to the RSP planner, set via `setPlannerCallback`.
Otherwise, do not forget to `initialize`, `start`, and `stop` the planning
*thread*.

## Classes

* `RandomizedDistributedPlanner`: Implements RSP
* `MyopicDistributedPlanner`: Implements myopic planning
  (robots plan in parallel and do not coordinate)

## Assumptions

This implementation of RSP runs in time-synchronized epochs, whereas each epoch
corresponds to a submodular maximization (sensing) sub-problem.
All planners should have access to synchronized clocks.
Although, RSP should not be too sensitive to synchronization issues unless
running much faster than 1Hz.

We assume each planner node can communicate with an implicit local neighborhood
via a given ROS topic.
While this neighborhood may vary dynamically or suffer communication failures,
users are responsible for ensuring that the size of the neighborhood is
appropriate for their coordination needs.

## Parameters

* `distributed_planning/epoch_duration`: The duration of the epoch during which
  robots collectively solve a sensing subproblem.
* `distributed_planning/planner_id`: Integer id of the planning node
  (used to disambiguate the source of a message/assignment).
* `distributed_planning/num_rounds`: The number of sequential steps used for
  distributed planning.
  Greater numbers of rounds enable the planner to better account for
  inter-robot redundancy in sensing actions but means individual robots have
  less time for planning.
  In practice, 3-10 rounds is probably appropriate.
* `distributed_planning/message_latency`: Slack time added on to each round to
  account for overhead in messaging.
* `distributed_planning/planner_topic`: Name of the topic for distributed
  coordination.
  Directing messages to a root "/" topic can enable communication between all
  planners.
  However, *RSP is designed to admit arbitrary local neighborhoods and possible
  communication failure.*
  Piping messages through an intermediary can enable implementation of local
  communication neighborhoods.
* `coordination_method`: Selects the appropriate distributed planner for the
  factory method `makeDistributedPlanner` to load.
  Valid values are: `myopic` or `randomized_partitioning`.

## References

If you use this package in published work, pleace consider citing the following:

```
@phdthesis{corah2020phd,
  author = {Corah, Micah},
  title = {Sensor Planning for Large Numbers of Robots},
  school = {Carnegie Mellon University},
  year = {2020}
}
```
