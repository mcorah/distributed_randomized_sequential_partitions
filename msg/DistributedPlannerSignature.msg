# This signature is meant to be included as a component of messages describing
# decisions in a distributed planning process.
#
# Messages should fit the form:
#
# informative_planning/DistributedPlannerSignature signature
#
# <Decision content>

# Unique id for the planner
uint32 planner_id

# ID for the sequential planning epoch
# (64 bits because the "planning epoch" could be the number of seconds since the
# Linux epoch, etc)
uint64 planning_epoch
