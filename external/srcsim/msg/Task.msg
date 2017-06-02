# Task id
uint32 task

# Id of current checkpoint
uint32 current_checkpoint

# Array with the total time taken to complete each checkpoint. The time is zero
# for skipped checkpoints or checkpoints which were never started due to task
# timeout. This doesn't include penalties.
duration[] checkpoint_durations

# Array with the total penalty time for each checkpoint.
duration[] checkpoint_penalties

# Absolute sim time when this task started (ROS time)
time start_time

# Time elapsed since task started (ROS time). This is the current sim time
# minus start time, plus penalties for all checkpoints on this task.
# When the elapsed time reaches the task timeout, `timed_out` is set to true.
duration elapsed_time

# True if task timed out. This happens when the elapsed time plus the penalty
# time are larger than the timeout.
bool timed_out

# True if task has finished
bool finished
