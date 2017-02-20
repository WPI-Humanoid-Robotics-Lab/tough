# Task id
uint32 task

# Id of current checkpoint
uint32 current_checkpoint

# Array of times when checkpoints were completed. If a checkpoint was skipped, 
# its time is zero.
time[] checkpoints_completion

# Time when this task started (ROS time)
time start_time

# Time elapsed since task started (ROS time)
time elapsed_time

# True if task timed out
bool timed_out

# True if task has finished
bool finished
