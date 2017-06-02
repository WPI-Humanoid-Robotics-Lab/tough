# Array with the total time taken to complete each checkpoint across all tasks.
# The time is zero for skipped checkpoints or checkpoints which were never
# started due to task timeout.
duration[] checkpoint_durations

# Array with the total penalty time for each checkpoint across all tasks.
duration[] checkpoint_penalties

# The sum of the score across all checkpoints
uint8 score

# The sum of the time taken to complete all successful checkpoints, including
# the time penalties for those checkpoints.
# Incomplete checkpoints are not taken into account (neither the time spent
# trying, nor the penalties)
duration total_completion_time
