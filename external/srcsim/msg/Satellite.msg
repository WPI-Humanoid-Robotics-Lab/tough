# Target pitch angle in radians
float64 target_pitch

# Target yaw angle in radians
float64 target_yaw

# Current pitch angle in radians
float64 current_pitch

# Current yaw angle in radians
float64 current_yaw

# True if the current pitch angle is within a tolerance distance from the
# target pitch angle
bool pitch_correct_now

# True if the current yaw angle is within a tolerance distance from the
# target yaw angle
bool yaw_correct_now

# True if the pitch angle has been correct for enough time
bool pitch_completed

# True if the yaw angle has been correct for enough time
bool yaw_completed
