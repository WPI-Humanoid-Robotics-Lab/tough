# "Enum" for harness states

# Harness is detached
uint8 NONE = 0

# Harness is detached, soon will attach
uint8 DETACH_TO_ATTACH = 1

# Harness is attached, soon will lower
uint8 ATTACH_TO_LOWER = 2

# Harness is lowering, soon will change to high level control
uint8 LOWER_TO_STAND = 3

# Robot is changing to high level control, harness will detach soon
uint8 STAND_TO_DETACH = 4

# Harness is detaching, robot will be released soon
uint8 DETACH_TO_NONE = 5

# Current harness status
uint8 status

