# Execution path mode, defined in ExecutionPathType.
int32 K_LOCAL_PATH = 0
int32 K_HOMING_PATH = 1
int32 K_GLOBAL_PATH = 2

# Set one of above values.
int32 mode

# The path is in forward direction compared to current exploration direction or not
bool is_forward
