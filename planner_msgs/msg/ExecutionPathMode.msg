# Execution path mode, defined in ExecutionPathType.
int32 kLocalPath = 0
int32 kHomingPath = 1
int32 kGlobalPath = 2

# Set one of above values.
int32 mode

# The path is in forward direction compared to current exploration direction or not
bool is_forward