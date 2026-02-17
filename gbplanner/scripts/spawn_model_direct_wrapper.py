#!/usr/bin/env python3
"""
Wrapper script for spawn_model_direct.py
This wrapper is installed and calls the actual script from source directory.
This works around the install issue with catkin_simple.
"""
import sys
import os

# Get the package path
try:
    import rospkg
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('gbplanner')
    script_path = os.path.join(pkg_path, 'scripts', 'spawn_model_direct.py')
    
    # Check if script exists
    if not os.path.exists(script_path):
        print("ERROR: spawn_model_direct.py not found at: %s" % script_path, file=sys.stderr)
        sys.exit(1)
    
    # Execute the actual script with all arguments
    # Replace this process with the actual script
    os.execv(script_path, [script_path] + sys.argv[1:])
    
except Exception as e:
    print("ERROR in wrapper: %s" % str(e), file=sys.stderr)
    import traceback
    traceback.print_exc(file=sys.stderr)
    sys.exit(1)


