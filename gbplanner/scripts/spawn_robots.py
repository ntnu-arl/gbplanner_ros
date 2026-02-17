#!/usr/bin/env python3
"""
Simple and stable robot spawner.
Reads robot configuration from ROS parameters and spawns each robot using roslaunch subprocess.
This approach is more stable than using the roslaunch API directly.
"""

import rospy
import os
import sys
import subprocess
import signal
import time

def spawn_robots():
    """Spawn robots by reading config from rosparam and calling roslaunch for each."""
    rospy.init_node('spawn_robots', anonymous=True)
    
    # Wait a moment for rosparam to load
    rospy.sleep(0.5)
    
    # Get robot list from rosparam
    try:
        robots = rospy.get_param('robots_config/robots', [])
    except KeyError:
        rospy.logwarn("No robots config found, using default single robot")
        robots = [{'name': 'rmf_obelix', 'base_name': 'rmf_obelix', 'x': 0.0, 'y': 0.0, 'z': 0.5}]
    
    if not robots:
        rospy.logwarn("Empty robots list, using default single robot")
        robots = [{'name': 'rmf_obelix', 'base_name': 'rmf_obelix', 'x': 0.0, 'y': 0.0, 'z': 0.5}]
    
    rospy.loginfo("Spawning %d robots", len(robots))
    
    # Wait for Gazebo to be ready before spawning robots
    rospy.loginfo("Waiting for Gazebo to be ready...")
    try:
        rospy.wait_for_service('/gazebo/spawn_urdf_model', timeout=30.0)
        rospy.loginfo("Gazebo is ready!")
    except rospy.ROSException:
        rospy.logwarn("Gazebo spawn service not available, but continuing anyway...")
    
    # Get global parameters
    use_sim_time = rospy.get_param('/use_sim_time', True)
    launch_prefix = rospy.get_param('~launch_prefix', '')
    
    # Store processes for cleanup
    processes = []
    
    # Spawn each robot using roslaunch subprocess
    for robot in robots:
        robot_name = robot.get('name', 'robot')
        base_name = robot.get('base_name', 'rmf_obelix')
        robot_x = str(robot.get('x', 0.0))
        robot_y = str(robot.get('y', 0.0))
        robot_z = str(robot.get('z', 0.5))
        
        rospy.loginfo("Spawning robot: %s at (%s, %s, %s)", robot_name, robot_x, robot_y, robot_z)
        
        # Build roslaunch command
        cmd = ['roslaunch', 'gbplanner', 'rmf_robot.launch',
               'robot_name:=' + robot_name,
               'base_name:=' + base_name,
               'robot_x:=' + robot_x,
               'robot_y:=' + robot_y,
               'robot_z:=' + robot_z,
               'use_sim_time:=' + str(use_sim_time)]
        
        if launch_prefix:
            cmd.append('launch_prefix:=' + launch_prefix)
        
        try:
            # Start roslaunch process
            # Use Popen with proper environment to ensure ROS_MASTER_URI is set
            env = os.environ.copy()
            # Ensure ROS environment is sourced (PATH, ROS_PACKAGE_PATH, etc.)
            # The subprocess should inherit the environment from the parent
            rospy.loginfo("Launching robot %s with command: %s", robot_name, ' '.join(cmd))
            # Don't pipe stdout/stderr - let them go to terminal so we can see what's happening
            # This also prevents roslaunch from detecting it's in a non-interactive subprocess
            process = subprocess.Popen(
                cmd,
                stdout=None,  # Don't capture - let it go to terminal
                stderr=None,  # Don't capture - let it go to terminal
                env=env,
                preexec_fn=os.setsid  # Create new process group
            )
            processes.append((robot_name, process))
            rospy.loginfo("Started robot %s (PID: %d)", robot_name, process.pid)
            
            # Longer delay between spawns to avoid Gazebo conflicts and segfaults
            # Gazebo needs time to fully process each model before the next one
            time.sleep(2.0)
            
        except Exception as e:
            rospy.logerr("Error spawning robot %s: %s", robot_name, str(e))
            continue
    
    if not processes:
        rospy.logerr("No robots spawned!")
        return
    
    rospy.loginfo("All %d robots spawned. Monitoring processes...", len(processes))
    
    # Monitor processes and keep node alive
    def signal_handler(sig, frame):
        """Handle shutdown signal."""
        rospy.loginfo("Received shutdown signal, terminating robot processes...")
        for robot_name, proc in processes:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
            except:
                pass
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    try:
        rate = rospy.Rate(1)  # Check once per second
        while not rospy.is_shutdown():
            # Check if any process died unexpectedly
            for robot_name, proc in processes:
                if proc.poll() is not None:
                    rospy.logwarn("Robot %s process exited with code %d", robot_name, proc.returncode)
                    rospy.logwarn("Check the roslaunch logs for robot %s to see why it exited", robot_name)
            rate.sleep()
    except rospy.ROSInterruptException:
        rospy.loginfo("Shutting down...")
    finally:
        # Cleanup: terminate all processes
        rospy.loginfo("Terminating all robot processes...")
        for robot_name, proc in processes:
            try:
                # Kill the process group
                os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                rospy.logwarn("Force killing robot %s", robot_name)
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                except:
                    pass
            except Exception as e:
                rospy.logwarn("Error terminating robot %s: %s", robot_name, str(e))

if __name__ == '__main__':
    try:
        spawn_robots()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        rospy.logerr("Fatal error: %s", str(e))
        import traceback
        traceback.print_exc()
        sys.exit(1)
