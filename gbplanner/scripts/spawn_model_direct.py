#!/usr/bin/env python3
"""
Spawn Gazebo model directly with unique name.
This bypasses spawn_mav.launch's model spawning to avoid name conflicts.
Processes the xacro/urdf file and spawns via Gazebo service.
"""

# Print immediately to stderr so we can see if script is being called
import sys
print("=" * 60, file=sys.stderr)
print("SPAWN_MODEL_DIRECT.PY: Script file loaded", file=sys.stderr)
print("Python version: %s" % sys.version, file=sys.stderr)
print("Arguments received: %s" % str(sys.argv), file=sys.stderr)
print("=" * 60, file=sys.stderr)
sys.stderr.flush()

import rospy
import os
import subprocess
import tempfile
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose, Point, Quaternion

def process_xacro_to_urdf(xacro_file, mav_name, namespace=None, enable_mavlink_interface=False, enable_logging=False, enable_ground_truth=True):
    """Process xacro file to URDF using xacro command."""
    try:
        # Use xacro to process the file
        # xacro needs mav_name, namespace, and other parameters
        # If namespace not provided, use mav_name as namespace
        if namespace is None:
            namespace = mav_name
        # Build xacro command with all required parameters
        # These match what spawn_mav.launch typically passes
        cmd = ['xacro', xacro_file,
               'mav_name:=' + mav_name,
               'namespace:=' + namespace,
               'enable_mavlink_interface:=' + str(enable_mavlink_interface).lower(),
               'enable_logging:=' + str(enable_logging).lower(),
               'enable_ground_truth:=' + str(enable_ground_truth).lower()]
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=10)
        if result.returncode != 0:
            rospy.logerr("xacro processing failed: %s", result.stderr)
            rospy.logerr("xacro stdout: %s", result.stdout)
            return None
        return result.stdout
    except Exception as e:
        rospy.logerr("Error processing xacro: %s", str(e))
        return None

def spawn_model_direct(base_name, robot_name, x, y, z):
    """Spawn model directly via Gazebo service with unique name."""
    # Always init node - anonymous=True allows multiple instances
    rospy.init_node('spawn_model_direct_' + robot_name, anonymous=True)
    
    rospy.loginfo("=== Starting spawn_model_direct for robot: %s ===", robot_name)
    rospy.loginfo("Parameters: base_name=%s, robot_name=%s, pos=(%s, %s, %s)", base_name, robot_name, x, y, z)
    
    # Wait a bit for Gazebo to fully initialize
    rospy.sleep(2.0)
    
    # Wait for Gazebo services - try both URDF and SDF services
    rospy.loginfo("Waiting for Gazebo spawn services...")
    spawn_service_name = None
    try:
        rospy.wait_for_service('/gazebo/spawn_urdf_model', timeout=10.0)
        spawn_service_name = '/gazebo/spawn_urdf_model'
        rospy.loginfo("Found /gazebo/spawn_urdf_model service")
    except rospy.ROSException:
        rospy.logwarn("/gazebo/spawn_urdf_model not available, trying /gazebo/spawn_sdf_model")
        try:
            rospy.wait_for_service('/gazebo/spawn_sdf_model', timeout=10.0)
            spawn_service_name = '/gazebo/spawn_sdf_model'
            rospy.loginfo("Found /gazebo/spawn_sdf_model service")
        except rospy.ROSException:
            rospy.logerr("No Gazebo spawn services available!")
            rospy.logerr("Please check that Gazebo is running and gazebo_ros plugins are loaded")
            sys.exit(1)
    
    if not spawn_service_name:
        rospy.logerr("Failed to find any spawn service!")
        sys.exit(1)
    
    try:
        spawn_model = rospy.ServiceProxy(spawn_service_name, SpawnModel)
        
        # Get model file path
        try:
            import rospkg
            rospack = rospkg.RosPack()
            rotors_desc_path = rospack.get_path('rotors_description')
            model_file = os.path.join(rotors_desc_path, 'urdf', base_name + '.gazebo')
        except Exception as e:
            rospy.logerr("Could not find rotors_description package: %s", str(e))
            return
        
        if not os.path.exists(model_file):
            rospy.logerr("Model file not found: %s", model_file)
            return
        
        # Process xacro to URDF
        # Use base_name for mav_name and namespace (xacro file resolution)
        # The URDF will be modified later to use robot_name for TF frames
        # Pass enable_ground_truth=true to match single-robot setup
        rospy.loginfo("Processing xacro file: %s with mav_name=%s, namespace=%s", model_file, base_name, base_name)
        urdf_xml = process_xacro_to_urdf(model_file, base_name, namespace=base_name, 
                                         enable_mavlink_interface=False, 
                                         enable_logging=False, 
                                         enable_ground_truth=True)
        
        if not urdf_xml:
            rospy.logerr("Failed to process xacro file")
            return
        
        # Replace frame names in URDF to use robot_name instead of base_name
        # Use simple string replacement but be careful to preserve XML structure
        # XML parsing loses comments and formatting, so we use targeted string replacement
        import re
        
        # Store original length for validation
        original_length = len(urdf_xml)
        
        # Debug: Check what we're replacing
        rospy.loginfo("Replacing '%s' with '%s' in URDF", base_name, robot_name)
        base_name_count = urdf_xml.count(base_name)
        rospy.loginfo("Found '%s' %d times in URDF before replacement", base_name, base_name_count)
        
        # Replace in order of specificity to avoid double-replacement issues
        # 1. Replace link names: <link name="base_name/...">
        urdf_xml = urdf_xml.replace('<link name="' + base_name + '/', '<link name="' + robot_name + '/')
        
        # 2. Replace joint names: <joint name="base_name/...">
        urdf_xml = urdf_xml.replace('<joint name="' + base_name + '/', '<joint name="' + robot_name + '/')
        
        # 3. Replace joint parent/child: <parent link="base_name/..."> or <child link="base_name/...">
        urdf_xml = urdf_xml.replace('<parent link="' + base_name + '/', '<parent link="' + robot_name + '/')
        urdf_xml = urdf_xml.replace('<child link="' + base_name + '/', '<child link="' + robot_name + '/')
        
        # 3b. Also replace non-namespaced joint names that might be created by macros (e.g., mount_joint)
        # This must happen AFTER we replace the child link names, so we can check for robot_name/velodyne
        # Pattern: <joint name="mount_joint"> with <child link="robot_name/velodyne">
        if '<joint name="mount_joint"' in urdf_xml and robot_name + '/velodyne' in urdf_xml:
            # Replace mount_joint with namespaced version
            urdf_xml = urdf_xml.replace('<joint name="mount_joint"', '<joint name="' + robot_name + '/mount_joint"')
            rospy.loginfo("Replaced non-namespaced mount_joint with namespaced version: %s/mount_joint", robot_name)
        
        # 4. Replace frame_id in plugins: frame_id="base_name/..."
        urdf_xml = urdf_xml.replace('frame_id="' + base_name + '/', 'frame_id="' + robot_name + '/')
        
        # 5. Replace robot name attribute: <robot name="base_name">
        urdf_xml = urdf_xml.replace('<robot name="' + base_name + '"', '<robot name="' + robot_name + '"')
        
        # 6. Replace in plugin frame references (common patterns in gazebo plugins)
        urdf_xml = urdf_xml.replace('<frameName>' + base_name + '/', '<frameName>' + robot_name + '/')
        urdf_xml = urdf_xml.replace('frame_name="' + base_name + '/', 'frame_name="' + robot_name + '/')
        urdf_xml = urdf_xml.replace('reference_frame="' + base_name + '/', 'reference_frame="' + robot_name + '/')
        # Also replace frameName that might have been expanded from ${name} variable
        # Pattern: <frameName>rmf_obelix/velodyne</frameName> -> <frameName>rmf_obelix_1/velodyne</frameName>
        urdf_xml = re.sub(r'(<frameName>)' + re.escape(base_name) + r'(/velodyne</frameName>)', 
                         r'\1' + robot_name + r'\2', urdf_xml)
        
        # 7. Replace ${namespace} in plugin configurations (xacro variables that weren't expanded)
        # This is critical for plugins to work correctly - they need the actual namespace, not the variable
        urdf_xml = urdf_xml.replace('${namespace}', robot_name)
        rospy.loginfo("Replaced ${namespace} variables with robot_name: %s", robot_name)
        
        # 8. DO NOT replace topic names - Gazebo's robot_namespace parameter handles this automatically
        # When robot_namespace is set in spawn_model service, Gazebo automatically prefixes
        # all plugin topics with the namespace. Using absolute topics breaks this mechanism.
        # Keep relative topics like "velodyne_points" and let Gazebo namespace them to "/robot_name/velodyne_points"
        
        # Debug: Verify replacement worked
        remaining_base_name = urdf_xml.count(base_name)
        robot_name_count = urdf_xml.count(robot_name)
        rospy.loginfo("After replacement: '%s' appears %d times, '%s' appears %d times", 
                      base_name, remaining_base_name, robot_name, robot_name_count)
        if remaining_base_name > 0:
            # Find where base_name still appears
            import re
            remaining_matches = re.findall(r'[^"]*' + re.escape(base_name) + r'[^"]*', urdf_xml)
            rospy.logwarn("WARNING: '%s' still appears %d times after replacement! Sample matches: %s", 
                         base_name, remaining_base_name, remaining_matches[:5])
        
        # Validate that we didn't lose significant content (comments might be lost but structure should remain)
        new_length = len(urdf_xml)
        if new_length < original_length * 0.9:  # If we lost more than 10%, something's wrong
            rospy.logwarn("WARNING: URDF length decreased significantly (was %d, now %d)", original_length, new_length)
        
        rospy.loginfo("URDF replacement via string replacement completed (length: %d -> %d)", original_length, new_length)
        
        # Check for velodyne link in URDF
        if 'velodyne' in urdf_xml.lower():
            velodyne_count = urdf_xml.lower().count('velodyne')
            rospy.loginfo("URDF contains 'velodyne' %d times", velodyne_count)
            # Try to find the velodyne link
            velodyne_link_pattern = '<link name="' + robot_name + '/velodyne'
            if velodyne_link_pattern in urdf_xml:
                rospy.loginfo("Found velodyne link: %s", velodyne_link_pattern)
                # Extract velodyne link section for debugging
                import re
                velodyne_link_match = re.search(r'<link name="' + robot_name + r'/velodyne[^>]*>.*?</link>', urdf_xml, re.DOTALL)
                if velodyne_link_match:
                    velodyne_link_xml = velodyne_link_match.group(0)
                    rospy.loginfo("Velodyne link XML (first 500 chars): %s", velodyne_link_xml[:500])
                # Check for velodyne joint - the OS0-128 macro creates a joint named "mount_joint"
                # that connects parent (base_link) to child (velodyne link)
                # The joint name might be namespaced or not, depending on the macro implementation
                velodyne_joint_patterns = [
                    '<joint name="' + robot_name + '/mount_joint',  # Namespaced mount_joint
                    '<joint name="mount_joint"',  # Non-namespaced mount_joint (from macro)
                    '<joint name="' + robot_name + '/velodyne',
                    '<joint name="' + robot_name + '/velodyne_joint',
                ]
                joint_found = False
                for pattern in velodyne_joint_patterns:
                    if pattern in urdf_xml:
                        rospy.loginfo("Found velodyne joint with pattern: %s", pattern)
                        joint_found = True
                        # Try to extract the joint XML - look for mount_joint that connects to velodyne
                        joint_match = re.search(r'<joint[^>]*name="[^"]*mount_joint[^"]*"[^>]*>.*?</joint>', urdf_xml, re.DOTALL | re.IGNORECASE)
                        if joint_match:
                            velodyne_joint_xml = joint_match.group(0)
                            rospy.loginfo("Velodyne mount_joint XML (first 500 chars): %s", velodyne_joint_xml[:500])
                            # Check if it connects to velodyne link
                            if robot_name + '/velodyne' in velodyne_joint_xml:
                                rospy.loginfo("Confirmed: mount_joint connects to velodyne link")
                            else:
                                rospy.logwarn("WARNING: mount_joint found but doesn't connect to velodyne link!")
                        break
                
                if not joint_found:
                    rospy.logwarn("WARNING: Velodyne mount_joint not found! Searching for any joints connecting to velodyne...")
                    # Search for any joint that has velodyne as child or parent
                    all_velodyne_joints = re.findall(r'<joint[^>]*>.*?<child link="[^"]*velodyne[^"]*"', urdf_xml, re.DOTALL | re.IGNORECASE)
                    if all_velodyne_joints:
                        rospy.logwarn("Found joints connecting to velodyne: %s", [j[:200] for j in all_velodyne_joints[:3]])
                    else:
                        rospy.logerr("ERROR: No joint connecting to velodyne link found! This will prevent the link from being created in Gazebo!")
                        rospy.logerr("The OS0-128 macro should create a 'mount_joint' connecting base_link to velodyne link")
                # Check for velodyne plugin - look for libgazebo_ros_lidar plugin in gazebo block
                # The plugin is in a <gazebo reference="...velodyne..."> tag
                velodyne_gazebo_match = re.search(r'<gazebo[^>]*reference="[^"]*velodyne[^"]*"[^>]*>.*?</gazebo>', urdf_xml, re.DOTALL | re.IGNORECASE)
                if velodyne_gazebo_match:
                    velodyne_gazebo_xml = velodyne_gazebo_match.group(0)
                    rospy.loginfo("Velodyne gazebo block found (first 800 chars): %s", velodyne_gazebo_xml[:800])
                    # Check for lidar plugin
                    if 'libgazebo_ros_lidar' in velodyne_gazebo_xml or 'gazebo_ros_laser_controller' in velodyne_gazebo_xml:
                        rospy.loginfo("Found libgazebo_ros_lidar plugin in velodyne gazebo block")
                        # Check if frameName is correct
                        if robot_name + '/velodyne' in velodyne_gazebo_xml:
                            rospy.loginfo("Plugin frameName is correctly namespaced: %s/velodyne", robot_name)
                        else:
                            rospy.logwarn("WARNING: Plugin frameName might not be correctly namespaced!")
                            frame_match = re.search(r'<frameName>[^<]*</frameName>', velodyne_gazebo_xml, re.IGNORECASE)
                            if frame_match:
                                rospy.logwarn("Found frameName: %s (should be %s/velodyne)", frame_match.group(0), robot_name)
                    else:
                        rospy.logwarn("WARNING: libgazebo_ros_lidar plugin not found in velodyne gazebo block!")
                else:
                    rospy.logwarn("WARNING: Velodyne gazebo block not found! Searching for any plugin...")
                    # Fallback: search for any plugin mentioning velodyne
                    if '<plugin' in urdf_xml:
                        plugin_match = re.search(r'<plugin[^>]*>.*?velodyne.*?</plugin>', urdf_xml, re.DOTALL | re.IGNORECASE)
                        if plugin_match:
                            plugin_xml = plugin_match.group(0)
                            rospy.loginfo("Velodyne plugin found (first 500 chars): %s", plugin_xml[:500])
                        else:
                            rospy.logwarn("WARNING: No plugin found for velodyne!")
            else:
                rospy.logwarn("WARNING: Velodyne link pattern '%s' not found in URDF!", velodyne_link_pattern)
                # Check for any velodyne link
                velodyne_links = re.findall(r'<link name="[^"]*velodyne[^"]*"', urdf_xml)
                if velodyne_links:
                    rospy.logwarn("Found velodyne links with different names: %s", velodyne_links)
                else:
                    rospy.logerr("ERROR: No velodyne link found in URDF at all!")
        else:
            rospy.logerr("ERROR: URDF does not contain 'velodyne' - sensor may not be included!")
        
        # Validate URDF XML structure using Python's xml parser
        try:
            import xml.etree.ElementTree as ET
            # Try to parse the URDF to ensure it's valid XML
            ET.fromstring(urdf_xml)
            rospy.loginfo("URDF XML validation passed")
        except ET.ParseError as e:
            rospy.logerr("URDF XML validation FAILED: %s", str(e))
            rospy.logerr("This URDF will likely fail in Gazebo!")
            # Write to temp file for debugging
            try:
                with tempfile.NamedTemporaryFile(mode='w', suffix='.urdf', delete=False) as f:
                    f.write(urdf_xml)
                    rospy.logerr("Invalid URDF written to: %s", f.name)
            except:
                pass
        
        # Basic XML validation - check for balanced tags
        open_tags = urdf_xml.count('<')
        close_tags = urdf_xml.count('>')
        if open_tags != close_tags:
            rospy.logwarn("WARNING: XML tag count mismatch (open: %d, close: %d)", open_tags, close_tags)
        
        # Debug: log a snippet of the URDF to verify replacement
        rospy.loginfo("URDF replacement complete for robot %s (replaced %s -> %s)", robot_name, base_name, robot_name)
        rospy.loginfo("URDF sample (first 500 chars): %s", urdf_xml[:500])
        
        # Check if visual/collision elements are present
        visual_count = urdf_xml.count('<visual')
        collision_count = urdf_xml.count('<collision')
        link_count = urdf_xml.count('<link name=')
        rospy.loginfo("URDF contains %d links, %d visual elements, %d collision elements", link_count, visual_count, collision_count)
        
        # Check for base_link specifically
        if robot_name + '/base_link' in urdf_xml:
            base_link_start = urdf_xml.find('<link name="' + robot_name + '/base_link')
            if base_link_start != -1:
                base_link_end = urdf_xml.find('</link>', base_link_start)
                if base_link_end != -1:
                    base_link_xml = urdf_xml[base_link_start:base_link_end+7]
                    has_visual = '<visual' in base_link_xml
                    has_collision = '<collision' in base_link_xml
                    rospy.loginfo("base_link found: has_visual=%s, has_collision=%s", has_visual, has_collision)
                    if not has_visual and not has_collision:
                        rospy.logwarn("WARNING: base_link has no visual or collision elements! This might cause rendering issues.")
                        rospy.logwarn("base_link XML snippet: %s", base_link_xml[:300])
        
        # Create pose
        pose = Pose()
        pose.position = Point(float(x), float(y), float(z))
        pose.orientation = Quaternion(0, 0, 0, 1)  # Identity quaternion
        
        # Spawn with unique name
        rospy.loginfo("Spawning model %s at (%s, %s, %s) using service %s", robot_name, x, y, z, spawn_service_name)
        rospy.loginfo("URDF XML length: %d characters", len(urdf_xml))
        
        # Add a small delay to avoid race conditions when spawning multiple models
        # This gives Gazebo time to finish processing the previous model
        rospy.sleep(0.5)
        
        try:
            # Spawn with robot_namespace - Gazebo will automatically namespace plugin topics
            # The URDF links are pre-namespaced for TF frames, but plugin topics should be relative
            # Gazebo's robot_namespace will prefix plugin topics with the namespace
            rospy.loginfo("Spawning model with robot_namespace=%s", robot_name)
            spawn_resp = spawn_model(
                model_name=robot_name,
                model_xml=urdf_xml,
                robot_namespace=robot_name,  # This namespaces plugin topics automatically
                initial_pose=pose,
                reference_frame='world'
            )
        except rospy.ServiceException as e:
            rospy.logerr("Service call exception: %s", str(e))
            rospy.logerr("This might indicate the service doesn't accept URDF, trying to convert to SDF...")
            # Try converting URDF to SDF if needed
            # Note: SpawnModel is already imported at top of file
            # For SDF, we might need to wrap URDF differently
            # But for now, just log the error
            rospy.logerr("Cannot automatically convert URDF to SDF. Please check service type.")
            return
        
        if spawn_resp.success:
            rospy.loginfo("=== Successfully spawned model %s ===", robot_name)
            
            # Gazebo should publish world->model_name/base_link automatically,
            # but when using robot_namespace it might not work correctly.
            # Add a static transform publisher as backup to ensure TF tree is connected
            import subprocess
            # static_transform_publisher format: x y z yaw pitch roll frame_id child_frame_id period_in_ms
            tf_cmd = [
                'rosrun', 'tf', 'static_transform_publisher',
                str(x), str(y), str(z),  # translation (x y z)
                '0', '0', '0',  # rotation (yaw pitch roll in radians)
                'world', robot_name + '/base_link', '100'
            ]
            rospy.loginfo("Publishing static transform: world -> %s/base_link at (%s, %s, %s)", robot_name, x, y, z)
            # Run in background - this will keep running
            subprocess.Popen(tf_cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            rospy.loginfo("Static transform publisher started for world -> %s/base_link", robot_name)
            
            # Verify model exists in Gazebo
            rospy.sleep(0.5)  # Give Gazebo time to register the model
            try:
                from gazebo_msgs.srv import GetModelState
                get_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
                state_resp = get_state(model_name=robot_name, relative_entity_name='')
                if state_resp.success:
                    rospy.loginfo("Verified: Model %s exists in Gazebo at position (%f, %f, %f)", 
                                 robot_name, state_resp.pose.position.x, 
                                 state_resp.pose.position.y, state_resp.pose.position.z)
                else:
                    rospy.logwarn("Model %s spawn reported success but not found in Gazebo!", robot_name)
            except Exception as e:
                rospy.logwarn("Could not verify model existence: %s", str(e))
            
            # Keep node alive to prevent launch file from thinking everything is done
            rospy.loginfo("Spawn complete. Keeping node alive (other nodes will keep launch file running).")
            rospy.spin()  # Keep node alive indefinitely
            return
        else:
            rospy.logerr("=== Failed to spawn model %s ===", robot_name)
            rospy.logerr("Status message: %s", spawn_resp.status_message)
            rospy.logerr("Possible causes:")
            rospy.logerr("  1. Model name already exists in Gazebo")
            rospy.logerr("  2. URDF is malformed")
            rospy.logerr("  3. Invalid spawn position")
            rospy.logerr("  4. Gazebo plugin error")
            rospy.logerr("Check Gazebo console for more details")
            # Keep node alive even on failure - let respawn handle retries
            # This prevents launch file from exiting
            rospy.logwarn("Spawn failed but keeping node alive for respawn...")
            rospy.spin()  # Keep node alive so respawn can retry
            return
            
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", str(e))
        rospy.logwarn("Keeping node alive for respawn...")
        rospy.spin()  # Keep node alive so respawn can retry
        return
    except Exception as e:
        rospy.logerr("Error spawning model: %s", str(e))
        import traceback
        traceback.print_exc()
        rospy.logwarn("Keeping node alive for respawn...")
        rospy.spin()  # Keep node alive so respawn can retry
        return

if __name__ == '__main__':
    # Print to stderr so it shows up even if stdout is buffered
    print("=" * 60, file=sys.stderr)
    print("SPAWN_MODEL_DIRECT.PY STARTING", file=sys.stderr)
    print("Arguments: %s" % str(sys.argv), file=sys.stderr)
    print("=" * 60, file=sys.stderr)
    sys.stderr.flush()
    
    if len(sys.argv) < 6:
        print("Usage: spawn_model_direct.py <base_name> <robot_name> <x> <y> <z>", file=sys.stderr)
        print("Got %d arguments: %s" % (len(sys.argv), str(sys.argv)), file=sys.stderr)
        sys.exit(1)
    
    base_name = sys.argv[1]
    robot_name = sys.argv[2]
    x = sys.argv[3]
    y = sys.argv[4]
    z = sys.argv[5]
    
    print("Parsed arguments: base_name=%s, robot_name=%s, pos=(%s, %s, %s)" % 
          (base_name, robot_name, x, y, z), file=sys.stderr)
    sys.stderr.flush()
    
    try:
        spawn_model_direct(base_name, robot_name, x, y, z)
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        print("FATAL ERROR in spawn_model_direct: %s" % str(e), file=sys.stderr)
        import traceback
        traceback.print_exc(file=sys.stderr)
        sys.stderr.flush()
        raise

