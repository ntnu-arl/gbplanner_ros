import rospy
from sensor_msgs import PointCloud2
from std_msgs import String, Float32MultiArray, Float32, Int16


cloud_size_publisher = rospy.Publisher(
    "/gbplanner_stats/cloud_sizes", Float32MultiArray, queue_size=10
)

occ_cloud_buffer = []
complete_cloud_buffer = []

buffer_size = 5


def occ_cloud_callback(cloud_msg):
    global occ_cloud_buffer
    occ_cloud_buffer.append(cloud_msg)
    if len(occ_cloud_buffer) > buffer_size:
        occ_cloud_buffer.pop(0)


def complete_cloud_callback(cloud_msg):
    global complete_cloud_buffer
    complete_cloud_buffer.append(cloud_msg)


def cloud_process_callback(event):
    global occ_cloud_buffer, complete_cloud_buffer
    if len(occ_cloud_buffer) > 0 and len(complete_cloud_buffer) > 0:
        current_occ_cloud = occ_cloud_buffer.pop(0)
        current_complete_cloud = None
        closest_time = 9999999999999
        for cc in complete_cloud_buffer:
            if (
                abs(
                    cc.header.stamp.to_nsec() - current_occ_cloud.header.stamp.to_nsec()
                )
                < closest_time
            ):
                current_complete_cloud = cc
                closest_time = abs(
                    cc.header.stamp.to_nsec() - current_occ_cloud.header.stamp.to_nsec()
                )
        complete_cloud_buffer.remove(current_complete_cloud)
