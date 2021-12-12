import tf
import rospy
from geometry_msgs.msg import * 

def execute_motion(cor_list, moveit_interface,tf_buffer):
    # Construct the points forming a square around the center point
    p1, p2, p3, p4 = square_points(cor_list)

    # Organise poses in a list 
    pose_list = [p1, p2, p3, p4]
    stop_duration = rospy.Duration(5,0) # Set wait duration to 5 seconds 

    # Iterate over the 4 poses
    for pose in pose_list:

        target_pose = PoseStamped() # Initialise PoseStamped object
        target_pose.header.frame_id = "base_link" # Set the pose frame to base_link
        target_pose.header.stamp = rospy.Time.now() # Time stamp
        target_pose.pose = pose # 

        moveit_interface.move(target_pose,tf_buffer)
        print( "target reached")
        rospy.sleep(stop_duration)
        print ("wait finished")

# Define square points function
def square_points(cor):

    # Convert from RPY to quaternion
    quaternion = Quaternion(*tf.transformations.quaternion_from_euler(cor[3],cor[4],cor[5]))
    # Convert to meters
    x, y, z = cor[0] * 0.001,  cor[1] * 0.001, cor[2] * 0.001

    # Create square points with the corresponding x and y coordinates and with the same z
    # and orientation values 
    p1 = Pose(Point(x-0.025, y-0.025, z)
             , quaternion)
    p2 = Pose(Point(x-0.025, y+0.025, z)
             , quaternion)
    p3 = Pose(Point(x+0.025, y+0.025, z)
             , quaternion)
    p4 = Pose(Point(x+0.025, y-0.025, z)
             , quaternion)

    return p1,p2,p3,p4