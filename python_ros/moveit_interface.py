import moveit_commander
import moveit_msgs.msg
import tf2_geometry_msgs
import rospy


# Helper class to interact with Moveit! interface 
class MoveitInterface:

    def __init__(self, eef_name):
        self.eef_name = eef_name 
        self.robot = moveit_commander.RobotCommander() # Initialise moveit! robot commander
        self.group = moveit_commander.MoveGroupCommander(self.eef_name) # Initialise Moveit! planning group
        self.planning_frame = self.group.get_planning_frame() # Get the frame in whixh all the plannings are performed

    # Plan and execute the path to the received pose 
    def move(self,pose, tf_buffer):
        target_pose = self.transform_pose(pose,tf_buffer)
        self.group.set_pose_target(target_pose)
        plan = self.group.plan()
        self.group.execute(plan)

    # Convert the target pose in the robot base frame, to the manipulator planning frame 
    def transform_pose(self,local_pose, tf_buffer):
        # Get the transform between base_link and planning frame at planning time
        transform = tf_buffer.lookup_transform(self.planning_frame,
                                       # source frame:
                                       local_pose.header.frame_id,
                                       # get the tf at the time the pose was valid
                                       local_pose.header.stamp,
                                       # wait for at most 1 second for transform, otherwise throw
                                       rospy.Duration(1.0))
        # Get transformed pose  
        pose_transformed = tf2_geometry_msgs.do_transform_pose(local_pose, transform)
        return pose_transformed 

