#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import cv2
from cv_bridge import CvBridge, CvBridgeError
from moveit_msgs.msg import OrientationConstraint, Constraints
from geometry_msgs.msg import PoseStamped
from ar_track_alvar_msgs.msg import AlvarMarker, AlvarMarkers
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import Image
#Define the callback method which is called whenever this node receives a 
#message on its subscribed topic. The received message is passed as the 
#first argument to callback().
class ar_tracking:

    def __init__(self):
        self.total_markers = 2
        self.subscriber = None
        self.markers = None

    def callback(self, message):
        if len(message.markers) == self.total_markers:
            self.markers = message.markers
            self.subscriber.unregister()
            rospy.signal_shutdown("ar tags found")

    def begin_tracking(self):
        rospy.init_node('listener', anonymous=True)
        self.subscriber = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.callback)
        rospy.spin()
        return self.markers

class path_planning:

    def __init__(self, ar_tags):
        print (ar_tags)


    # TODO #
    def move(self, poses):
        #Initialize moveit_commander
        moveit_commander.roscpp_initialize(sys.argv)

        #Start a node
        rospy.init_node('moveit_node')

        #Initialize both arms
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
    #    left_arm = moveit_commander.MoveGroupCommander('left_arm')
        right_arm = moveit_commander.MoveGroupCommander('right_arm')
    #    left_arm.set_planner_id('RRTConnectkConfigDefault')
    #    left_arm.set_planning_time(10)
        right_arm.set_planner_id('RRTConnectkConfigDefault')
        right_arm.set_planning_time(10)
        waypoints = []

        # start with the current pose
        waypoints.append(right_arm.get_current_pose().pose)

        # first orient gripper and move forward (+x)
        wpose = geometry_msgs.msg.Pose()
        wpose.orientation.x = 0.0
        wpose.orientation.y = -1.0
        wpose.orientation.z = 0.0
        wpose.orientation.w = 0.0
        wpose.position.x = waypoints[0].position.x
        wpose.position.y = waypoints[0].position.y + 0.1
        wpose.position.z = waypoints[0].position.z
        waypoints.append(copy.deepcopy(wpose))

        # second move down
        #wpose.position.z -= 0.10
        #waypoints.append(copy.deepcopy(wpose))

        # third move to the side
        #wpose.position.y += 0.05
        #waypoints.append(copy.deepcopy(wpose))

        ## We want the cartesian path to be interpolated at a resolution of 1 cm
        ## which is why we will specify 0.01 as the eef_step in cartesian
        ## translation.  We will specify the jump threshold as 0.0, effectively
        ## disabling it.
        (plan, fraction) = right_arm.compute_cartesian_path(
                                   waypoints,   # waypoints to follow
                                   0.01,        # eef_step
                                   0.0)         # jump_threshold
                                   
        right_arm.execute(plan)


        goal_1 = PoseStamped()
        goal_1.header.frame_id = "base"
        #x, y, and z position
        goal_1.pose.position.x = 0.56
        goal_1.pose.position.y = 0.31
        goal_1.pose.position.z = -0.24
        
        #Orientation as a quaternion
        goal_1.pose.orientation.x = 0.0
        goal_1.pose.orientation.y = -1.0
        goal_1.pose.orientation.z = 0.0
        goal_1.pose.orientation.w = 0.0
        #Set the goal state to the pose you just defined
        right_arm.set_pose_target(goal_1)
        #Set the start state for the right arm
        right_arm.set_start_state_to_current_state()
        #Plan a path
        right_plan = right_arm.plan()
        #Execute the plan
        right_arm.execute(right_plan)


def main():
    tracker = ar_tracking()
    print (tracker.begin_tracking())

    # get image
    # homeography or w/e its called
    # plan drawing
    # do drawing on whiteboard

if __name__ == '__main__':
    main()
