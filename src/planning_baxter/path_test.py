#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from moveit_msgs.msg import OrientationConstraint, Constraints
from geometry_msgs.msg import PoseStamped

def main():
    #Initialize moveit_commander
    moveit_commander.roscpp_initialize(sys.argv)

    #Start a node
    rospy.init_node('moveit_node')

    #Initialize both arms
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
#    left_arm = moveit_commander.MoveGroupCommander('left_arm')
    right_arm = moveit_commander.MoveGroupCommander('left_arm')
#    left_arm.set_planner_id('RRTConnectkConfigDefault')
#    left_arm.set_planning_time(10)
    right_arm.set_planner_id('RRTConnectkConfigDefault')
    right_arm.set_planning_time(10)

    #First goal pose ------------------------------------------------------
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
    raw_input('Press <Enter> to move the right arm to goal pose 1 (path constraints are never enforced during this motion): ')
    right_arm.execute(right_plan)

    #First goal pose ------------------------------------------------------
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
    raw_input('Press <Enter> to move the right arm to goal pose 1 (path constraints are never enforced during this motion): ')
    right_arm.execute(right_plan)

    #Second goal pose -----------------------------------------------------
    rospy.sleep(2.0)
    goal_2 = PoseStamped()
    goal_2.header.frame_id = "base"

    #x, y, and z position
    goal_2.pose.position.x = 0.48
    goal_2.pose.position.y = 0
    goal_2.pose.position.z = -0.24
    
    #Orientation as a quaternion
    goal_2.pose.orientation.x = 0.0
    goal_2.pose.orientation.y = -1.0
    goal_2.pose.orientation.z = 0.0
    goal_2.pose.orientation.w = 0.0

    #Set the goal state to the pose you just defined
    right_arm.set_pose_target(goal_2)

    #Set the start state for the right arm
    right_arm.set_start_state_to_current_state()

    # #Create a path constraint for the arm
    # #UNCOMMENT TO ENABLE ORIENTATION CONSTRAINTS
    # orien_const = OrientationConstraint()
    # orien_const.link_name = "right_gripper";
    # orien_const.header.frame_id = "base";
    # orien_const.orientation.y = -1.0;
    # orien_const.absolute_x_axis_tolerance = 0.1;
    # orien_const.absolute_y_axis_tolerance = 0.1;
    # orien_const.absolute_z_axis_tolerance = 0.1;
    # orien_const.weight = 1.0;
    # consts = Constraints()
    # consts.orientation_constraints = [orien_const]
    # right_arm.set_path_constraints(consts)

    #Plan a path
    right_plan = right_arm.plan()

    #Execute the plan
    raw_input('Press <Enter> to move the right arm to goal pose 2: ')
    right_arm.execute(right_plan)

    """
    #Third goal pose -----------------------------------------------------
    rospy.sleep(2.0)
    goal_3 = PoseStamped()
    goal_3.header.frame_id = "base"

    #x, y, and z position
    goal_3.pose.position.x = 0.6
    goal_3.pose.position.y = -0.1
    goal_3.pose.position.z = 0.1
    
    #Orientation as a quaternion
    goal_3.pose.orientation.x = 0.0
    goal_3.pose.orientation.y = -1.0
    goal_3.pose.orientation.z = 0.0
    goal_3.pose.orientation.w = 0.0

    #Set the goal state to the pose you just defined
    right_arm.set_pose_target(goal_3)

    #Set the start state for the right arm
    right_arm.set_start_state_to_current_state()

    # #Create a path constraint for the arm
    # #UNCOMMENT TO ENABLE ORIENTATION CONSTRAINTS
    # orien_const = OrientationConstraint()
    # orien_const.link_name = "right_gripper";
    # orien_const.header.frame_id = "base";
    # orien_const.orientation.y = -1.0;
    # orien_const.absolute_x_axis_tolerance = 0.1;
    # orien_const.absolute_y_axis_tolerance = 0.1;
    # orien_const.absolute_z_axis_tolerance = 0.1;
    # orien_const.weight = 1.0;
    # consts = Constraints()
    # consts.orientation_constraints = [orien_const]
    # right_arm.set_path_constraints(consts)

    #Plan a path
    right_plan = right_arm.plan()

    #Execute the plan
    raw_input('Press <Enter> to move the right arm to goal pose 3: ')
    right_arm.execute(right_plan)



    raw_input('Cartesian path')
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
    (plan3, fraction) = right_arm.compute_cartesian_path(
                               waypoints,   # waypoints to follow
                               0.01,        # eef_step
                               0.0)         # jump_threshold
                               
    right_arm.execute(plan3)
    """

if __name__ == '__main__':
    main()
