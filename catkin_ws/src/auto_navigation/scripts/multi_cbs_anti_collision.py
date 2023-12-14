#!/usr/bin/env python3

import rospy
import math
import threading
import time
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import PoseStamped
import yaml
import numpy as np
import cv2

rospy.init_node('goal_pose')
def convert_sim_to_real_poses(points, matrix):
    real_poses = []
    for point in points:
        sim_point = np.array([point[0], point[1], 1])
        transformed_point = np.dot(matrix, sim_point)
        transformed_point = transformed_point / transformed_point[2]
        real_pose = [transformed_point[0], transformed_point[1]]
        real_poses.append(real_pose)
    print(real_poses)
    return real_poses


def check_goal_reached(init_pose, goal_coor, bias):
    if (init_pose.pose.position.x > goal_coor[0] - bias and init_pose.pose.position.x < goal_coor[0] + bias
            and init_pose.pose.position.y > goal_coor[1] - bias and init_pose.pose.position.y < goal_coor[1] + bias):
        return True
    else:
        return False

def control_agent(agent_id, coordinates):
    global state
    global lock

    topic_name = f'agent{agent_id}/cmd_vel'
    cmd_pub = rospy.Publisher(topic_name, Twist, queue_size=10)

    twist = Twist()
    init_pose = rospy.wait_for_message(f'/id{agent_id}/aruco_single/pose', PoseStamped)

    for coordinate in coordinates:
        goal_x = coordinate[0]
        goal_y = coordinate[1]

        while not check_goal_reached(init_pose, [goal_x, goal_y], 0.015):
            init_pose = rospy.wait_for_message(f'/id{agent_id}/aruco_single/pose', PoseStamped)
            orientation_q = init_pose.pose.orientation
            orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
            (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
            Orientation = yaw
            dx = goal_x - init_pose.pose.position.x
            dy = goal_y - init_pose.pose.position.y
            distance = math.dist([init_pose.pose.position.x, init_pose.pose.position.y], [goal_x, goal_y])
            goal_direct = math.atan2(dy, dx)

            print(f"Agent {agent_id}: current_coor", [init_pose.pose.position.x, init_pose.pose.position.y])
            print(f"Agent {agent_id}: goal_coor", [goal_x, goal_y])
            print(f"Agent {agent_id}: Orientation", Orientation)
            print(f"Agent {agent_id}: goal_direct", goal_direct)

            if (Orientation < 0):
                Orientation = Orientation + 2 * math.pi
            if (goal_direct < 0):
                goal_direct = goal_direct + 2 * math.pi

            theta = goal_direct - Orientation

            if theta < 0 and abs(theta) > abs(theta + 2 * math.pi):
                theta = theta + 2 * math.pi
            elif theta > 0 and abs(theta - 2 * math.pi) < theta:
                theta = theta - 2 * math.pi

            print(f"Agent {agent_id}: theta:", theta)

            k2 = 0.4
            linear = 1.5
            angular = k2 * theta
            twist.linear.x = linear * ((distance * 10) ** (1/6)) * (((2 / math.pi) * abs(theta) - 1) ** 8)
            twist.angular.z = -angular
            cmd_pub.publish(twist)

        # Block until all agents are ready
        while (True):
            with lock:
                if len(set(state.values())) == 1:
                    break
            time.sleep(0.01)

        # Toggle state
        state[agent_id] = not state[agent_id]

   
# Define agent ids
ids = [321, 325]

# Define points in the simulation plane and the real-world plane
pose_tl = rospy.wait_for_message('/id500/aruco_single/pose', PoseStamped)
pose_tr = rospy.wait_for_message('/id501/aruco_single/pose', PoseStamped)
pose_br = rospy.wait_for_message('/id502/aruco_single/pose', PoseStamped)
pose_bl = rospy.wait_for_message('/id503/aruco_single/pose', PoseStamped)
print(f'tl x={pose_tl.pose.position.x} y={pose_tl.pose.position.y}')
print(f'tr x={pose_tr.pose.position.x} y={pose_tr.pose.position.y}')
print(f'br x={pose_br.pose.position.x} y={pose_br.pose.position.y}')
print(f'bl x={pose_bl.pose.position.x} y={pose_bl.pose.position.y}')

real_points = np.float32([[pose_bl.pose.position.x, pose_bl.pose.position.y],
                         [pose_br.pose.position.x, pose_br.pose.position.y],
                         [pose_tl.pose.position.x, pose_tl.pose.position.y],
                         [pose_tr.pose.position.x, pose_tr.pose.position.y]])
sim_points = np.float32([[0, 0], [10, 0], [0, 10], [10, 10]])

### Calculate the perspective transformation matrix
matrix = cv2.getPerspectiveTransform(sim_points, real_points)

# Read the YAML data from the file for agent 1 (schedule 1)
with open("/home/zeyu/catkin_ws/src/auto_navigation/scripts/cbs_output.yaml", "r") as yaml_file:
    yaml_data = yaml.safe_load(yaml_file)

# Extract coordinates for agent 1 (schedule 1)
agent_1_schedule = yaml_data["schedule"][1]

# Initialize a list to store the coordinates for agent 1
coordinates_1 = []

# Iterate through the schedule and extract x, y values for agent 1
for item in agent_1_schedule:
    x = item["x"]
    y = item["y"]
    coordinates_1.append([x, y])

# Read the YAML data from the file for agent 2 (schedule 2)
# Replace the file path with the correct one for agent 2
with open("/home/zeyu/catkin_ws/src/auto_navigation/scripts/cbs_output.yaml", "r") as yaml_file:
    yaml_data = yaml.safe_load(yaml_file)

# Extract coordinates for agent 2 (schedule 2)
agent_2_schedule = yaml_data["schedule"][2]

# Initialize a list to store the coordinates for agent 2
coordinates_2 = []

# Iterate through the schedule and extract x, y values for agent 2
for item in agent_2_schedule:
    x = item["x"]
    y = item["y"]
    coordinates_2.append([x, y])

# Delete start points
coordinates_1.pop(0)
coordinates_2.pop(0)

state = {ids[0]: True, ids[1]: True}
lock = threading.Lock()

threads = []
t1 = threading.Thread(target=control_agent, args=(ids[0], convert_sim_to_real_poses(coordinates_1, matrix)))
threads.append(t1)
t1.start()
t2 = threading.Thread(target=control_agent, args=(ids[1], convert_sim_to_real_poses(coordinates_2, matrix)))
threads.append(t2)
t2.start()
for t in threads:
    t.join()

rospy.spin()

