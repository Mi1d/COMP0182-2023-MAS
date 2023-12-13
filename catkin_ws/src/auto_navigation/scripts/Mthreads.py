#!/usr/bin/env python3

import rospy
import math
import threading
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import PoseStamped
import yaml
import numpy as np
import cv2



rospy.init_node('goal_pose')

agent_positions = {}
cmd_pubs = {}
distance_info = {}
theta_info = {}
cmd_pub_lock = threading.Lock()
distance_lock = threading.Lock()
theta_lock = threading.Lock()
positions_lock = threading.Lock()

def update_cmd_pub(agent_id, pub):
    with cmd_pub_lock:
        cmd_pubs[agent_id] = pub

def get_cmd_pub(agent_id):
    with cmd_pub_lock:
        return cmd_pubs.get(agent_id)

def update_distance(agent_id, distance):
    with distance_lock:
        distance_info[agent_id] = distance

def get_distance(agent_id):
    with distance_lock:
        return distance_info.get(agent_id)

def update_theta(agent_id, theta):
    with theta_lock:
        theta_info[agent_id] = theta

def get_theta(agent_id):
    with theta_lock:
        return theta_info.get(agent_id)

def update_position(agent_id, position):
    with positions_lock:
        agent_positions[agent_id] = position

def get_position(agent_id):
    with positions_lock:
        return agent_positions.get(agent_id)

def agent_pose_callback(msg, agent_id):
    # 更新agent的位置信息

    update_position(agent_id, (msg.pose.position.x, msg.pose.position.y))
    collision_distance=check_for_collision(agent_id)
    adjust_speed(agent_id, collision_distance)
    # 可以在这里添加碰撞检测逻辑

def check_for_collision(current_agent_id):
    min_collision_distance = float('inf') 
    current_position = get_position(current_agent_id)
    for agent_id, position in agent_positions.items():
        if agent_id != current_agent_id:
            collision_distance = math.dist(current_position, position)
    return collision_distance


def linear_modified_sigmoid(x, k=20, x0=0.15):
    return 1 / (1 + math.exp(-k * (x - x0)))

def angular_modified_sigmoid(theta, k=5):
    return 1 / (1 + math.exp(-k * theta))


def adjust_speed(agent_id,collision_distance):
    twist = Twist()
    # 根据距离计算新的速度
    k2 = 8
    linear = 3
    angular = k2 * get_theta(agent_id)
    if collision_distance <0.2:
        twist.linear.x = linear * get_distance(agent_id)* math.cos(get_theta(agent_id))*linear_modified_sigmoid(collision_distance)
    else:
        twist.linear.x = linear * get_distance(agent_id)* math.cos(get_theta(agent_id))*linear_modified_sigmoid(get_distance(agent_id))+1
    twist.angular.z = -angular*angular_modified_sigmoid(get_theta(agent_id))
    cmd_pubs[agent_id].publish(twist)


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
    topic_name = f'agent{agent_id}/cmd_vel'
    cmd_pub = rospy.Publisher(topic_name, Twist, queue_size=10)
    update_cmd_pub(agent_id, cmd_pub)

    
    rospy.Subscriber(f'/id{agent_id}/aruco_single/pose', PoseStamped, agent_pose_callback, agent_id)
    init_pose = get_position(agent_id)
    for coordinate in coordinates:
        goal_x = coordinate[0]
        goal_y = coordinate[1]

        while not check_goal_reached(init_pose, [goal_x, goal_y], 0.02):
            init_pose = get_position(agent_id)

            orientation_q = init_pose.pose.orientation
            orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
            (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
            Orientation = yaw
            dx = goal_x - init_pose.pose.position.x
            dy = goal_y - init_pose.pose.position.y
            goal_distance = math.dist([init_pose.pose.position.x, init_pose.pose.position.y], [goal_x, goal_y])
            goal_direct = math.atan2(dy, dx)

            print(f"Agent {agent_id}: current_coor", [init_pose.pose.position.x, init_pose.pose.position.y])
            print(f"Agent {agent_id}: goal_coor", [goal_x, goal_y])
            print(f"Agent {agent_id}: Orientation", Orientation)
            print(f"Agent {agent_id}: goal_direct", goal_direct)

            if (Orientation < 0):
                Orientation = Orientation + 2 * math.pi
            if (goal_direct < 0):
                goal_direct = goal_direct + 2 * math.pi

            goal_theta = goal_direct - Orientation

            if goal_theta < 0 and abs(goal_theta) > abs(goal_theta + 2 * math.pi):
                goal_theta = goal_theta + 2 * math.pi
            elif goal_theta > 0 and abs(theta - 2 * math.pi) < theta:
                goal_theta = goal_theta - 2 * math.pi

            print(f"Agent {agent_id}: theta:", goal_theta)
            update_distance(agent_id,goal_distance)
            update_theta(agent_id,goal_theta)

### Define your points in the simulation plane and the real-world plane
# """
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
# """

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
    
print(coordinates_1)

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

threads = []
t1 = threading.Thread(target=control_agent, args=(321, convert_sim_to_real_poses(coordinates_1, matrix)))
threads.append(t1)
t1.start()
t2 = threading.Thread(target=control_agent, args=(325, convert_sim_to_real_poses(coordinates_2, matrix)))
threads.append(t2)
t2.start()
for t in threads:
    t.join()

# Control Agent 1
# control_agent(init_pose_1, coordinates_1, scale)

# Control Agent 2
# You may want to set the initial pose for Agent 2 if it's different from Agent 1
# control_agent(init_pose_for_agent_2, coordinates_2, scale)

rospy.spin()
