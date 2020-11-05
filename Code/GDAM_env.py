import rospy
from numpy import inf
import subprocess
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Twist
from pyquaternion import Quaternion
from collections import deque
import numpy as np
import math
from statistics import stdev
import tf as tranf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from sensor_msgs.msg import Image, LaserScan
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from nav_msgs.msg import Odometry, Path
import os


def rotateneg(point, angle):
    x, y = point
    xx = x * math.cos(angle) + y * math.sin(angle)
    yy = -x * math.sin(angle) + y * math.cos(angle)
    return xx, yy


def rotatepos(point, angle):
    x, y = point
    xx = x * math.cos(angle) - y * math.sin(angle)
    yy = x * math.sin(angle) + y * math.cos(angle)
    return xx, yy


def calcqxqy(dist, angl, ang):
    angl = math.radians(angl)
    angle = angl + ang
    if angle > np.pi:
        angle = np.pi - angle
        angle = -np.pi - angle
    if angle < -np.pi:
        angle = -np.pi - angle
        angle = np.pi - angle
    if angle > 0:
        qx, qy = rotatepos([dist, 0], angle)
    else:
        qx, qy = rotateneg([dist, 0], -angle)
    return qx, qy


class ImplementEnv:
    def __init__(self, args):

        self.countZero = 0
        self.count_turn = -100

        self.node_vicinity = args.node_vicinity
        self.del_node_vicinity = args.deleted_node_vicinity
        self.min_in = args.min_in
        self.side_min_in = args.side_min_in
        self.del_nodes_range = args.delete_nodes_range
        launchfile = args.launchfile
        self.accelPos_low = args.acceleration_low
        self.accelPos_high = args.acceleration_high
        self.accelNeg_low = args.deceleration_low
        self.accelNeg_high = args.deceleration_high
        self.angPos = args.angular_acceleration
        self.original_goal_x = args.x
        self.original_goal_y = args.y
        nr_nodes = args.nr_of_nodes
        nr_closed_nodes = args.nr_of_closed_nodes
        nr_deleted_nodes = args.nr_of_deleted_nodes
        self.update_rate = args.update_rate
        self.remove_rate = args.remove_rate
        self.stddev_threshold = args.stddev_threshold
        self.freeze_rate = args.freeze_rate

        self.accelNeg = 0

        self.angle = 0
        self.odomX = 0
        self.odomY = 0
        self.linearLast = 0
        self.angularLast = 0

        self.LaserData = None
        self.LaserDataTop = None
        self.OdomData = None

        self.lock = 0

        self.PathData = None

        self.last_statesX = deque(maxlen=self.freeze_rate)
        self.last_statesY = deque(maxlen=self.freeze_rate)
        self.last_statesX.append(0)
        self.last_statesY.append(0)

        self.last_states = False

        self.global_goal_x = self.original_goal_x
        self.global_goal_y = self.original_goal_y

        self.nodes = deque(maxlen=nr_nodes)
        self.nodes.append([4.5, 0, 4.5, 0, 0])
        self.nodes.append([self.global_goal_x, self.global_goal_y, self.global_goal_x, self.global_goal_y, 0])

        self.closed_nodes = deque(maxlen=nr_closed_nodes)
        self.closed_nodes.append([0, 0])
        self.closed_nodes_rotated = deque(maxlen=nr_closed_nodes)

        self.map_nodes = deque(maxlen=400)

        self.deleted_nodes = deque(maxlen=nr_deleted_nodes)
        self.deleted_nodes_rotated = deque(maxlen=nr_deleted_nodes)

        self.goalX = self.nodes[0][2]
        self.goalY = self.nodes[0][3]

        self.g_node = 0

        port = os.environ["ROS_PORT_SIM"]

        rospy.init_node('gym', anonymous=True)
        if launchfile.startswith("/"):
            fullpath = launchfile
        else:
            fullpath = os.path.join(os.path.dirname(__file__), "assets", "launch", launchfile)
        if not os.path.exists(fullpath):
            raise IOError("File " + fullpath + " does not exist")

        subprocess.Popen(["roslaunch", "-p", port, fullpath])

        self.vel_pub = rospy.Publisher('/r1/cmd_vel', Twist, queue_size=1)
        self.vel_pub_rob = rospy.Publisher('/RosAria/cmd_vel', Twist, queue_size=1)
        topic_nodes_pub = 'vis_mark_array_node'
        self.nodes_pub = rospy.Publisher(topic_nodes_pub, Marker, queue_size=3)
        topic_nodes_pub_closed = 'vis_mark_array_node_closed'
        self.nodes_pub_closed = rospy.Publisher(topic_nodes_pub_closed, Marker, queue_size=3)
        topic_map_nodes = 'vis_mark_array_map_nodes'
        self.map_nodes_viz = rospy.Publisher(topic_map_nodes, Marker, queue_size=3)
        topic = 'vis_mark_array'
        self.publisher = rospy.Publisher(topic, MarkerArray, queue_size=3)
        self.global_goal_publisher = rospy.Publisher('global_goal_publisher', MarkerArray, queue_size=1)
        self.navLaser = rospy.Subscriber('/scan', LaserScan, self.Laser_callback, queue_size=1)
        self.navLaserTop = rospy.Subscriber('/rpTop/scan', LaserScan, self.Laser_callback_Top, queue_size=1)
        self.navOdom = rospy.Subscriber('/RosAria/pose', Odometry, self.Odom_callback, queue_size=1)
        self.path = rospy.Subscriber('/move_base/TrajectoryPlannerROS/global_plan', Path, self.path_callback,
                                     queue_size=1)
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        self.listener = tranf.TransformListener()

    def Laser_callback(self, l):
        self.LaserData = l

    def Laser_callback_Top(self, lt):
        self.LaserDataTop = lt

    def Odom_callback(self, o):
        self.OdomData = o

    def path_callback(self, p):
        self.PathData = p

    def step(self, act):
        rplidar = None
        rplidarTop = None
        self.map_nodes.clear()
        while rplidar is None:
            try:
                rplidar = self.LaserData
                rplidarTop = self.LaserDataTop
            except:
                print("Laser scans didn't work")
                pass
        dataOdom = None
        while dataOdom is None:
            try:
                dataOdom = self.OdomData
            except:
                print("Odometry not available")
                pass

        rpleft = np.array(rplidar.ranges[1440 - 360:1440])
        rpright = np.array(rplidar.ranges[0:360])
        rp_state = np.append(rpleft, rpright)
        laser_in = rp_state.copy()
        laser_in[laser_in == inf] = 10
        laser_in[laser_in > 10] = 10

        rpleftTop = np.array(rplidarTop.ranges[1440 - 360:1440])
        rprightTop = np.array(rplidarTop.ranges[0:360])
        rp_stateTop = np.append(rpleftTop, rprightTop)
        laser_in_top = rp_stateTop.copy()
        laser_in_top[laser_in_top == inf] = 10
        laser_in_top[laser_in_top > 10] = 10

        for l in range(len(laser_in)):
            laser_in[l] = min(laser_in[l], laser_in_top[l])

        laser_state = np.array(
            [min(laser_in[0:20]), min(laser_in[21:60]),
             min(laser_in[61:100]), min(laser_in[101:140]),
             min(laser_in[141:180]), min(laser_in[181:220]),
             min(laser_in[221:260]), min(laser_in[260:300]),
             min(laser_in[301:340]), min(laser_in[341:380]),
             min(laser_in[381:420]), min(laser_in[421:460]),
             min(laser_in[461:500]), min(laser_in[501:540]),
             min(laser_in[541:580]), min(laser_in[581:620]),
             min(laser_in[621:660]), min(laser_in[661:700]),
             min(laser_in[701:719])])
        laser_state = laser_state / 10

        col, colleft, colright, minleft, minright = self.laser_check(laser_in)

        self.odomX = dataOdom.pose.pose.position.x
        self.odomY = dataOdom.pose.pose.position.y
        quaternion = (
            dataOdom.pose.pose.orientation.x,
            dataOdom.pose.pose.orientation.y,
            dataOdom.pose.pose.orientation.z,
            dataOdom.pose.pose.orientation.w)
        euler = euler_from_quaternion(quaternion)
        self.angle = round(euler[2], 4)

        (trans, rot) = self.listener.lookupTransform('/map', '/odom', rospy.Time(0))

        local_x = self.original_goal_x - trans[0]
        local_y = self.original_goal_y - trans[1]
        eul = euler_from_quaternion(rot)
        q8c = Quaternion(axis=[0.0, 0.0, 1.0], radians=-eul[2])
        q9c = Quaternion(axis=[0.0, 0.0, 1.0], radians=eul[2])
        p_tmp = q8c.rotate([local_x, local_y, 0])
        self.global_goal_x = p_tmp[0]
        self.global_goal_y = p_tmp[1]

        global_goal_distance = math.sqrt((self.odomX - self.global_goal_x)**2 + (self.odomY - self.global_goal_y)**2)
        if global_goal_distance < 1.5:
            print("Arrived at the goal")
            while True:
                pass

        self.new_nodes(laser_in, self.odomX, self.odomY, self.angle, trans, q9c)
        self.free_space_nodes(laser_in, self.odomX, self.odomY, self.angle, trans, q9c)
        self.infinite_nodes(laser_in, self.odomX, self.odomY, self.angle, trans, q9c)

        if not self.nodes:
            vel_cmd = Twist()
            vel_cmd.linear.x = 0
            vel_cmd.angular.z = 0.1
            self.vel_pub_rob.publish(vel_cmd)
            print("Looking for nodes")
            return laser_state, [0, 0]

        for i_node in range(len(self.nodes)):
            if self.nodes[i_node][4] == 0:
                node = [self.nodes[i_node][0], self.nodes[i_node][1]]
                local_x = node[0] - trans[0]
                local_y = node[1] - trans[1]
                p_tmp = q8c.rotate([local_x, local_y, 0])
                self.nodes[i_node][2] = p_tmp[0]
                self.nodes[i_node][3] = p_tmp[1]

        self.closed_nodes_rotated.clear()
        for i_node in range(len(self.closed_nodes)):
            node = self.closed_nodes[i_node]

            local_x = node[0] - trans[0]
            local_y = node[1] - trans[1]
            p_tmp = q8c.rotate([local_x, local_y, 0])
            self.closed_nodes_rotated.append([p_tmp[0], p_tmp[1]])

        self.deleted_nodes_rotated.clear()
        for i_node in range(len(self.deleted_nodes)):
            node = self.deleted_nodes[i_node]
            local_x = node[0] - trans[0]
            local_y = node[1] - trans[1]
            p_tmp = q8c.rotate([local_x, local_y, 0])
            self.deleted_nodes_rotated.append([p_tmp[0], p_tmp[1]])

        if not self.last_states:
            self.last_states = self.freeze(self.odomX, self.odomY)
            self.count_turn = 8

        self.goalX = self.nodes[self.g_node][2]
        self.goalY = self.nodes[self.g_node][3]

        Dist = math.sqrt(math.pow(self.odomX - self.goalX, 2) + math.pow(self.odomY - self.goalY, 2))

        skewX = self.goalX - self.odomX
        skewY = self.goalY - self.odomY

        dot = skewX * 1 + skewY * 0
        mag1 = math.sqrt(math.pow(skewX, 2) + math.pow(skewY, 2))
        mag2 = math.sqrt(math.pow(1, 2) + math.pow(0, 2))
        beta = math.acos(dot / (mag1 * mag2))

        if skewY < 0:
            if skewX < 0:
                beta = -beta
            else:
                beta = 0 - beta
        beta2 = (beta - self.angle)

        if beta2 > np.pi:
            beta2 = np.pi - beta2
            beta2 = -np.pi - beta2
        if beta2 < -np.pi:
            beta2 = -np.pi - beta2
            beta2 = np.pi - beta2

        linear = act[0]
        angular = act[1]

        linear, angular = self.recover(linear, angular, minleft, minright, colleft, colright, col)

        vel_cmd = Twist()
        vel_cmd.linear.x = linear
        if abs(angular) < 0.3:
            if angular > 0:
                vel_cmd.angular.z = (angular ** 2) / 0.3
            else:
                vel_cmd.angular.z = -(angular ** 2) / 0.3
        else:
            vel_cmd.angular.z = angular

        self.vel_pub_rob.publish(vel_cmd)

        self.linearLast = linear
        self.angularLast = angular

        sphere_list = Marker()
        sphere_list.header.frame_id = "odom"
        sphere_list.type = sphere_list.SPHERE_LIST
        sphere_list.action = sphere_list.ADD
        sphere_list.scale.x = 0.3
        sphere_list.scale.y = 0.1
        sphere_list.scale.z = 0.01
        sphere_list.color.a = 1.0
        sphere_list.color.r = 0.0
        sphere_list.color.g = 0.0
        sphere_list.color.b = 1.0
        sphere_list.pose.orientation.w = 1.0
        for i_node in range(len(self.nodes)):
            p = Point()
            node = self.nodes[i_node]
            p.x = node[2]
            p.y = node[3]
            p.z = 0
            sphere_list.points.append(p)
        self.nodes_pub.publish(sphere_list)

        closed_list = Marker()
        closed_list.header.frame_id = "odom"
        closed_list.type = closed_list.SPHERE_LIST
        closed_list.action = closed_list.ADD
        closed_list.scale.x = 0.15
        closed_list.scale.y = 0.1
        closed_list.scale.z = 0.01
        closed_list.color.a = 1.0
        closed_list.color.r = 1.0
        closed_list.color.g = 0.5
        closed_list.color.b = 0.5
        closed_list.pose.orientation.w = 1.0
        for i_node in range(len(self.closed_nodes_rotated)):
            node = self.closed_nodes_rotated[i_node]
            p = Point()
            p.x = node[0]
            p.y = node[1]
            p.z = 0
            closed_list.points.append(p)
        self.nodes_pub_closed.publish(closed_list)

        map_list = Marker()
        map_list.header.frame_id = "odom"
        map_list.type = map_list.SPHERE_LIST
        map_list.action = map_list.ADD
        map_list.scale.x = 0.25
        map_list.scale.y = 0.1
        map_list.scale.z = 0.01
        map_list.color.a = 1.0
        map_list.color.r = 1.0
        map_list.color.g = 0.0
        map_list.color.b = 0.0
        map_list.pose.orientation.w = 1.0

        for i_node in range(len(self.map_nodes)):
            p = Point()
            node = self.map_nodes[i_node]
            p.x = node[0]
            p.y = node[1]
            p.z = 0
            map_list.points.append(p)
        self.map_nodes_viz.publish(map_list)

        markerArray = MarkerArray()
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.type = marker.CYLINDER
        marker.action = marker.ADD
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.01
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = self.goalX
        marker.pose.position.y = self.goalY
        marker.pose.position.z = 0

        markerArray.markers.append(marker)
        self.publisher.publish(markerArray)

        markerArrayGoal = MarkerArray()
        markerGoal = Marker()
        markerGoal.header.frame_id = "odom"
        markerGoal.type = marker.CYLINDER
        markerGoal.action = marker.ADD
        markerGoal.scale.x = 0.6
        markerGoal.scale.y = 0.6
        markerGoal.scale.z = 0.01
        markerGoal.color.a = 1.0
        markerGoal.color.r = 0.1
        markerGoal.color.g = 0.9
        markerGoal.color.b = 0.0
        markerGoal.pose.orientation.w = 1.0
        markerGoal.pose.position.x = self.global_goal_x
        markerGoal.pose.position.y = self.global_goal_y
        markerGoal.pose.position.z = 0

        markerArrayGoal.markers.append(markerGoal)
        self.global_goal_publisher.publish(markerArrayGoal)

        if Dist < 1.0:
            self.change_goal()
            self.countZero = 0

        if Dist > 5.0:
            p_data = self.PathData

            try:
                path_len = len(p_data.poses) - 1
            except:
                path_len = 0

            c_p = 0
            while c_p <= path_len and path_len > 0:
                plan_x = p_data.poses[c_p].pose.position.x
                plan_y = p_data.poses[c_p].pose.position.y
                node = [plan_x, plan_y]
                local_x = node[0] - trans[0]
                local_y = node[1] - trans[1]
                p_tmp = q8c.rotate([local_x, local_y, 0])

                d = math.sqrt(math.pow(self.odomX - p_tmp[0], 2) + math.pow(self.odomY - p_tmp[1], 2))
                if d > 4.0 or (c_p == path_len and d>1.5):
                    f = True
                    for j in range(len(self.nodes)):
                        check_d = math.sqrt(
                            math.pow(self.nodes[j][2] - p_tmp[0], 2) + math.pow(self.nodes[j][3] - p_tmp[1], 2))
                        if check_d < 1.0:
                            self.g_node = j
                            f = False
                            break
                    if f:
                        self.nodes.append([p_data.poses[c_p].pose.position.x, p_data.poses[c_p].pose.position.y,
                                           p_data.poses[c_p].pose.position.x, p_data.poses[c_p].pose.position.y, 0])
                        self.g_node = len(self.nodes) - 1
                    break
                c_p += 1

        self.check_pos(self.odomX, self.odomY)

        if self.countZero % 30 == 0:
            f = True
            for j in range(len(self.closed_nodes_rotated)):
                d = math.sqrt(
                    math.pow(self.closed_nodes_rotated[j][0] - self.odomX, 2) + math.pow(
                        self.closed_nodes_rotated[j][1] - self.odomY, 2))
                if d < 0.85:
                    f = False
                    break
            if f:
                node = [self.odomX, self.odomY]
                p_tmp = q9c.rotate([node[0], node[1], 0])
                self.closed_nodes.append([p_tmp[0] + trans[0], p_tmp[1] + trans[1]])

        if self.countZero % self.update_rate == 0:
            self.check_goal()

        if self.countZero > self.remove_rate:
            self.change_goal()
            self.countZero = 0

        self.countZero += 1
        Dist = min(5.0, Dist)
        toGoal = [Dist / 10, (beta2 + np.pi) / (np.pi*2)]
        return laser_state, toGoal

    def recover(self, linear, angular, minleft, minright, colleft=False, colright=False, col=False):

        if colleft and colright:
            angular = 0
            linear = 0.3

        if col:
            if self.lock == 0:
                if minright - minleft > 0:
                    self.lock = 1
                else:
                    self.lock = -1
            angular = self.lock
            linear = 0
        else:
            self.lock = 0

        if self.last_states:
            if col:
                angular = 0.7
                linear = 0
            else:
                angular = 0
                linear = 0.35
                self.count_turn -= 1
            if self.count_turn < 0:
                self.last_states = False

        if linear > self.linearLast:
            self.accelNeg = 0
            if self.linearLast > 0.25:
                linear = min(self.linearLast + self.accelPos_low, linear)
            else:
                linear = min(self.linearLast + self.accelPos_high, linear)
        if linear < self.linearLast:
            if self.linearLast > 0.25:
                self.accelNeg += self.accelNeg_low
                linear = max(self.linearLast - self.accelNeg, linear)
            else:
                self.accelNeg += self.accelNeg_high
                linear = max(self.linearLast - self.accelNeg, linear)
                
        if self.angularLast < angular:
            angular = min(self.angularLast + self.angPos, angular)
        if self.angularLast > angular:
            angular = max(self.angularLast - self.angPos, angular)

        return linear, angular

    def heuristic(self, odomX, odomY, candidateX, candidateY):
        forward = False
        to_goal = False
        to_goal2 = True
        gX = self.global_goal_x
        gY = self.global_goal_y
        d = 0
        if forward:
            d = math.sqrt(math.pow(candidateX - odomX, 2) + math.pow(candidateY - odomY, 2))
            skewX = candidateX - odomX
            skewY = candidateY - odomY
            dot = skewX * 1 + skewY * 0
            mag1 = math.sqrt(math.pow(skewX, 2) + math.pow(skewY, 2))
            mag2 = math.sqrt(math.pow(1, 2) + math.pow(0, 2))
            beta = math.acos(dot / (mag1 * mag2))
            if skewY < 0:
                if skewX < 0:
                    beta = -beta
                else:
                    beta = 0 - beta
            beta2 = (beta - self.angle)
            if beta2 > np.pi:
                beta2 = np.pi - beta2
                beta2 = -np.pi - beta2
            if beta2 < -np.pi:
                beta2 = -np.pi - beta2
                beta2 = np.pi - beta2
            d = d + abs(beta2)

        if to_goal:
            d1 = math.sqrt(math.pow(candidateX - odomX, 2) + math.pow(candidateY - odomY, 2))
            d2 = math.sqrt(math.pow(candidateX - gX, 2) + math.pow(candidateY - gY, 2))
            d = d1 * 1.3 + d2 * 0.7

        if to_goal2:
            d1 = math.sqrt((candidateX - odomX) ** 2 + (candidateY - odomY) ** 2)
            d2 = math.sqrt((candidateX - gX) ** 2 + (candidateY - gY) ** 2)
            if 5 < d1 < 10:
                d1 = 5
            if d1 < 5:
                d1 = 0
            d = d1 + d2
        return d

    def change_goal(self):
        self.nodes.remove(self.nodes[self.g_node])
        self.check_goal()

    def check_goal(self):
        min_d = math.sqrt(math.pow(self.nodes[0][2] - self.odomX, 2) + math.pow(self.nodes[0][3] - self.odomY, 2))
        node_out = 0

        for i in range(len(self.nodes)):
            d = self.heuristic(self.odomX, self.odomY, self.nodes[i][2], self.nodes[i][3])
            if d < min_d:
                min_d = d
                node_out = i

        self.g_node = node_out
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "odom"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = self.nodes[self.g_node][2]
        goal.target_pose.pose.position.y = self.nodes[self.g_node][3]
        goal.target_pose.pose.orientation.w = 1.0
        self.client.send_goal(goal)

    def new_nodes(self, laser, odomX, odomY, ang, trans, q9c):

        for i in range(1, len(laser)):
            if len(self.nodes) > 0 and laser[i] < 7.5:
                dist = laser[i]
                angl = i / 4 - 90
                qx, qy = calcqxqy(dist, angl, ang)

                self.map_nodes.append([qx + odomX, qy + odomY])

                for j in range(len(self.nodes) - 1, -1, -1):
                    d = math.sqrt(
                        math.pow(self.nodes[j][2] - qx - odomX, 2) + math.pow(self.nodes[j][3] - qy - odomY, 2))
                    if d < self.del_nodes_range:
                        node = [self.nodes[j][2], self.nodes[j][3]]
                        p_tmp = q9c.rotate([node[0], node[1], 0])
                        self.deleted_nodes.append([p_tmp[0] + trans[0], p_tmp[1] + trans[1]])
                        self.nodes.remove(self.nodes[j])
                        self.check_goal()
                        break

            if abs(laser[i - 1] - laser[i]) > 1.5 and laser[i - 1] < 8.5 and laser[i] < 8.5:
                dist = (laser[i - 1] + laser[i]) / 2
                angl = i / (2 * 2) - 90
                qx, qy = calcqxqy(dist, angl, ang)

                f = True
                j = 0
                while j < len(self.nodes):
                    d = math.sqrt(math.pow(self.nodes[j][2] - qx - odomX, 2) +
                                  math.pow(self.nodes[j][3] - qy - odomY, 2))
                    if d < self.node_vicinity:
                        f = False
                        break
                    j += 1
                j = 0
                while f and j < len(self.closed_nodes_rotated):
                    d = math.sqrt(math.pow(self.closed_nodes_rotated[j][0] - qx - odomX, 2) +
                                  math.pow(self.closed_nodes_rotated[j][1] - qy - odomY, 2))
                    if d < self.node_vicinity:
                        f = False
                        break
                    j += 1
                j = 0
                while f and j < len(self.deleted_nodes_rotated):
                    d = math.sqrt(math.pow(self.deleted_nodes_rotated[j][0] - qx - odomX, 2) +
                                  math.pow(self.deleted_nodes_rotated[j][1] - qy - odomY, 2))
                    if d < self.del_node_vicinity:
                        f = False
                        break
                    j += 1

                if f:
                    node = [qx + odomX, qy + odomY]
                    local_x = node[0]
                    local_y = node[1]
                    p_tmp = q9c.rotate([local_x, local_y, 0])
                    self.nodes.append(
                        [p_tmp[0] + trans[0], p_tmp[1] + trans[1], p_tmp[0] + trans[0], p_tmp[1] + trans[1], 0])

    def free_space_nodes(self, laser, odomX, odomY, ang, trans, q9c):
        count5 = 0
        min_d = 100

        for i in range(1, len(laser)):
            go = False

            if 4.5 < laser[i] < 9.9:
                count5 += 1
                min_d = min(min_d, laser[i])
                continue

            if count5 > 35:
                go = True
            else:
                count5 = 0

            if go:
                dist = 4
                angl = (i - count5 / 2) / (2 * 2) - 90
                count5 = 0
                min_d = 100
                qx, qy = calcqxqy(dist, angl, ang)

                f = True
                j = 0
                while j < len(self.nodes):
                    d = math.sqrt(math.pow(self.nodes[j][2] - qx - odomX, 2) +
                                  math.pow(self.nodes[j][3] - qy - odomY, 2))
                    if d < self.node_vicinity:
                        f = False
                        break
                    j += 1
                j = 0
                while f and j < len(self.closed_nodes_rotated):
                    d = math.sqrt(math.pow(self.closed_nodes_rotated[j][0] - qx - odomX, 2) +
                                  math.pow(self.closed_nodes_rotated[j][1] - qy - odomY, 2))
                    if d < self.node_vicinity:
                        f = False
                        break
                    j += 1
                j = 0
                while f and j < len(self.deleted_nodes_rotated):
                    d = math.sqrt(math.pow(self.deleted_nodes_rotated[j][0] - qx - odomX, 2) +
                                  math.pow(self.deleted_nodes_rotated[j][1] - qy - odomY, 2))
                    if d < self.del_node_vicinity:
                        f = False
                        break
                    j += 1

                if f:
                    node = [qx + odomX, qy + odomY]
                    p_tmp = q9c.rotate([node[0], node[1], 0])
                    self.nodes.append(
                        [p_tmp[0] + trans[0], p_tmp[1] + trans[1], p_tmp[0] + trans[0], p_tmp[1] + trans[1], 0])

    def infinite_nodes(self, laser, odomX, odomY, ang, trans, q9c):
        tmp_i = 0
        save_i = 0

        for i in range(1, len(laser)):
            go = False

            if laser[i] < 6.9:
                if i - tmp_i > 50 and tmp_i > 0:
                    go = True
                    save_i = tmp_i
                tmp_i = i

            if go:

                dist = min(laser[save_i], laser[i])
                angl = (i - (i - save_i) / 2) / (2 * 2) - 90
                qx, qy = calcqxqy(dist, angl, ang)

                f = True
                j = 0
                while j < len(self.nodes):
                    d = math.sqrt(math.pow(self.nodes[j][2] - qx - odomX, 2) +
                                  math.pow(self.nodes[j][3] - qy - odomY, 2))
                    if d < self.node_vicinity:
                        f = False
                        break
                    j += 1
                j = 0
                while f and j < len(self.closed_nodes_rotated):
                    d = math.sqrt(math.pow(self.closed_nodes_rotated[j][0] - qx - odomX, 2) +
                                  math.pow(self.closed_nodes_rotated[j][1] - qy - odomY, 2))
                    if d < self.node_vicinity:
                        f = False
                        break
                    j += 1
                j = 0
                while f and j < len(self.deleted_nodes_rotated):
                    d = math.sqrt(math.pow(self.deleted_nodes_rotated[j][0] - qx - odomX, 2) +
                                  math.pow(self.deleted_nodes_rotated[j][1] - qy - odomY, 2))
                    if d < self.del_node_vicinity:
                        f = False
                        break
                    j += 1

                if f:
                    node = [qx + odomX, qy + odomY]
                    p_tmp = q9c.rotate([node[0], node[1], 0])
                    self.nodes.append(
                        [p_tmp[0] + trans[0], p_tmp[1] + trans[1], p_tmp[0] + trans[0], p_tmp[1] + trans[1], 0])

    def check_pos(self, odomX, odomY):
        for i in range(len(self.nodes)):
            d = math.sqrt(math.pow(self.nodes[i][2] - odomX, 2) + math.pow(self.nodes[i][3] - odomY, 2))
            if d < 0.75:
                self.nodes.remove(self.nodes[i])
                self.check_goal()
                break

    def freeze(self, X, Y):
        self.last_statesX.append(X)
        self.last_statesY.append(Y)
        if len(self.last_statesX) > (self.freeze_rate-50) and stdev(self.last_statesX) < self.stddev_threshold and\
                stdev(self.last_statesY) < self.stddev_threshold:
            return True
        return False

    def laser_check(self, lscan, col=False, colleft=False, colright=False, minleft=7, minright=7):
        min_in = self.min_in
        side_min_in = self.side_min_in

        for i in range(0, len(lscan)):
            if i < len(lscan) / 2:
                minleft = min(minleft, lscan[i])
            else:
                minright = min(minright, lscan[i])

            if len(lscan) / 4.5 < i < len(lscan) / 3.5 and lscan[i] < side_min_in:
                colleft = True
            if len(lscan) - len(lscan) / 4.5 > i > len(lscan) - len(lscan) / 3.5 and lscan[i] < side_min_in:
                colright = True

            if len(lscan) / 7 < i < len(lscan) - len(lscan) / 7 and lscan[i] < min_in:
                col = True

        return col, colleft, colright, minleft, minright
