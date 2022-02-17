import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from tf.transformations import quaternion_from_euler
from threading import Thread
from math import pi, atan2, sqrt


class Drone_api:
    def __init__(self):
        self.__started = False
        self.__type_of_move = None  # 'POSE' or 'SPEED'
        self.__yaw_head_first = False
        self.__allowable_error = 0.5
        self.__last_command_pose = None
        self.__thread_command = None

        self.current_state = State()
        self.current_pose = PoseStamped()

        rospy.init_node('drone_offb', anonymous=True)
        # get state, position, set position, speeds, mode and arm
        self.__state_sub = rospy.Subscriber('mavros/state', State,
                                            self.__state_cb, queue_size=10)
        self.__local_pos_sub = rospy.Subscriber('/mavros/local_position/pose',
                                                PoseStamped, self.__local_pos_cb, queue_size=10)
        self.__local_pos_pub = rospy.Publisher('mavros/setpoint_position/local',
                                               PoseStamped, queue_size=10)
        self.__set_mode_client = rospy.ServiceProxy('mavros/set_mode', SetMode)
        self.__arming_client = rospy.ServiceProxy('mavros/cmd/arming',
                                                  CommandBool)

    def __state_cb(self, state):
        self.current_state = state

    def __local_pos_cb(self, pose):
        self.current_pose = pose

    def __command_target(self):
        rate = rospy.Rate(20)
        # Waiting for communication between MAVROS and autopilot
        while not rospy.is_shutdown() and not self.current_state.connected:
            try:
                rate.sleep()
            except rospy.exceptions.ROSTimeMovedBackwardsException:
                pass
        last_request = rospy.Time.now()
        while self.__started and not rospy.is_shutdown():
            try:
                now = rospy.Time.now()
                # OFFBOARD and ARM
                if now - last_request > rospy.Duration(3):
                    last_request = now
                    if self.current_state.mode != 'OFFBOARD':
                        self.__set_mode_client(base_mode=0,
                                               custom_mode='OFFBOARD')
                    elif not self.current_state.armed:
                        # fixme: дрон бесконечно дизармится, если не взлететь
                        self.__arming_client(True)
                # CONTROL
                if self.__type_of_move == 'POSE':
                    delta_x = self.__last_command_pose.pose.position.x - \
                        self.current_pose.pose.position.x
                    delta_y = self.__last_command_pose.pose.position.y - \
                        self.current_pose.pose.position.y
                    distance = sqrt((delta_x)**2 + (delta_y)**2)
                    if self.__yaw_head_first and distance > self.__allowable_error:
                        yaw_rad = atan2(delta_y, delta_x)
                        q = quaternion_from_euler(0, 0, yaw_rad)
                        self.__last_command_pose.pose.orientation.x = q[0]
                        self.__last_command_pose.pose.orientation.y = q[1]
                        self.__last_command_pose.pose.orientation.z = q[2]
                        self.__last_command_pose.pose.orientation.w = q[3]
                    self.__local_pos_pub.publish(self.__last_command_pose)
                elif self.__type_of_move == 'SPEED':
                    pass  # todo: написать управление скоростью
                rate.sleep()
            except rospy.exceptions.ROSException:
                pass

    def start(self):
        if not self.__started:
            self.__started = True
            self.__type_of_move = 'POSE'
            self.__last_command_pose = PoseStamped()
            self.__last_command_pose.pose.position.z = -3  # started but didn't takeoff

            if self.__thread_command is None or not self.__thread_command.is_alive():
                self.__thread_command = Thread(target=self.__command_target,
                                               daemon=True)
                self.__thread_command.start()

    def stop(self):
        if self.__started:
            self.__started = False

    def sleep(self, time):
        try:
            rospy.sleep(time)
        except rospy.ROSInterruptException:
            pass

    def is_shutdown(self):
        return rospy.is_shutdown()

    def set_pose(self, x=None, y=None, z=None, yaw=None, yaw_head_first=False):
        new_pose = PoseStamped()
        if x is None:
            x = self.__last_command_pose.pose.position.x
        if y is None:
            y = self.__last_command_pose.pose.position.y
        if z is None:
            z = self.__last_command_pose.pose.position.z
        new_pose.pose.position.x = x
        new_pose.pose.position.y = y
        new_pose.pose.position.z = z
        if yaw is None:
            new_pose.pose.orientation = self.__last_command_pose.pose.orientation
        else:
            yaw_rad = -yaw * (pi/180)
            q = quaternion_from_euler(0, 0, yaw_rad)
            new_pose.pose.orientation.x = q[0]
            new_pose.pose.orientation.y = q[1]
            new_pose.pose.orientation.z = q[2]
            new_pose.pose.orientation.w = q[3]
        self.__last_command_pose = new_pose
        self.__yaw_head_first = yaw_head_first
        self.__type_of_move = "POSE"
