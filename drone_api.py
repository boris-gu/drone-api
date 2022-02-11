import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from tf.transformations import quaternion_from_euler
from threading import Thread


class Drone_api:
    def __init__(self):
        self.arm = False
        self.current_state = State()
        self.current_pose = PoseStamped()
        self.last_command_pose = None

        rospy.init_node('drone_offb', anonymous=True)
        # Подписчик состояния, издатель локальной позиции, клиенты для смены режима и арма
        self.state_sub = rospy.Subscriber('mavros/state', State,
                                          self.state_cb, queue_size=10)
        self.local_pos_sub = rospy.Subscriber('/mavros/local_position/pose',
                                              PoseStamped, self.local_pos_cb, queue_size=10)
        self.local_pos_pub = rospy.Publisher('mavros/setpoint_position/local',
                                             PoseStamped, queue_size=10)
        self.set_mode_client = rospy.ServiceProxy('mavros/set_mode', SetMode)
        self.arming_client = rospy.ServiceProxy('mavros/cmd/arming',
                                                CommandBool)

        # Дрон не перейдет в режим OFFBOARD, пока не начнется потоковая передача значений
        thread_state = Thread(target=self.state_target, daemon=True)
        thread_state.start()
        thread_pose = Thread(target=self.pose_target, daemon=True)
        thread_pose.start()

    def state_cb(self, state):
        self.current_state = state

    def local_pos_cb(self, pose):
        self.current_pose = pose
        if not self.last_command_pose:
            self.last_command_pose = pose

    def state_target(self):
        rate = rospy.Rate(1)
        prev_state = self.current_state
        while not rospy.is_shutdown():
            try:
                if self.current_state.mode != 'OFFBOARD':
                    self.set_mode_client(base_mode=0, custom_mode='OFFBOARD')
                else:
                    if not self.current_state.armed:
                        self.arming_client(True)
                if prev_state.armed != self.current_state.armed:
                    self.arm = True
                prev_state = self.current_state
                try:
                    rate.sleep()
                except rospy.exceptions.ROSTimeMovedBackwardsException as err:
                    pass
            except rospy.ROSInterruptException:
                pass

    def pose_target(self):
        # Установка скорости публикации выше 2Гц
        rate = rospy.Rate(20)
        # Ожидаем связь между МАВРОС и автопилотом
        while not rospy.is_shutdown() and not self.current_state.connected:
            try:
                rate.sleep()
            except rospy.exceptions.ROSTimeMovedBackwardsException as err:
                pass
        while not rospy.is_shutdown():
            self.local_pos_pub.publish(self.last_command_pose)
            try:
                rate.sleep()
            except rospy.exceptions.ROSTimeMovedBackwardsException as err:
                pass

    def set_pose(self, x=None, y=None, z=None, yaw=None):
        new_pose = PoseStamped()
        if not x:
            x = self.current_pose.pose.position.x
        if not y:
            y = self.current_pose.pose.position.y
        if not z:
            z = self.current_pose.pose.position.z
        new_pose.pose.position.x = x
        new_pose.pose.position.y = y
        new_pose.pose.position.z = z
        if yaw:
            q = quaternion_from_euler(0, 0, yaw)
            new_pose.pose.orientation.x = q[0]
            new_pose.pose.orientation.y = q[1]
            new_pose.pose.orientation.z = q[2]
            new_pose.pose.orientation.w = q[3]
        self.last_command_pose = new_pose
