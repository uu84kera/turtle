#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node

from turtlesim.msg import Pose

from functools import partial
from geometry_msgs.msg import Twist
from robot_interfaces.msg import Turtle, TurtleArray
from robot_interfaces.srv import CatchTurtle
from std_srvs.srv import Empty
from turtlesim.srv import TeleportAbsolute

from geometry_msgs.msg import TransformStamped

from tf2_ros import TransformBroadcaster, TransformException, LookupException
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
import numpy as np

# some computation from:https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Broadcaster-Py.html 
# Utility Function
def quaternion_from_euler(ai, aj, ak):
    """
    Converts Euler angles (roll, pitch, yaw) to a quaternion, 
    which is a common way to represent orientation in 3D space in ROS2.
    """
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk
    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss
    return q
######---------------------------------------------------------------#####

# 'TurtleControllerNode' Class
class TurtleControllerNode(Node): 
    # Initialization
    def __init__(self):
        super().__init__("turtle_controller")
        self.pose_ = None
        self.turtle_to_catch_ = None

        # creating a flag for controller behavior. True: catch closest turtle, False: catch first turtle
        self.declare_parameter("turtle_master_name", "turtle1")
        self.turtle_master_name_ = self.get_parameter("turtle_master_name").value
        self.declare_parameter("flag", True)
        self.flag_ = self.get_parameter("flag").value

        # creating subscriber = msg type, topic, callback function, queue size
        self.pose_subscriber_ = self.create_subscription(Pose,(self.turtle_master_name_+"/pose"),self.callback_turtle_pose,10)
        self.cmd_vel_publisher_ = self.create_publisher(Twist, (self.turtle_master_name_+"/cmd_vel"), 10)
        self.alive_turtles_subscriber = self.create_subscription(TurtleArray, "alive_turtles", self.callback_alive_turtles, 10)
        self.queue_turtles_subscriber = self.create_subscription(TurtleArray, "queue_turtles", self.callback_queue_turtles, 10)
        
        self.turtle_to_queue_ = []
       
        # turtles to queue
        self.turtle_to_queue_cmd_vel_publisher = []
        self.queue_turtle_pose_subscriber = []

        # Transform Setup
        self.subscription = self.create_subscription(Pose, "/{}/pose".format(self.turtle_master_name_), partial(self.handle_turtle_pose, self.turtle_master_name_), 1)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # turtle master 
        self.get_logger().info("Turtle Master has been connected")
        # Creates a timer to run the control loop every 0.1 seconds
        self.control_loop_timer_ = self.create_timer(0.1, self.control_loop)


######---------------------------------------------------------------#####
    # Handling Transformations
    def handle_turtle_pose(self, turtle_name, msg):
        """
        Receives the pose of a turtle and broadcasts its transformation, 
        allowing other parts of the system to understand the turtle's position and orientation in the `world` frame.
        """
        t = TransformStamped()
        # Read message content and assign it to
        # corresponding tf variables
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = turtle_name
        # Turtle only exists in 2D, thus we get x and y translation
        # coordinates from the message and set the z coordinate to 0
        t.transform.translation.x = msg.x
        t.transform.translation.y = msg.y
        t.transform.translation.z = 0.0
        # For the same reason, turtle can only rotate around one axis
        # and this why we set rotation in x and y to 0 and obtain
        # rotation in z axis from the message
        q = quaternion_from_euler(0, 0, msg.theta)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        # Send the transformation
        self.tf_broadcaster.sendTransform(t)


    # Pose Callback
    def callback_turtle_pose(self, msg):
        # msg type: Pose
        """
        Updates the turtle's pose when a new `Pose` message is received.
        """
        self.pose_ = msg


    # Alive Turtles Callback
    def callback_alive_turtles(self, msg):
        # msg type: TurtleArray
        """
        This function is used to detect which turtle should be caught
        If `flag` is false, it catches the first turtle; if true, it catches the closest turtle.
        """
        if len(msg.turtles) > 0:
            if self.flag_ == False:
                self.turtle_to_catch_ = msg.turtles[0]
            else:
                closest_turtle = None
                closest_turtle_distance = None
                for turtle in msg.turtles:
                    dist_x = turtle.x - self.pose_.x
                    dist_y = turtle.y - self.pose_.y
                    distance = math.sqrt(dist_x * dist_x  + dist_y * dist_y)
                    if closest_turtle == None or distance < closest_turtle_distance:
                        closest_turtle = turtle
                        closest_turtle_distance = distance
                self.turtle_to_catch_ = closest_turtle
                self.get_logger().info("Start to catch [{}]".format(self.turtle_to_catch_.name))
    
    # Queue Trutles Callbadk
    def callback_queue_turtles(self, msg):
        """
        Manages the list of turtles that should follow the master turtle by setting up necessary publishers and subscribers for their movement control.
        """
        if len(msg.turtles) > 0:
            self.turtle_to_queue_ = msg.turtles
            if len(self.turtle_to_queue_) > len(self.turtle_to_queue_cmd_vel_publisher):
                index = len(self.turtle_to_queue_) - 1
                # dynamic create publisher to control the movement of the turtle
                self.turtle_to_queue_cmd_vel_publisher.append(self.create_publisher(Twist, (self.turtle_to_queue_[-1].name + "/cmd_vel"), 10))
                self.queue_turtle_pose_subscriber.append(self.create_subscription(Pose,(self.turtle_to_queue_[-1].name+"/pose"), partial(self.handle_turtle_pose, self.turtle_to_queue_[-1].name), 1))

######---------------------------------------------------------------#####
    # Main Control Loop
    def control_loop(self):
        if self.pose_ == None or self.turtle_to_catch_ == None:
            return
        # start to append the current position of the master turtle
        dist_x = self.turtle_to_catch_.x - self.pose_.x
        dist_y = self.turtle_to_catch_.y  - self.pose_.y
        position_error = math.sqrt(dist_x * dist_x + dist_y * dist_y)
        # control signal to pass
        msg = Twist()
        if position_error > 0.3:
            # simple proportional controller
            Kp = 0.8 # change the velocity
            msg.linear.x = Kp * position_error
            # orientation
            Ktheta = 8.0 # change the angular velocity
            theta_target = math.atan2(dist_y, dist_x)
            theta_error = theta_target - self.pose_.theta
            if theta_error > math.pi:
                theta_error = theta_error - 2*math.pi
            elif theta_error < -math.pi:
                theta_error = theta_error + 2*math.pi
            msg.angular.z = Ktheta*theta_error
        else:
            # turtle stops
            msg.linear.x = 0.0 
            msg.angular.z = 0.0
            self.call_catch_turtle_server(self.turtle_to_catch_.name)
            self.turtle_to_catch_ = None
        #self.get_logger().info("start to process main, size of turtle_to_queue: {}".format(len(self.turtle_to_queue_)))
        self.cmd_vel_publisher_.publish(msg)
        if len(self.turtle_to_queue_) > 0:
            self.control_loop_follower()

    # Followers Control Loop
    def control_followers(self, from_frame_rel, to_frame_rel, index):
        """
        This function is used to control the followers
        """
        t = self.tf_buffer.lookup_transform(to_frame_rel, from_frame_rel, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=5.0))
        distance = math.sqrt(t.transform.translation.x ** 2 + t.transform.translation.y ** 2)
        msg = Twist()
        if distance > 1.0:
            scale_rotation_rate = 10.0
            msg.angular.z = scale_rotation_rate * math.atan2(t.transform.translation.y, t.transform.translation.x)
            scale_forward_speed = 1.5
            msg.linear.x = scale_forward_speed * math.sqrt(t.transform.translation.x ** 2 + t.transform.translation.y ** 2)
        else:
            msg.angular.z = 0.0
            msg.linear.x = 0.0
        self.turtle_to_queue_cmd_vel_publisher[index].publish(msg)

    # Control Loop for Followers
    def control_loop_follower(self):
        for (i, turtle) in enumerate(self.turtle_to_queue_):
            from_frame_rel = self.turtle_master_name_ if i == 0 else self.turtle_to_queue_[i-1].name
            to_frame_rel = turtle.name
            self.control_followers(from_frame_rel, to_frame_rel, i)

######---------------------------------------------------------------#####
    # Service Call
    """
    Calls the catch_turtle service to catch a turtle. 
    The callback method handles the response, logging whether the catch was successful.
    """
    def call_catch_turtle_server(self,turtle_name):
        client = self.create_client(CatchTurtle, "catch_turtle")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server...")
        request = CatchTurtle.Request()
        request.turtle_name = turtle_name
        future_obj = client.call_async(request)
        future_obj.add_done_callback(partial(self.callback_call_catch_turtle_server, turtle_name=turtle_name))

    def callback_call_catch_turtle_server(self, future_obj, turtle_name):
        try:
            response = future_obj.result()
            if not response.success:
                self.get_logger().error("Turtle " + str(turtle_name) + " was not caught")
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))



def main(args=None):
    rclpy.init(args=args)
    node = TurtleControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()