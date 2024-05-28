#!/usr/bin/env python3
import random
import math
from functools import partial
import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn
from std_srvs.srv import Empty
from turtlesim.srv import Kill
from robot_interfaces.msg import Turtle, TurtleArray
from robot_interfaces.srv import CatchTurtle
from turtlesim.srv import SetPen

random.seed(2)

class TurtleSpawnerNode(Node): 
    def __init__(self):
        super().__init__("turtle_spawner")
        self.turtle_first_name_ = "turtle"
        self.turtle_counter = 1
        self.alive_turtles_ = []
        # a new list to store queue turtles
        self.queue_turtles_ = []
        self.caught_turtle_ = None
        self.set_turtle_color("turtle1")
        self.alive_turtles_publisher_= self.create_publisher(TurtleArray,"alive_turtles", 10)
        # create a publisher for the queue turtle
        self.queue_turtles_publisher_= self.create_publisher(TurtleArray,"queue_turtles", 10)
        # change the frequency to 3 second
        self.declare_parameter("spawn_frequency", 3)
        self.spawn_frequency_ = self.get_parameter("spawn_frequency").value
        # declare parameter circle
        self.declare_parameter("circle_spawn", True)
        self.circle_spawn_ = self.get_parameter("circle_spawn").value
        self.timer_ = self.create_timer(self.spawn_frequency_, self.spawn_new_turtle)
        # creating service /catch_turtle
        self.catch_turtle_service_ = self.create_service(CatchTurtle, "catch_turtle", self.callback_catch_turtle)
    
    # Set the pen color of a specified turtle to red
    def set_turtle_color(self, turtle_name):
        cli = self.create_client(SetPen, "{}/set_pen".format(turtle_name))
        while not cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        req = SetPen.Request()
        req.r = 220
        req.g = 0
        req.b = 0
        req.width = 3
        req.off = 1
        future = cli.call_async(req)


    # Service Callback
    def callback_catch_turtle(self, request, response):
        self.call_turtle_killer(request.turtle_name)
        # self.call_erase_path()
        response.success = True
        return response
    

    # Publish Functions
    def publish_alive_turtles(self):
        msg = TurtleArray()
        msg.turtles = self.alive_turtles_
        self.alive_turtles_publisher_.publish(msg)

    def publish_queue_turtles(self):
        msg = TurtleArray()
        msg.turtles = self.queue_turtles_
        self.queue_turtles_publisher_.publish(msg) 


    # Turtle Spawning
    def spawn_new_turtle(self):
        self.turtle_counter +=1
        name = self.turtle_first_name_ + str(self.turtle_counter)
        if self.circle_spawn_:
            xs = [2.1, 2.1, 8.9, 8.9]
            ys = [2.1, 8.9, 8.9, 2.1]
            x = xs[(self.turtle_counter-1) % 4]
            y = ys[(self.turtle_counter-1) % 4]
        else:
            x = random.uniform(2.0, 9.0)
            y = random.uniform(2.0, 9.0)
        theta = random.uniform(0.0, 2*math.pi)
        self.call_turtle_spawner(x, y, theta, name)
        
######---------------------------------------------------------------#####
    # Service Calls
    # 1. Turtle Spawner
    def call_turtle_spawner(self, x, y, theta, turtle_name):
        client = self.create_client(Spawn, "spawn")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server...")
        request = Spawn.Request()
        request.x = x
        request.y = y
        request.theta = theta
        request.name = turtle_name
        future_obj = client.call_async(request)
        future_obj.add_done_callback(partial(self.callback_call_turtle_spawner, x=x, y=y, theta=theta, turtle_name=turtle_name))

    def callback_call_turtle_spawner(self, future_obj, x, y, theta, turtle_name):
        try:
            response = future_obj.result()
            if response.name != "":
                self.get_logger().info("Turtle " + response.name + " has been spawned")
                new_turtle = Turtle()
                new_turtle.name = response.name
                new_turtle.x = x
                new_turtle.y = y
                new_turtle.theta = theta
                self.set_turtle_color(new_turtle.name)
                self.alive_turtles_.append(new_turtle)
                self.publish_alive_turtles()
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))

#####---------------------------------------------------------------######
    # 2. Turtle Killer
    def call_turtle_killer(self, turtle_name):
        #client = self.create_client(Kill, "kill")
        #while not client.wait_for_service(1.0):
        #    self.get_logger().warn("Waiting for Server...")
        #request = Kill.Request()
        #request.name = turtle_name
        #future_obj = client.call_async(request)
        #future_obj.add_done_callback(partial(self.callback_call_turtle_killer, turtle_name=turtle_name))
        self.callback_call_turtle_killer(turtle_name)

    def callback_call_turtle_killer(self, turtle_name):
    #def callback_call_turtle_killer(self, future_obj, turtle_name):
        #try:
            #future_obj.result()
        for (i, turtle) in enumerate(self.alive_turtles_):
            if turtle.name == turtle_name:
                catched_turtle_ =  self.alive_turtles_.pop(i)
                self.queue_turtles_.append(catched_turtle_)
                self.publish_alive_turtles()
                # publish queue turtles
                self.publish_queue_turtles()
                break
        #except Exception as e:
        #    self.get_logger().error("Service call failed %r" % (e,))

#####---------------------------------------------------------------######
    # 3. Erase Path
    def call_erase_path(self):
        client = self.create_client(Empty, "clear")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server...")
        request = Empty.Request()
        future_obj = client.call_async(request)
        future_obj.add_done_callback(partial(self.callback_call_erase_path))

    def callback_call_erase_path(self, future_obj):
        try:
            future_obj.result()
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))
    

def main(args=None):
    rclpy.init(args=args)
    node = TurtleSpawnerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
