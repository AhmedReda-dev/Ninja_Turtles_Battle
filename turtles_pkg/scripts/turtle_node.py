#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool
from std_msgs.msg import Int16
from turtlesim.srv import *
import threading

class Turtle:
    def __init__(self, name):
        self.name = name
        self.health = 100
        self.num_of_attacks = 10
        self.is_attacking = False

    # create turtle
    def spawn(self, x, y, theta):
        serv = rospy.ServiceProxy('/spawn', Spawn)
        serv(x, y, theta, self.name)
    # kill turtle
    def kill_turtle(self):
        serv = rospy.ServiceProxy('/kill', Kill)
        serv(self.name)

    def publish_all(self):
        rate = rospy.Rate(2)

        health_pub = rospy.Publisher(f"{self.name}/turtle_health", Int16, queue_size=10)
        num_of_attacks_pub = rospy.Publisher(f"{self.name}/turtle_num_of_attacks", Int16, queue_size=10)
        is_attacking_pub = rospy.Publisher(f"{self.name}/is_attacking", Bool, queue_size=10)

        while not rospy.is_shutdown():
            health_msg = self.health
            num_of_attacks_msg = self.num_of_attacks
            is_attacking_msg = self.is_attacking

            health_pub.publish(health_msg)
            num_of_attacks_pub.publish(num_of_attacks_msg)
            is_attacking_pub.publish(is_attacking_msg)
            rate.sleep()
            
    # to puplish the 2 turtles in parallel
    # use this to publish
    def start_publishing(self):
        publishing_thread = threading.Thread(target=self.publish_all)
        publishing_thread.start()

# delete the default turtle
def reset_game():
    reset = rospy.ServiceProxy("/kill", Kill)
    reset("turtle1")

if __name__ == '__main__':
    # create node
    rospy.init_node("turtle_node")
    rospy.loginfo("turtle_node has been started")
    # reset the game
    reset_game()
    # create first turtle
    turtle1 = Turtle("turtle1")
    turtle1.spawn(2,2,0)
    # create second turtle
    turtle2 = Turtle("turtle2")
    turtle2.spawn(9,2,0)
    # puplish the 2 turtles
    turtle2.start_publishing()
    turtle1.start_publishing()



"""
those are the topics that puplished after running the script:
/turtle1/cmd_vel
/turtle1/color_sensor
/turtle1/is_attacking
/turtle1/pose
/turtle1/turtle_health
/turtle1/turtle_num_of_attacks

/turtle2/cmd_vel
/turtle2/color_sensor
/turtle2/is_attacking
/turtle2/pose
/turtle2/turtle_health
/turtle2/turtle_num_of_attacks

"""
