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
    turtle_one = Turtle("turtle_one")
    turtle_one.spawn(2,2,0)
    # create second turtle
    turtle_two = Turtle("turtle_two")
    turtle_two.spawn(9,2,0)
    # puplish the 2 turtles
    turtle_two.start_publishing()
    turtle_one.start_publishing()



"""
those are the topics that puplished after running the script:
/turtle_one/cmd_vel
/turtle_one/color_sensor
/turtle_one/is_attacking
/turtle_one/pose
/turtle_one/turtle_health
/turtle_one/turtle_num_of_attacks

/turtle_two/cmd_vel
/turtle_two/color_sensor
/turtle_two/is_attacking
/turtle_two/pose
/turtle_two/turtle_health
/turtle_two/turtle_num_of_attacks

"""