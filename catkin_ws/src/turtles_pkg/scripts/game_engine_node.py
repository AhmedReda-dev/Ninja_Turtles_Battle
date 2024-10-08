#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from turtlesim.msg import Pose
from std_msgs.msg import Bool
from turtlesim.srv import Kill
import math
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

# Initialize dictionaries to store turtle states
turtle_health = {'turtle1': 100, 'turtle2': 100}  
turtle_attacks = {'turtle1': 10, 'turtle2': 10}
turtles_positions = {'turtle1': Pose(), 'turtle2': Pose()}

def update_position(data, turtle_name):
    
    #rospy.loginfo(f"Updating position for {turtle_name}")
    turtles_positions[turtle_name] = data

def check_attack(msg, turtle_name):
    """Callback function to handle attack events."""
    if msg.data:  # Attack is indicated by a True boolean
        rospy.loginfo(f"{turtle_name} has attacked!")
        turtle_attacks[turtle_name] -= 1

        # range check
        if check_turtle_in_range():
            if(turtle_name == "turtle1"): # turtle1 attacked turtle2 we update turtle2 health
                update_health("turtle2") 
                rospy.loginfo(f"turtle2 has been hit!")
            else : # turtle2 attacked turtle1 we update turtle1 health
                update_health("turtle1")
                rospy.loginfo(f"turtle1 has been hit!")
    check_game_end()
             
# Define the range radius
radius_range = 0.376
def check_turtle_in_range():
    """function returns true if the 2 turtles in the range of each other"""
    x1 = turtles_positions['turtle1'].x
    y1 = turtles_positions['turtle1'].y

    x2 = turtles_positions['turtle2'].x
    y2 = turtles_positions['turtle2'].y
    # get the distance between the 2 turtles
    distance_between_turtles = math.sqrt((x1 - x2)**2 + (y1 - y2)**2)
    
    # Check if they are in range
    return (distance_between_turtles <= radius_range)


def update_health(turtle_name):
    """Function to update the health of the turtles."""
    turtles_to_remove = []
    turtle_health[turtle_name] -= 50
    if turtle_health[turtle_name] <= 0:
        turtles_to_remove.append(turtle_name)  

    for turtle in turtles_to_remove:
        remove_turtle(turtle)
        del turtle_health[turtle]  # Remove turtle from health dictionary
        del turtle_attacks[turtle]  # Remove turtle from attacks dictionary
        del turtles_positions[turtle]  # Remove turtle from positions dictionary

turtle_count = 2
def remove_turtle(turtle_name):
    """Remove a turtle from the simulation."""
    reset = rospy.ServiceProxy("/kill", Kill)
    reset(turtle_name)
    rospy.loginfo(f"Removing {turtle_name} from the game.")
    global turtle_count
    turtle_count = turtle_count - 1

def check_game_end():
    global turtle_count

    # Check if all turtles have exhausted their attacks
    attacks_exhausted = all(attacks == 0 for attacks in turtle_attacks.values())

    if turtle_count == 1 or attacks_exhausted:
        end_game()

def end_game():
    """Function to handle the end of the game and declare the winner."""
    max_health = -1  
    winner = None    

    for turtle, health in turtle_health.items():
        if health > max_health:
            max_health = health
            winner = turtle

    result = f'Game Over! The winner is {winner}'
    rospy.loginfo(result)
    end_game_pub.publish(result)
    rospy.signal_shutdown('Game Over')

if __name__ == '__main__':
    # create node
    rospy.init_node('game_engine')
    rospy.loginfo('Node has been started')
    # reset the game
    reset_game()
    # create first turtle
    turtle1 = Turtle("turtle1")
    turtle1.spawn(2,2,0)
    # create second turtle
    turtle2 = Turtle("turtle2")
    turtle2.spawn(9,2,0)
    # publish the 2 turtles
    turtle2.start_publishing()
    turtle1.start_publishing()

    # Subscribers
    rospy.Subscriber('/turtle1/pose', Pose, update_position, 'turtle1')
    rospy.Subscriber('/turtle2/pose', Pose, update_position, 'turtle2')
    rospy.Subscriber('/turtle1/is_attacking', Bool, check_attack, 'turtle1')
    rospy.Subscriber('/turtle2/is_attacking', Bool, check_attack, 'turtle2')

    # Publisher
    end_game_pub = rospy.Publisher('/game_end', String, queue_size=10)

    rospy.spin()
