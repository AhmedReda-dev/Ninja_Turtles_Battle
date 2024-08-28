import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool

turtle_one_health = 100
turtle_one_attacks = 10
turtle_two_health = 100
turtle_two_attacks = 10


turtles_positions = {'turtle1': Pose(), 'turtle2': Pose()}

def update_position(data, turtle_name):
    rospy.loginfo(f"Updating position for {turtle_name}")
    turtles_positions[turtle_name] = data
    #rospy.loginfo(f"New position for {turtle_name}: {turtles_positions[turtle_name]}")

def update_health_turtle1 (msg):
    if msg.data:
        rospy.loginfo ("Attack detected")
        turtle_one_attacks = turtle_one_attacks -1
        turtle_one_health = turtle_one_health -50

    else:
        rospy.loginfo ("No Attack detected")

def update_health_turtle2 (msg):
    if msg.data:
        rospy.loginfo ("Attack detected")
        turtle_two_attacks = turtle_two_attacks -1
        turtle_two_health = turtle_two_health -50

    else:
        rospy.loginfo ("No Attack detected")




if __name__ == '__main__':
    rospy.init_node('game_engine')
    rospy.loginfo("Node has been started")

    # Subscribers
    rospy.Subscriber('/turtle1/pose', Pose, update_position, 'turtle1')
    rospy.Subscriber('/turtle2/pose', Pose, update_position, 'turtle2')
    rospy.Subscriber('/turtle_one/is_attacking' , Bool , update_health_turtle1)
    rospy.Subscriber('/turtle_two/is_attacking' , Bool , update_health_turtle2)

rospy.spin()