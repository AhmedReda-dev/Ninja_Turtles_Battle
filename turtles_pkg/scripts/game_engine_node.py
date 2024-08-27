import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose

turtles_positions = {'turtle1': Pose(), 'turtle2': Pose()}

def update_position(data, turtle_name):
    rospy.loginfo(f"Updating position for {turtle_name}")
    turtles_positions[turtle_name] = data
    #rospy.loginfo(f"New position for {turtle_name}: {turtles_positions[turtle_name]}")

if __name__ == '__main__':
    rospy.init_node('game_engine')
    rospy.loginfo("Node has been started")

    # Subscribers
    rospy.Subscriber('/turtle1/pose', Pose, update_position, 'turtle1')
    rospy.Subscriber('/turtle2/pose', Pose, update_position, 'turtle2')

rospy.spin()