#! /usr/bin/python
import rospy
from geometry_msgs.msg import Point
from rospy.exceptions import ROSInterruptException


def waypoint_publisher():
    global height
    pub = rospy.Publisher('/waypoints', Point, queue_size=1, latch=True)
    # rate = rospy.Rate(20)

    waypoint = Point()
    # while not rospy.is_shutdown():
    #     waypoint.x, waypoint.y, waypoint.z = [0, 0, 2]
    #     pub.publish(waypoint)
    #     rate.sleep()

    waypoint.x, waypoint.y, waypoint.z = [0, 0, height]
    pub.publish(waypoint)
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('waypoint_node')
    height = rospy.get_param("waypoint_node/waypoint")
    try:
        waypoint_publisher()
    except ROSInterruptException:
        pass
