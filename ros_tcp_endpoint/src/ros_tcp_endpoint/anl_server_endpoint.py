#!/usr/bin/env python

import rospy

from ros_tcp_endpoint import TcpServer, RosPublisher, RosService, RosSubscriber
from ur_msgs.msg import RobotStateRTMsg
from ur_msgs.srv import cmdVelJ1, MoverService
from sensor_msgs import point_cloud2
from sensor_msgs.msg import Image


def main():
    ros_node_name = rospy.get_param("/TCP_NODE_NAME", "TCPServer")
    tcp_server = TcpServer(ros_node_name)

    # Start the Server Endpoint
    rospy.init_node(ros_node_name, anonymous=True)
    # Start the Server Endpoint with a ROS communication objects dictionary for routing messages
    tcp_server.start({
        'Robot_State': RosPublisher('SourceDestination', RobotStateRTMsg, queue_size=10),
        'Cmd_Vel': RosService('Cmd_Vel', cmdVelJ1),
        # '/camera/rgb/image_color': RosSubscriber('/camera/rgb/image_color', Image, tcp_server)
        'ur5_moveit': RosService('ur5_moveit', MoverService)
    })
    rospy.spin()


if __name__ == "__main__":
    main()