#!/usr/bin/env python

import rospy

import socket
import sys
import threading

import glove_connection.message_processor as mp


def process_connection(conn, rate):
    # We sleep twice per loop so double the rate
    time_keeper = rospy.Rate(rate * 2)

    data = ""

    while not rospy.is_shutdown():
        # Read data from the glove
        try:
            data = conn.recv(64)
            if len(data) > 0:
                mp.process_glove_data(data)
        except socket.timeout:
            pass
        except socket.error as e:
            rospy.loginfo("Unknown error: %s", e)
            return

        time_keeper.sleep()

        # Send data to the glove
        msg = mp.get_glove_message()
        try:
            conn.sendall(msg)
        except socket.timeout:
            rospy.loginfo("Connection timed out.")
            return
        except socket.error as e:
            rospy.loginfo("Unknown error: %s", e)
            return

        time_keeper.sleep()

    # Close the connection if the node is shut down
    conn.close()
    listener_running = False
    listener.join()


if __name__ == "__main__":
    rospy.init_node("glove_connection_point")

    # Get the port
    port = rospy.get_param("~port", 0)
    if port == 0:
        rospy.logfatal("Failed to get param 'port'!")
        sys.exit(1)

    # Get the communication frequency
    rate = rospy.get_param("~communication_frequency", 0)
    if rate == 0:
        rospy.logwarn("Failed to get param 'communication_frequency', using default of 10 Hz.")
        rate = 10

    # Get the timeout value
    timeout = rospy.get_param("~connection_timeout", 0)
    if timeout == 0:
        rospy.logwarn("Failed to get param 'connection_timeout', using default of 0.5s.")
        timeout = 0.5

    # Get the hostname
    host = rospy.get_param("~hostname", 0)
    if host == 0:
        # Get the local machine hostname
        host = socket.gethostname()
        rospy.logwarn("Failed to get param 'hostname', using default of '%s'.", host)

    # Create the thread to run the ROS pub/sub
    ros_thread = threading.Thread(target=mp.ros_worker)
    ros_thread.start()

    rospy.loginfo("Starting glove server on '%s:%d'", host, port)
    
    # Create a socket and bind it to the host and port   
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind((host, port))

    # Listen for incoming connections
    s.listen(1024)
    while not rospy.is_shutdown():
        # Accept an incoming connection
        conn, addr = s.accept()
        
        rospy.loginfo("Incoming connection from '" + addr[0] + "' accepted!")

        # Set the timeout for the connection
        conn.settimeout(timeout)

        # Process the connection while it is open
        process_connection(conn, rate)

        rospy.loginfo("Lost connection with '" + addr[0] + "'.")

        # Close the connection
        conn.close()

    ros_thread.join()
    s.close()
