#!/usr/bin/env python3
import sys
import rclpy
import serial
import time

from std_msgs.msg import Float32, Empty
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default

active = True

def do_shutdown(msg):
    global active
    active = False

def main(args=None):
    rclpy.init(args=args)
    node = Node('thruster_force_gauge_publisher')
    node.declare_parameter("port", "/dev/ttyACM0")
    pub = node.create_publisher(
        Float32, "force_gauge/force", qos_profile_system_default)
    
    shutdown_sub = node.create_subscription(
        Empty, "force_gauge/shutdown", do_shutdown, qos_profile_system_default)

    openedPort = False
    while(openedPort == False):
        try:
            ser = serial.Serial(node.get_parameter("port").value, 115200)
            openedPort = True
        except:
            node.get_logger().info("Cannot open port: " + str(node.get_parameter("port").value), throttle_duration_sec=1)

    node.get_logger().info("Please let me sit for 5 seconds while I take a tare")
    time.sleep(2.0)

    start = time.time()
    tare = 0
    weight = 0
    badReadings = 0
    while(time.time() < start + 5):
        if ser.in_waiting:

            try:
                new = float(ser.readline()[:-2])

                node.get_logger().info("New: " + str(new) + " tare: " + str(tare))

                if tare == 0:
                    tare = new
                    weight = 1

                else:
                    tare = tare * ((weight) / (weight + 1)) + new / (weight + 1)
            except:
                badReadings += 1

    if(badReadings > 5):
        node.get_logger().warn("Significant Number of Bad Readings in Tare: " + str(badReadings))
    
    while rclpy.ok() and active:
        if ser.in_waiting:
            msg = Float32()
            msg.data = float(ser.readline()[:-2]) - tare
            pub.publish(msg)
        
        # If you change the publish rates on the arduino, adjust this
        rclpy.spin_once(node, timeout_sec=0)
    
    node.get_logger().info("Shutting down")
    rclpy.shutdown()


if __name__ == "__main__":
    main(sys.argv)
