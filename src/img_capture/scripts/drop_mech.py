#!/usr/bin/env python3

## opens drop mechanism when receives command
# changes servo value/angle when it recieves true from dropcommand topic

# TODO: pigpio library not installing. need to fix

# import os
import math
import json
import numpy as np
import pandas as pd
import rclpy
from rclpy.node import Node

from std_msgs.msg import String


## Following will be needed to open drop mechanism
# from gpiozero.pins.pigpio import PiGPIOFactory
from gpiozero import Servo
from time import sleep

# factory = PiGPIOFactory()
# servo = Servo(25, pin_factory=factory)


class DropMechNode(Node):
    def __init__(self):
        super().__init__("drop_mech_node")
        self.get_logger().info("DropMechNode started.")

        ## data taken from topics ##
        self.hot_location = None
        self.x = 0
        self.y = 0
        self.alt = 0

        self.servo = Servo(25)
        self.openVal = -1
        self.closeVal = 0

        ## Subscribers ##
        self.img_sub = self.create_subscription(
            String, "hotspot_info", self.drop_callback, 10
        )

    def drop_callback(self, msg):
        try: 
            data = json.loads(msg.data)
            self.x = data.get("x", 0)
            self.y = data.get("y", 0)
            self.alt = data.get("alt", 70)
            self.hot_location = data
            if math.abs(self.x) < 5 and math.abs(self.y) < 5:
                self.get_logger().info(f"Drop mech activated")
                self.servo.value = self.val
        except Exception as e:
            self.get_logger().error(f"Failed to parse hotspot data in mech drop code: {e}")
        # self.hot_location = msg
        # # probably use json to unpack data
        # if msg.data is True:
        #     ## figure out what position to move the servo to
        #     #servo.min()
        #     #servo.max()
        #     servo.value = 0.5
	

def main(args=None):
    rclpy.init(args=args)
    node = DropMechNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()