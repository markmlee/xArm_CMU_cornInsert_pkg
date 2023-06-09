#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from pynput import keyboard


class KeyboardNode(object):
    def __init__(self):
        rospy.init_node("keyboard_node")
        self.pub = rospy.Publisher("keyboard_topic", String, queue_size=10)
        self.listener = keyboard.Listener(on_press=self.on_press)
        self.listener.start()  # start the listener in a separate thread

    def on_press(self, key):
        try:
            # publish the key as a string
            self.pub.publish(str(key.char))
        except AttributeError:
            # special key pressed, ignore for simplicity
            pass

    def run(self):
        # keep the main thread alive
        rospy.spin()


if __name__ == "__main__":
    node = KeyboardNode()
    node.run()
