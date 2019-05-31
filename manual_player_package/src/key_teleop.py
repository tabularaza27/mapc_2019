#! /usr/bin/env python
# -*- coding: utf-8 -*-
#

import curses
import math

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

import sys

class TextWindow():

    _screen = None
    _window = None
    _num_lines = None

    def __init__(self, stdscr, lines=20):
        self._screen = stdscr
        self._screen.nodelay(True)
        curses.curs_set(0)

        self._num_lines = lines

    def read_key(self):
        keycode = self._screen.getch()
        return keycode if keycode != -1 else None

    def clear(self):
        self._screen.clear()

    def write_line(self, lineno, message):
        if lineno < 0 or lineno >= self._num_lines:
            raise ValueError, 'lineno out of bounds'
        height, width = self._screen.getmaxyx()
        y = (height / self._num_lines) * lineno
        x = 10
        for text in message.split('\n'):
            text = text.ljust(width)
            self._screen.addstr(y, x, text)
            y += 1

    def refresh(self):
        self._screen.refresh()

    def beep(self):
        curses.flash()


class SimpleKeyTeleop():
    def __init__(self, interface, agentName):
        self._interface = interface
        self._agentName = agentName
        self._pub_cmd = rospy.Publisher('key_player_' + agentName, String)

        self._hz = rospy.get_param('~hz', 10)
        self._last_pressed = {}
        self._direction = "n"

        self.movement_bindings = {
        curses.KEY_UP:    "n",
        curses.KEY_DOWN:  "s",
        curses.KEY_LEFT:  "w",
        curses.KEY_RIGHT: "e",
        ord("d"):         "d", # dispense
        ord("a"):         "a", # attach
        ord("c"):         "c", # connect
        ord("z"):         "z", # detach
        ord("l"):         "l", # rotate left
        ord("r"):         "r", # rotate right
        ord("t"):         "t", # submit
        ord("g"):         "g", # gira, change fixed_direction
        ord("1"): "1",  # submit
        ord("2"): "2",  # submit
        ord("3"): "3",  # submit
        ord("4"): "4",  # submit
        ord("5"): "5",  # submit
        ord("6"): "6",  # submit
        ord("7"): "7",  # submit

            #add more keys here
    }

    def run(self):
        rate = rospy.Rate(self._hz)
        self._running = True
        while self._running:
            while True:
                keycode = self._interface.read_key()
                if keycode is None:
                    break
                self._key_pressed(keycode)
            self._set_velocity()
            self._publish()
            rate.sleep()

    def _set_velocity(self):
        now = rospy.get_time()
        keys = []
        for a in self._last_pressed:
            if now - self._last_pressed[a] < 0.05:
                keys.append(a)

        for k in keys:
            self._direction = self.movement_bindings[k]
            self._pub_cmd.publish(self._direction)

    def _key_pressed(self, keycode):
        if keycode == ord('q'):
            self._running = False
            rospy.signal_shutdown('Bye')
        elif keycode in self.movement_bindings:
            self._last_pressed[keycode] = rospy.get_time()

    def _publish(self):
        self._interface.clear()
        self._interface.write_line(1, 'Agent name: ' + self._agentName)
        self._interface.write_line(2, 'Last command sent: ' + self._direction)
        self._interface.write_line(3, 'Use arrow keys to move, q to exit and :')
        self._interface.write_line(4, 'a - attach')
        self._interface.write_line(5, 'c - connect')
        self._interface.write_line(6, 's - submit')
        self._interface.write_line(7, 'l - rotate left')
        self._interface.write_line(8, 'r - rotateright')
        self._interface.write_line(9, 'z - detach')
        self._interface.write_line(10, 'd - dispense')
        self._interface.refresh()

        
        


def main(stdscr):
    rospy.init_node('manual_player_package', anonymous=True)
    agentName = sys.argv[1]
    app = SimpleKeyTeleop(TextWindow(stdscr), agentName)
    app.run()

if __name__ == '__main__':
    try:
        curses.wrapper(main)
    except rospy.ROSInterruptException:
        pass
