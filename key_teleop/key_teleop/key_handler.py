#! /usr/bin/env python
# -*- coding: utf-8 -*-
#
# Copyright (c) 2013 PAL Robotics SL.
# All rights reserved.
#
# Software License Agreement (BSD License 2.0)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of PAL Robotics SL. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
#
# Authors:
#   * Siegfried-A. Gevatter
#   * Jeremie Deray (artivis)

import curses

# For 'q' keystroke exit
import os
import signal
import time
import math
import numpy as np 


import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import Int32
from rclpy.qos import qos_profile_system_default
from std_msgs.msg import Header

class TextWindow():

    _screen = None
    _window = None
    _num_lines = None

    def __init__(self, stdscr, lines=10):
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
            raise ValueError('lineno out of bounds')
        height, width = self._screen.getmaxyx()
        y = (height / self._num_lines) * lineno
        x = 10
        for text in message.split('\n'):
            text = text.ljust(width)
            # TODO(artivis) Why are those floats ??
            self._screen.addstr(int(y), int(x), text)
            y += 1

    def refresh(self):
        self._screen.refresh()

    def beep(self):
        curses.flash()


class KeyboardHandler(Node):

    def __init__(self, interface):
        super().__init__('key_teleop')

        self._interface = interface

        self.publisher_ = self.create_publisher(Int32, 'keyboard_key', 10)

        self._hz = self.declare_parameter('hz', 10).value
        self._last_pressed = {}

    def run(self):
        self._running = True
        while self._running:
            while True:
                keycode = self._interface.read_key()
                if keycode is None:
                    break
                self._key_pressed(keycode)
            self._publish()
            # TODO(artivis) use Rate once available
            time.sleep(1.0/self._hz)

    def _key_pressed(self, keycode):
        if keycode == ord('q'):
            self._running = False
            # TODO(artivis) no rclpy.signal_shutdown ?
            os.kill(os.getpid(), signal.SIGINT)
        else:
            self._last_pressed[keycode] = self.get_clock().now()

    def _publish(self):
        now = self.get_clock().now()
        keys = []
        for a in self._last_pressed:
            if now - self._last_pressed[a] < Duration(seconds=0.6):
                keys.append(a)
        if not keys:
            self._interface.clear()
            self._interface.write_line(2, 'Keycode: None')
            self._interface.write_line(5, 'Use arrow keys to move, q to exit.')
            self._interface.refresh()
            msg = Int32()
            msg.data = 0
            self.publisher_.publish(msg)
        else:
            for keycode in keys:
                self._interface.clear()
                self._interface.write_line(2, 'Keycode: %d' % keycode)
                self._interface.write_line(5, 'Use arrow keys to move, q to exit.')
                self._interface.refresh()
                msg = Int32()
                msg.data = keycode
                self.publisher_.publish(msg)

def execute(stdscr):
    rclpy.init()

    app = KeyboardHandler(TextWindow(stdscr))
    app.run()

    app.destroy_node()
    rclpy.shutdown()


def main():
    try:
        curses.wrapper(execute)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
