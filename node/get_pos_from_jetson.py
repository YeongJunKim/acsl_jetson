#!/usr/bin/env python


__author__ = "colson@korea.ac.kr"
__author_sub_ = "dud3722000@naver.com"


import rospy
from time import sleep
import multiprocessing

import imp
import threading
import StringIO
import errno
import signal
import socket
import struct
from Queue import Queue

from serial import Serial, SerialException, SerialTimeoutException


import sys

import time



class SerialClient(object):
    """
        ServiceServer responds to requests from the serial device.
    """

    def __init__(self, port=None, baud=57600, timeout=5.0, fix_pyserial_for_test=False):
        """ Initialize node, connect to bus, attempt to negotiate topics. """

        self.read_lock = threading.RLock()

        self.last_read = rospy.Time(0)
        self.timeout = timeout
        self.synced = False
        self.fix_pyserial_for_test = fix_pyserial_for_test

        self.machine_state = '\x00'
        self.checksum = '\x00'

        if port is None:
            # no port specified, listen for any new port?
            pass
        elif hasattr(port, 'read'):
            #assume its a filelike object
            self.port=port
        else:
            # open a specific port
            try:
                if self.fix_pyserial_for_test:
                    self.port = Serial(port, baud, timeout=self.timeout, write_timeout=10, rtscts=True, dsrdtr=True)
                else:
                    self.port = Serial(port, baud, timeout=self.timeout, write_timeout=10)
            except SerialException as e:
                rospy.logerr("Error opening serial: %s", e)
                rospy.signal_shutdown("Error opening serial: %s" % e)
                raise SystemExit
                

    def tryRead(self, length):
        try:
            read_start = time.time()
            bytes_remaining = length
            result = bytearray()
            while bytes_remaining != 0 and time.time() - read_start < self.timeout:
                with self.read_lock:
                    received = self.port.read(bytes_remaining)
                if len(received) != 0:
                    self.last_read = rospy.Time.now()
                    result.extend(received)
                    bytes_remaining -= len(received)

            if bytes_remaining != 0:
                raise IOError("Returned short (expected %d bytes, received %d instead)." % (length, length - bytes_remaining))

            return bytes(result)
        except Exception as e:
            raise IOError("Serial Port read failure: %s" % e)

    def bytes_to_int(bytes):
        result = 0
        for b in bytes:
            result = result * 256 + int(b)

        return result

    def int_to_bytes(value, length):
        result = []

        for i in range(0, length):
            result.append(value >> (i*8) & 0xff)

        result.reverse()
        return result
    def run(self):

        rospy.loginfo("run?\r\n")
        read_step = None
        while not rospy.is_shutdown():
            try:
                with self.read_lock:
                    if self.port.inWaiting() < 1:
                        time.sleep(0.001)
                        continue

                flag = [0, 0]

                data = self.tryRead(1)

                rospy.loginfo("data : " + data +"\r\n")
                if (self.machine_state == '\x00'):
                    if(data == '\xFF'):
                        self.machine_state = '\x01'
                        rospy.loginfo("FF?\r\n")
                    else:
                        self.machine_state = '\x00'
                        rospy.loginfo("NO\r\n")

                elif (self.machine_state == '\x01'):
                    if(data == '\xFF'):
                        self.machine_state = '\x02'
                    else:
                        self.machine_state = '\x00'

                elif (self.machine_state == '\x02'):
                    # get x pos
                    self.machine_state = '\x03'
                elif (self.machine_state == '\x03'):
                    # get y pos
                    self.machine_state = '\x04'
                elif (self.machine_state == '\x04'):
                    # get x size
                    self.machine_state = '\x05'
                elif (self.machine_state == '\x05'):
                    # get y size
                    self.machine_state = '\x06'
                elif (self.machine_state == '\x06'):
                    # checksum (x pos + y pos + x size + y size)
                    if (self.checksum == data):
                        # TODO publishing position , size
                        rospy.loginfo("machine complete \r\n")
                    
                    self.checksum = '\x00'
                    self.machine_state = '\x00'
            except IOError as exc:
                rospy.logwarn('Last read step: %s' % read_step)
                rospy.logwarn('Run loop error: %s' % exc)
                with self.read_lock:
                    self.port.flushInput()


if __name__=="__main__":
    
    rospy.init_node("get_box_node")
    rospy.loginfo("Get Box size and position Python Node")

    port_name = rospy.get_param('jetson_port', '/dev/ttyUSB0')
    baud = int(rospy.get_param('jetson_baud', '115200'))


    while not rospy.is_shutdown():
        try:
            client = SerialClient(port_name, baud)
            client.run()
        except KeyboardInterrupt:
            break
        except SerialException:
            sleep(1.0)
            continue
        except OSError:
            sleep(1.0)
            continue



        

