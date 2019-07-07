#!/usr/bin/env python


__author__ = "colson, dud3722000@naver.com"

import imp
import threading
import sys
import multiprocessing
import StringIO
import errno
import signal
import socket
import struct
import time
from Queue import Queue

from serial import Serial, SerialException, SerialTimeoutException


ERROR_MISMATCHED_PROTOCOL = "Mismatched protocol version in packet: lost sync or rosserial_python is from different ros release than the rosserial client"
ERROR_NO_SYNC = "no sync with device"
ERROR_PACKET_FAILED = "Packet Failed : Failed to read msg data"


class RosSerialServer:
    """
        RosSerialServer waits for a socket connection then passes itself, forked as a
        new process, to SerialClient which uses it as a serial port. It continues to listen
        for additional connections. Each forked process is a new ros node, and proxies ros
        operations (e.g. publish/subscribe) from its connection to the rest of ros.
    """
    def __init__(self, tcp_portnum, fork_server=False):
        print "Fork_server is: ", fork_server
        self.tcp_portnum = tcp_portnum
        self.fork_server = fork_server

    def listen(self):
        self.serversocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.serversocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        #bind the socket to a public host, and a well-known port
        self.serversocket.bind(("", self.tcp_portnum)) #become a server socket
        self.serversocket.listen(1)

        while True:
            #accept connections
            print "waiting for socket connection"
            (clientsocket, address) = self.serversocket.accept()

            #now do something with the clientsocket
            rospy.loginfo("Established a socket connection from %s on port %s" % (address))
            self.socket = clientsocket
            self.isConnected = True

            if self.fork_server: # if configured to launch server in a separate process
                rospy.loginfo("Forking a socket server process")
                process = multiprocessing.Process(target=self.startSocketServer, args=(address))
                process.daemon = True
                process.start()
                rospy.loginfo("launched startSocketServer")
            else:
                rospy.loginfo("calling startSerialClient")
                self.startSerialClient()
                rospy.loginfo("startSerialClient() exited")

    def startSerialClient(self):
        client = SerialClient(self)
        try:
            client.run()
        except KeyboardInterrupt:
            pass
        except RuntimeError:
            rospy.loginfo("RuntimeError exception caught")
            self.isConnected = False
        except socket.error:
            rospy.loginfo("socket.error exception caught")
            self.isConnected = False
        finally:
            self.socket.close()
            for sub in client.subscribers.values():
                sub.unregister()
            for srv in client.services.values():
                srv.unregister()
            #pass

    def startSocketServer(self, port, address):
        rospy.loginfo("starting ROS Serial Python Node serial_node-%r" % (address,))
        rospy.init_node("serial_node_%r" % (address,))
        self.startSerialClient()

    def flushInput(self):
        pass

    def write(self, data):
        if not self.isConnected:
            return
        length = len(data)
        totalsent = 0

        while totalsent < length:
            sent = self.socket.send(data[totalsent:])
            if sent == 0:
                raise RuntimeError("RosSerialServer.write() socket connection broken")
            totalsent = totalsent + sent

    def read(self, rqsted_length):
        self.msg = ''
        if not self.isConnected:
            return self.msg

        while len(self.msg) < rqsted_length:
            chunk = self.socket.recv(rqsted_length - len(self.msg))
            if chunk == '':
                raise RuntimeError("RosSerialServer.read() socket connection broken")
            self.msg = self.msg + chunk
        return self.msg

    def inWaiting(self):
        try: # the caller checks just for <1, so we'll peek at just one byte
            chunk = self.socket.recv(1, socket.MSG_DONTWAIT|socket.MSG_PEEK)
            if chunk == '':
                raise RuntimeError("RosSerialServer.inWaiting() socket connection broken")
            return len(chunk)
        except socket.error, e:
            if e.args[0] == errno.EWOULDBLOCK:
                return 0
            raise

class SerialClient(object):
    """
        ServiceServer responds to requests from the serial device.
    """

    def __init__(self, port=None, baud=57600, timeout=5.0, fix_pyserial_for_test=False):
        """ Initialize node, connect to bus, attempt to negotiate topics. """

        self.read_lock = threading.RLock()

        self.write_lock = threading.RLock()
        self.write_queue = Queue()
        self.write_thread = None

        self.lastsync = rospy.Time(0)
        self.lastsync_lost = rospy.Time(0)
        self.lastsync_success = rospy.Time(0)
        self.last_read = rospy.Time(0)
        self.last_write = rospy.Time(0)
        self.timeout = timeout
        self.synced = False
        self.fix_pyserial_for_test = fix_pyserial_for_test

        self.pub_diagnostics = rospy.Publisher('/diagnostics', diagnostic_msgs.msg.DiagnosticArray, queue_size=10)

        if port is None:
            # no port specified, listen for any new port?
            pass
        elif hasattr(port, 'read'):
            #assume its a filelike object
            self.port=port
        else:
            # open a specific port
            while not rospy.is_shutdown():
                try:
                    if self.fix_pyserial_for_test:
                        # see https://github.com/pyserial/pyserial/issues/59
                        self.port = Serial(port, baud, timeout=self.timeout, write_timeout=10, rtscts=True, dsrdtr=True)
                    else:
                        self.port = Serial(port, baud, timeout=self.timeout, write_timeout=10)
                    break
                except SerialException as e:
                    rospy.logerr("Error opening serial: %s", e)
                    time.sleep(3)

        if rospy.is_shutdown():
            return

        time.sleep(0.1)           # Wait for ready (patch for Uno)

        # hydro introduces protocol ver2 which must match node_handle.h
        # The protocol version is sent as the 2nd sync byte emitted by each end
        self.protocol_ver1 = '\xff'
        self.protocol_ver2 = '\xfe'
        self.protocol_ver = self.protocol_ver2

        self.publishers = dict()  # id:Publishers
        self.subscribers = dict() # topic:Subscriber
        self.services = dict()    # topic:Service

        self.buffer_out = -1
        self.buffer_in = -1

        self.callbacks = dict()
        # endpoints for creating new pubs/subs
        self.callbacks[TopicInfo.ID_PUBLISHER] = self.setupPublisher
        self.callbacks[TopicInfo.ID_SUBSCRIBER] = self.setupSubscriber
        # service client/servers have 2 creation endpoints (a publisher and a subscriber)
        self.callbacks[TopicInfo.ID_SERVICE_SERVER+TopicInfo.ID_PUBLISHER] = self.setupServiceServerPublisher
        self.callbacks[TopicInfo.ID_SERVICE_SERVER+TopicInfo.ID_SUBSCRIBER] = self.setupServiceServerSubscriber
        self.callbacks[TopicInfo.ID_SERVICE_CLIENT+TopicInfo.ID_PUBLISHER] = self.setupServiceClientPublisher
        self.callbacks[TopicInfo.ID_SERVICE_CLIENT+TopicInfo.ID_SUBSCRIBER] = self.setupServiceClientSubscriber
        # custom endpoints
        self.callbacks[TopicInfo.ID_PARAMETER_REQUEST] = self.handleParameterRequest
        self.callbacks[TopicInfo.ID_LOG] = self.handleLoggingRequest
        self.callbacks[TopicInfo.ID_TIME] = self.handleTimeRequest

        rospy.sleep(2.0)
        self.requestTopics()
        self.lastsync = rospy.Time.now()

        signal.signal(signal.SIGINT, self.txStopRequest)

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

    def run(self):
        """ Forward recieved messages to appropriate publisher. """

        # Launch write thread.
        if self.write_thread is None:
            self.write_thread = threading.Thread(target=self.processWriteQueue)
            self.write_thread.daemon = True
            self.write_thread.start()

        # Handle reading.
        data = ''
        read_step = None
        while not rospy.is_shutdown():
            if (rospy.Time.now() - self.lastsync).to_sec() > (self.timeout * 3):
                if self.synced:
                    rospy.logerr("Lost sync with device, restarting...")
                else:
                    rospy.logerr("Unable to sync with device; possible link problem or link software version mismatch such as hydro rosserial_python with groovy Arduino")
                self.lastsync_lost = rospy.Time.now()
                self.sendDiagnostics(diagnostic_msgs.msg.DiagnosticStatus.ERROR, ERROR_NO_SYNC)
                self.requestTopics()
                self.lastsync = rospy.Time.now()

            # This try-block is here because we make multiple calls to read(). Any one of them can throw
            # an IOError if there's a serial problem or timeout. In that scenario, a single handler at the
            # bottom attempts to reconfigure the topics.
            try:
                with self.read_lock:
                    if self.port.inWaiting() < 1:
                        time.sleep(0.001)
                        continue

                # Find sync flag.
                flag = [0, 0]
                read_step = 'syncflag'
                flag[0] = self.tryRead(1)
                if (flag[0] != '\xff'):
                    continue

                # Find protocol version.
                read_step = 'protocol'
                flag[1] = self.tryRead(1)
                if flag[1] != self.protocol_ver:
                    self.sendDiagnostics(diagnostic_msgs.msg.DiagnosticStatus.ERROR, ERROR_MISMATCHED_PROTOCOL)
                    rospy.logerr("Mismatched protocol version in packet (%s): lost sync or rosserial_python is from different ros release than the rosserial client" % repr(flag[1]))
                    protocol_ver_msgs = {'\xff': 'Rev 0 (rosserial 0.4 and earlier)', '\xfe': 'Rev 1 (rosserial 0.5+)', '\xfd': 'Some future rosserial version'}
                    if flag[1] in protocol_ver_msgs:
                        found_ver_msg = 'Protocol version of client is ' + protocol_ver_msgs[flag[1]]
                    else:
                        found_ver_msg = "Protocol version of client is unrecognized"
                    rospy.loginfo("%s, expected %s" % (found_ver_msg, protocol_ver_msgs[self.protocol_ver]))
                    continue

                # Read message length.
                read_step = 'message length'
                msg_len_bytes = self.tryRead(2)
                msg_length, = struct.unpack("<h", msg_len_bytes)

                # Read message length checksum.
                read_step = 'message length checksum'
                msg_len_chk = self.tryRead(1)
                msg_len_checksum = sum(map(ord, msg_len_bytes)) + ord(msg_len_chk)

                # Validate message length checksum.
                if msg_len_checksum % 256 != 255:
                    rospy.loginfo("wrong checksum for msg length, length %d" %(msg_length))
                    rospy.loginfo("chk is %d" % ord(msg_len_chk))
                    continue

                # Read topic id (2 bytes)
                read_step = 'topic id'
                topic_id_header = self.tryRead(2)
                topic_id, = struct.unpack("<h", topic_id_header)

                # Read serialized message data.
                read_step = 'data'
                try:
                    msg = self.tryRead(msg_length)
                except IOError:
                    self.sendDiagnostics(diagnostic_msgs.msg.DiagnosticStatus.ERROR, ERROR_PACKET_FAILED)
                    rospy.loginfo("Packet Failed :  Failed to read msg data")
                    rospy.loginfo("expected msg length is %d", msg_length)
                    raise

                # Reada checksum for topic id and msg
                read_step = 'data checksum'
                chk = self.tryRead(1)
                checksum = sum(map(ord, topic_id_header) ) + sum(map(ord, msg)) + ord(chk)

                # Validate checksum.
                if checksum % 256 == 255:
                    self.synced = True
                    self.lastsync_success = rospy.Time.now()
                    try:
                        self.callbacks[topic_id](msg)
                    except KeyError:
                        rospy.logerr("Tried to publish before configured, topic id %d" % topic_id)
                        self.requestTopics()
                    rospy.sleep(0.001)
                else:
                    rospy.loginfo("wrong checksum for topic id and msg")

            except IOError as exc:
                rospy.logwarn('Last read step: %s' % read_step)
                rospy.logwarn('Run loop error: %s' % exc)
                # One of the read calls had an issue. Just to be safe, request that the client
                # reinitialize their topics.
                with self.read_lock:
                    self.port.flushInput()
                with self.write_lock:
                    self.port.flushOutput()
                self.requestTopics()

    def send(self, topic, msg):
        """
        Queues data to be written to the serial port.
        """
        self.write_queue.put((topic, msg))

    def _write(self, data):
        """
        Writes raw data over the serial port. Assumes the data is formatting as a packet. http://wiki.ros.org/rosserial/Overview/Protocol
        """
        with self.write_lock:
            self.port.write(data)
            self.last_write = rospy.Time.now()

    def _send(self, topic, msg):
        """
        Send a message on a particular topic to the device.
        """
        length = len(msg)
        if self.buffer_in > 0 and length > self.buffer_in:
            rospy.logerr("Message from ROS network dropped: message larger than buffer.\n%s" % msg)
            return -1
        else:
            #modified frame : header(2 bytes) + msg_len(2 bytes) + msg_len_chk(1 byte) + topic_id(2 bytes) + msg(x bytes) + msg_topic_id_chk(1 byte)
            # second byte of header is protocol version
            msg_len_checksum = 255 - ( ((length&255) + (length>>8))%256 )
            msg_checksum = 255 - ( ((topic&255) + (topic>>8) + sum([ord(x) for x in msg]))%256 )
            data = "\xff" + self.protocol_ver  + chr(length&255) + chr(length>>8) + chr(msg_len_checksum) + chr(topic&255) + chr(topic>>8)
            data = data + msg + chr(msg_checksum)
            self._write(data)
            return length

    def processWriteQueue(self):
        """
        Main loop for the thread that processes outgoing data to write to the serial port.
        """
        while not rospy.is_shutdown():
            if self.write_queue.empty():
                time.sleep(0.01)
            else:
                data = self.write_queue.get()
                while True:
                    try:
                        if isinstance(data, tuple):
                            topic, msg = data
                            self._send(topic, msg)
                        elif isinstance(data, basestring):
                            self._write(data)
                        else:
                            rospy.logerr("Trying to write invalid data type: %s" % type(data))
                        break
                    except SerialTimeoutException as exc:
                        rospy.logerr('Write timeout: %s' % exc)
                        time.sleep(1)
