#!/usr/bin/env python

# Modules used for the websocket server
import socket
import wspy

# Modules used for the ROS node
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

connectionOpened = False # If there's a connection, this will be True
lastOdomMessage = ""
lastLaserMessage = ""

# The websocket server object
class WebSocketServer(wspy.Connection):
	# Run upon opening the connection
	def onopen(self):
		print 'Connection opened at %s:%d' % self.sock.getpeername()
		global connectionOpened
		connectionOpened = True

	# Run upon receiving a message
	def onmessage(self, message):
		print 'Received message "%s"' % message.payload
		if message.payload == "ready": # If the webpage is ready, give it data
			global lastOdomMessage
			global lastLaserMessage
			stringToSend = lastOdomMessage + "\n\n" + lastLaserMessage
			self.send(wspy.TextMessage(unicode(stringToSend, "utf-8")))
			if lastOdomMessage != "OLD" and lastLaserMessage != "old":
				lastOdomMessage = "OLD"
				lastLaserMessage = "OLD"

	# Run upon closing the connection
	def onclose(self, code, reason):
		global connectionOpened
		print 'Connection closed'
		connectionOpened = False

# Start the websocket server on 127.0.0.1:12345 (aka localhost:12345)
server = wspy.websocket()
server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
server.bind(('127.0.0.1', 12345))
server.listen(5)

# This code is run whenever data is received from the ROS subscriber node
def odomCallback(data):
	global lastOdomMessage
	lastOdomMessage = data.__str__() #data.__str__() formats the message as string so it can
	print "Received odom data!"

def laserCallback(data):
	global lastLaserMessage
	lastLaserMessage = data.__str__()
	print "Received scan data!"

# This code creates the subscriber node
def odomListener():
	rospy.init_node('listener', anonymous=True)
	rospy.Subscriber("/odom", Odometry, odomCallback)
	# rospy.spin() is not used, because the code is supposed to loop in the while True loop below

def laserListener():
	rospy.init_node('listener', anonymous=True)
	rospy.Subscriber("/base_scan", LaserScan, laserCallback)

odomListener() # Start the listeners
laserListener()

# Run the websocket server
while True:
	client, addr = server.accept()
	WebSocketServer(client).receive_forever()