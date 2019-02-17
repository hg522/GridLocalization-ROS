#!/usr/bin/env python
#license removed for brevity

import rosbag
import rospy
import numpy as np
import tf
import sys

from std_msgs.msg import String
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

ubid = '50292195'
np.random.seed(sum([ord(c) for c in ubid]))

marker_publisher = rospy.Publisher('visualization_marker', Marker, queue_size=100)
tag_marker = Marker ()
motion_marker = Marker ()

gridsize = 7.0
gridstep = 0.2
anglestep = 10.0
roboinitpose = [12,28,21]
probSkipTh = 0.1

#initializing a 3D numpy grid to visualize the movement of the robot and update after each iteration.
maingrid = np.zeros((np.int32(gridsize/gridstep),np.int32(gridsize/gridstep),np.int32(360/anglestep)))
landmrks = [[1.25,5.25],[1.25,3.25],[1.25,1.25],[4.25,1.25],[4.25,3.25],[4.25,5.25]]
roboFlocations = []

#function to convert the continuous robot location and angle to discretized form corresponding to grid cells
def getDiscretizedLoc(x,y,theta,gridstep,anglestep):
	xloc = np.round(x/gridstep)
	yloc = np.round(y/gridstep)
	angle = np.round((theta/anglestep) + 1)
	return xloc,yloc,angle

#function to convert discretized location and angle back to continuous values
def getContinuousLoc(x,y,theta,gridstep,anglestep):
	xloc = x * gridstep
	yloc = y * gridstep
	theta = theta * anglestep
	return xloc,yloc,theta

#function to convert angles to the range 0 to 360 for proper calculation of probabilities
def convertToPiRange(angle):
	if angle > np.degrees(np.pi):
		angle = angle-(np.degrees(2*np.pi))
	elif angle < -np.degrees(np.pi):
		angle = angle+(np.degrees(2*np.pi))
	return angle

#function to calculate the probability of an element like rotation or translation, in our case with mean bieng the value
#obtained from the rosbag and the current value calculated by us
def getGaussProb(t,tmean,variance):
	prb = (1.0/np.sqrt(2.0*np.pi*(variance**2)))*(np.power(np.e,-((t-tmean)**2)/(2.0*variance**2)))
	return prb

#function to add noise to the rotations and translations with variance bieng the 
def addGaussNoise(sample,mean,variance):
	#noise = np.random.normal(mean,variance)
	noise = variance
	sample = sample + noise
	return sample

#function to calculate the rotation 1, rotation 2 and translation between the first point and the possible point to move
def getNewMovement(x2,y2,theta2,x1,y1,theta1):
	global gridstep,anglestep
	x2,y2,theta2 = getContinuousLoc(x2,y2,theta2,gridstep,anglestep)
	x1,y1,theta1 = getContinuousLoc(x1,y1,theta1,gridstep,anglestep)
	an = np.degrees(np.arctan2(y2-y1,x2-x1))
	
	#since arctan gives the angle between range -180 to 180, we need to convert theta from range (0,360) to the same	
	if theta1 > 180:
		theta1 = theta1 - 360
	if theta2 > 180:
		theta2 = theta2 - 360
	
	r1 = an - theta1
	r2 = theta2 - theta1 - r1
	translation = np.sqrt(np.square(x2 - x1) + np.square(y2 - y1))

	#here noise is added to the calculated translation equal to half the grid step
	translation = addGaussNoise(translation,0,gridstep/2.0)
	
	#the calculated rotations have to be converted back to -180 to 180 range after the computations above
	r1 = convertToPiRange(r1)
	r2 = convertToPiRange(r2)
	
	#noise is added to the rotations
	r1 = addGaussNoise(r1,0,anglestep/2.0)
	r2 = addGaussNoise(r2,0,anglestep/2.0)
	#r1 = convertToPiRange(r1)
	#r2 = convertToPiRange(r2)

	return r1,translation,r2

def getMovementAfterObservation(x2,y2,x1,y1,theta1):
	global gridstep,anglestep
	x1,y1,theta1 = getContinuousLoc(x1,y1,theta1,gridstep,anglestep)
	an = np.degrees(np.arctan2(y2-y1,x2-x1))
	
	if theta1 > 180:
		theta1 = theta1 - 360	
		
	r1 = an - theta1
	
	translation = np.sqrt(np.square(x2 - x1) + np.square(y2 - y1))
	translation = addGaussNoise(translation,0,gridstep/2.0)
	r1 = convertToPiRange(r1)

	r1 = addGaussNoise(r1,0,anglestep/2.0)
	#r1 = convertToPiRange(r1)

	return r1,translation
	
#function to iterate over the grid and distribute the probabilities of movement from a given grid cell to every cell in the grid
def distributeProbs(maingrid,maingridcpy,x,y,theta,r1org,r2org,transorg):
	global gridstep,anglestep
	totprbinst = 0.0
	for xt in range(maingrid.shape[0]):
		for yt in range(maingrid.shape[1]):
			for thetat in range(maingrid.shape[2]):
				rotvar = anglestep/2.0
				r1t,transt,r2t = getNewMovement(xt,yt,thetat,x,y,theta)
				pr1t = getGaussProb(r1t,r1org,rotvar)
				tranvar = gridstep/2.0
				ptranst = getGaussProb(transt,transorg,tranvar)
				pr2t = getGaussProb(r2t,r2org,rotvar)
				probtomove = pr1t*ptranst*pr2t
				probtodist = maingrid[x][y][theta] * probtomove
				totprbinst += probtodist
				maingridcpy[xt][yt][thetat] += probtodist
	return maingridcpy,totprbinst

#function to iterate over the grid and select once cell to distribute its probabilities after movement using the above function
def processGridMoveProbs(msg,maingrid):
	transorg = msg.translation
	maingridcpy = np.copy(maingrid)
	bayesTotPrb = 0.0
	r1org = np.degrees(tf.transformations.euler_from_quaternion([msg.rotation1.x,msg.rotation1.y,msg.rotation1.z,msg.rotation1.w])[2])
	r2org = np.degrees(tf.transformations.euler_from_quaternion([msg.rotation2.x,msg.rotation2.y,msg.rotation2.z,msg.rotation2.w])[2])
	
	for x in range(maingridcpy.shape[0]):
		for y in range(maingridcpy.shape[1]):
			for theta in range(maingridcpy.shape[2]):
				if maingrid[x][y][theta] > probSkipTh:
					maingridcpy,totprbinst = distributeProbs(maingrid,maingridcpy,x,y,theta,r1org,r2org,transorg)
					bayesTotPrb+=totprbinst
	return maingridcpy,bayesTotPrb

#function to calculate the cell probabilities after observations
def processGridObsProbs(msg,maingrid):
	global landmrks
	tagindex = msg.tagNum
	translation = msg.range 
	bearing = msg.bearing
	bearing = np.degrees(tf.transformations.euler_from_quaternion([bearing.x,bearing.y,bearing.z,bearing.w])[2])
	maingridcpy = np.copy(maingrid)
	xl,yl = landmrks[tagindex]
	for x in range(maingridcpy.shape[0]):
		for y in range(maingridcpy.shape[1]):
			for theta in range(maingridcpy.shape[2]):
				rotvar = anglestep/2.0
				rl,tl = getMovementAfterObservation(xl,yl,x,y,theta)
				prl = getGaussProb(rl,bearing,rotvar)
				tranvar = gridstep/2.0
				ptl = getGaussProb(tl,translation,tranvar)
				probexistincell = maingrid[x][y][theta]*prl*ptl
				maingridcpy[x][y][theta] = probexistincell
				
	return maingridcpy

def defineTagMarker ():
	global tag_marker
	tag_marker.header.frame_id = "/local_frame"
	tag_marker.header.stamp = rospy.Time.now()
	
	tag_marker.scale.x = 0.1
	tag_marker.scale.y = 0.1
	tag_marker.scale.z = 0.1
	tag_marker.type = Marker.CUBE
	tag_marker.action = Marker.ADD
	tag_marker.ns = "landmarks"
	tag_marker.color.r = 0.0
	tag_marker.color.g = 1.0
	tag_marker.color.b = 0.0
	tag_marker.color.a = 1.0;
	tag_marker.lifetime = rospy.Duration()

def pubTagMarker () :
	global tag_marker,marker_publisher,landmrks
	
	for ind,loc in enumerate(landmrks):
		tag_marker.id = ind
		tag_marker.pose.position.x = loc[0]
		tag_marker.pose.position.y = loc[1]
		tag_marker.pose.position.z = 0.0
		marker_publisher.publish(tag_marker)

def defineMotionMarker ():
	global motion_marker
	motion_marker.header.frame_id = "/local_frame"
	motion_marker.header.stamp = rospy.Time.now()
	motion_marker.ns = "lines"
	
	motion_marker.scale.x = 0.04
	motion_marker.id = 1
	motion_marker.color.r = 1.0
	motion_marker.color.g = 1.0
	motion_marker.color.a = 1.0
	motion_marker.type = Marker.LINE_STRIP
	motion_marker.action = Marker.ADD

def pubMotionMarker(loc):
	global motion_marker,marker_publisher
	p = Point()
	p.x = loc[0]
	p.y = loc[1]
	p.z = 0.0
	motion_marker.points.append(p)
	marker_publisher.publish(motion_marker)
	

def start() :
	global maingrid
	global roboinitpose
	global anglesize
	global transvar
	global rotvar
	global gridstep,anglestep,roboFlocations
	f = open(sys.argv[2],"w+")
	maingrid[roboinitpose[0]-1][roboinitpose[1]-1][roboinitpose[2]-1] = 1.0
	roboinitposition = getContinuousLoc(roboinitpose[0],roboinitpose[1],roboinitpose[2]-1,gridstep,anglestep)
	#pubMotionMarker(roboinitposition)
	#rospy.sleep(2)
	print("\n")
	#print "Robots Initial grid location  : ",roboinitpose
	#f.write("\nRobots Initial grid location  : "+ str(roboinitpose))
	#print "Robots Initial actual location: ",np.round(roboinitposition,2)
	#f.write("\nRobots Initial actual location: "+ str(np.round(roboinitposition,2)))
	#bag = rosbag.Bag('grid.bag')
	bag = rosbag.Bag(sys.argv[1])
	i = 1
	for topic, msg, t in bag.read_messages():
		if topic == 'Movements':
			maingrid,bayesTotPrb = processGridMoveProbs(msg,maingrid)
			maingrid = maingrid/bayesTotPrb
			#maingrid = maingrid/np.sum(maingrid)
			gridloc = np.unravel_index(np.argmax(maingrid, axis=None), maingrid.shape)
			loc = getContinuousLoc(gridloc[0],gridloc[1],gridloc[2],gridstep,anglestep)
			#roboFlocations.append(loc)
			#print "Robots location after Movement " + str(i) +" :    ",np.round(loc,2)
			pubTagMarker()
			pubMotionMarker(loc)
		elif topic == 'Observations':
			maingrid = processGridObsProbs(msg,maingrid)
			s = np.sum(maingrid)
			maingrid = maingrid/s
			gridloc = np.array(np.unravel_index(np.argmax(maingrid, axis=None), maingrid.shape))
			x,y,theta = getContinuousLoc(gridloc[0],gridloc[1],gridloc[2],gridstep,anglestep)
			printloc = [x+gridstep,y+gridstep,theta]
			roboFlocations.append(loc)
			print "Robots grid location after Observation   " + str(i) +" : ", gridloc+1
			f.write("\nRobots grid location after Observation   " + str(i) +" : "+ str(gridloc+1))
			#print "Robots actual location after Observation " + str(i) +" : ",np.round(printloc,2)
			#f.write("\nRobots actual location after Observation " + str(i) +" : "+str(np.round(printloc,2)))
			i=i+1
			pubTagMarker()
			pubMotionMarker([x,y])
	rospy.sleep(2)		
	bag.close()
	f.close()

if __name__ == '__main__':
	try:
	    rospy.init_node('localization', anonymous=True)
	    defineTagMarker()
	    defineMotionMarker()
	    start()
	except rospy.ROSInterruptException:
	    pass
		
		
	



