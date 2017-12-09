import sys
import os
import rospy

sys.path.append('./../../plays_py/scripts/utils/')
from geometry import Vector2D
from config import *
from krssg_ssl_msgs.srv import path_plan
from krssg_ssl_msgs.msg import point_2d
from krssg_ssl_msgs.msg import BeliefState
from krssg_ssl_msgs.msg import gr_Commands
from krssg_ssl_msgs.msg import gr_Robot_Command

from profiler import *
from pid import pid
from pso import PSO
from error import Error

global homePos
global awayPos
homePos = []
awayPos = []
global v
global kubid
global expectedTraverseTime
global start
global pso
global errorInfo
global target

def shouldReplan():
	global homePos,awayPos,kubid
	if v.velocity < 10:
		# Not Moving
		return False

	myPos = Vector2D(int(homePos[kubid].x),int(homePos[kubid].y))
	obsPos = Vector2D()
	index = v.GetExpectedPositionIndex()
	for i in xrange(len(homePos)):
		if i == kubid:
			pass
		else:
			obsPos.x = int(homePos[i].x)
			obsPos.y = int(homePos[i].y)
			if v.ellipse(myPos,obsPos,v.motionAngle[index]):
				return True
	for i in xrange(len(awayPos)):
		obsPos.x = int(awayPos[i].x)
		obsPos.y = int(awayPos[i].y)
		if v.ellipse(myPos,obsPos,v.motionAngle[index]):
			return True

	return False


def Callback(data):
	# print "."
	flag = 0
	global homePos
	global awayPos
	global expectedTraverseTime,kubid
	global v,start,target,errorInfo,pso
	replan = 0
	homePos = data.homePos
	awayPos = data.awayPos
	t = rospy.Time.now()
	t = t.secs + 1.0*t.nsecs/pow(10,9)
	curPos = Vector2D(int(homePos[kubid].x),int(homePos[kubid].y))
	command_msgs = gr_Robot_Command()
	final_command = gr_Commands()
	distance = sqrt(pow(target.x - homePos[kubid].x,2) + pow(target.y - homePos[kubid].y,2))
	
	if distance < BOT_BALL_THRESH:
		vX,vY,eX,eY = 0,0,0,0
		errorInfo.errorIX = 0.0
		errorInfo.errorIY = 0.0
		errorInfo.lastErrorX = 0.0
		errorInfo.lastErrorY = 0.0
	if (t - start< expectedTraverseTime):
		if v.trapezoid(t - start,curPos):
			index = v.GetExpectedPositionIndex()
			if index == -1:
				vX,vY,eX,eY = v.sendVelocity(v.getVelocity(),v.motionAngle[index],index)
				vX,vY = 0,0

			else:
				vX,vY,eX,eY = v.sendVelocity(v.getVelocity(),v.motionAngle[index],index)

		else:
			print(t-start, expectedTraverseTime)
			if expectedTraverseTime == 'REPLAN':
				replan = 1
			print("Motion Not Possible")
			vX,vY,eX,eY = 0,0,0,0
			flag = 1
	else:
		print("TimeOUT")
		vX,vY,eX,eY = 0,0,0,0
		errorInfo.errorIX = 0.0
		errorInfo.errorIY = 0.0
		errorInfo.lastErrorX = 0.0
		errorInfo.lastErrorY = 0.0
	errorMag = sqrt(pow(eX,2) + pow(eY,2))
	if  shouldReplan() or \
		(errorMag > 350 and distance > 2* BOT_BALL_THRESH) or \
		replan == 1:
			print("Should Replan",shouldReplan())
			print("ErrorMag",errorMag > 350 and distance > 2*BOT_BALL_THRESH)
			# print("Time Out", ((t - start) > expectedTraverseTime and distance > 2*BOT_BALL_THRESH))
			

			startPt = point_2d()
			startPt.x = homePos[kubid].x
			startPt.y = homePos[kubid].y
			findPath(startPt,target)
		
	else:
		# if distance < BOT_BALL_THRESH or flag ==1:
		# if flag == 1 and t - start > expectedTraverseTime:
		# 	errorInfo.errorIX = 0.0
		# 	errorInfo.errorIY = 0.0
		# 	errorInfo.lastErrorX = 0.0
		# 	errorInfo.lastErrorY = 0.0
		errorInfo.errorX = eX
		errorInfo.errorY = eY
		vX,vY = pid(vX,vY,errorInfo,pso)
		botAngle = homePos[kubid].theta
		vXBot = vX*cos(botAngle) + vY*sin(botAngle)
		vYBot = -vX*sin(botAngle) + vY*cos(botAngle)
		command_msgs.id          = kubid
		command_msgs.wheelsspeed = 0
		command_msgs.veltangent  = vXBot/1000.0
		command_msgs.velnormal = vYBot/1000.0
		command_msgs.velangular = 0
		command_msgs.kickspeedx  = 0
		command_msgs.kickspeedz  = 0
		command_msgs.spinner     = 0

		final_command.timestamp      = rospy.get_rostime().secs
		final_command.isteamyellow   = False
		final_command.robot_commands = command_msgs
		pub.publish(final_command)

def findPath(startPoint,end):
	global v,expectedTraverseTime,kubid
	global start,target
	global pso,errorInfo
	startPt = point_2d()
	target = point_2d()
	startPt.x = startPoint.x
	startPt.y = startPoint.y
	target.x = end.x
	target.y = end.y
	print("Start Point ",startPt.x,startPt.y)
	print("Target Point",target.x,target.y)
	print("Waiting for service")
	rospy.wait_for_service('planner')

	planner = rospy.ServiceProxy('planner', path_plan)
	message = planner(startPt,target)
	path = []
	for i in xrange(len(message.path)):
		path = path + [Vector2D(message.path[i].x,message.path[i].y)]
	start = rospy.Time.now()
	start = 1.0*start.secs + 1.0*start.nsecs/pow(10,9)
	v = Velocity(path,start,startPt)
	v.updateAngle()
	expectedTraverseTime = v.getTime(v.GetPathLength())
	pso = PSO(5,20,1000,1,1,0.5)
	errorInfo = Error()
	print("Path Planned")


if __name__ == "__main__":
	global v
	global expectedTraverseTime,kubid
	global start
	global target
	kubid = 0
	print "Initializing the node"
	rospy.init_node('node',anonymous=False)
	rospy.Subscriber('/belief_state', BeliefState, Callback, queue_size=1000)
	start = rospy.Time.now()
	start = 1.0*start.secs + 1.0*start.nsecs/pow(10,9)
	pub = rospy.Publisher('/grsim_data', gr_Commands, queue_size=1000)
	startPt = point_2d()
	target = point_2d()
	print("Enter starting")
	startPt.x = -2000
	startPt.y = -1700
	target.x = 1000
	target.y = 1200
	findPath(startPt,target)
	# print("Waiting for service")
	# rospy.wait_for_service('planner')

	# planner = rospy.ServiceProxy('planner', path_plan)
	# message = planner(startPt,target)
	# path = []
	# for i in xrange(len(message.path)):
	# 	path = path + [Vector2D(message.path[i].x,message.path[i].y)]
	# start = rospy.Time.now()
	# start = 1.0*start.secs + 1.0*start.nsecs/pow(10,9)
	# v = Velocity(path,start,startPt)
	# v.updateAngle()
	# expectedTraverseTime = v.getTime(v.GetPathLength())
	rospy.spin()

