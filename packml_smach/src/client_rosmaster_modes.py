#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright 2017, Advanced Remanufacturing and Technology Centre
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
#  * Neither the name of the Southwest Research Institute, nor the names
#    of its contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#

from opcua import Client, ua
from opcua.ua import ua_binary as uabin
from opcua.common.methods import call_method
from opcua import common
from packml_msgs.msg import AllStatus
from packml_msgs.msg import AllTimes
from std_msgs.msg import Bool
from std_msgs.msg import Int8
from packml_msgs.srv import Transition

import time
import rospy


var1 = False
var2 = False
var3 = False
var4 = False
var5 = False
var6 = False
var7 = False
var8 = False
var9 = False
var10 = 0


class HelloClient:
    def __init__(self, endpoint):
        self.client = Client(endpoint)

    def __enter__(self):
        self.client.connect()
        return self.client

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.client.disconnect()

# These functions respond to the activation of a button press through the interface (transmitted via topics)
def callback_1(data):
	global var1
	if data.data == True:
		var1 = True
	else:
		var1 = False

def callback_2(data):
	global var2
	if data.data == True:
		var2 = True
	else:
		var2 = False


def callback_3(data):
	global var3
	if data.data == True:
		var3 = True
	else:
		var3 = False


def callback_4(data):
	global var4
	if data.data == True:
		var4 = True
	else:
		var4 = False

def callback_5(data):
	global var5
	if data.data == True:
		var5 = True
	else:
		var5 = False

def callback_6(data):
	global var6
	if data.data == True:
		var6 = True
	else:
		var6 = False

def callback_7(data):
	global var7
	if data.data == True:
		var7 = True
	else:
		var7 = False

def callback_8(data):
	global var8
	if data.data == True:
		var8 = True
	else:
		var8 = False

def callback_9(data):
	global var9
	if data.data == True:
		var9 = True
	else:
		var9 = False


def callback_10(data):
	global var10
	var10 = data.data

def plc_event(req):
	global var1
	global var2
	global var3
	global var4
	global var5
	global var6
	global var7
	global var8
	global var9
	print req.command
	print "Here"
	if req.command == STOP:
		var2 = True
     		res.success = True
      		res.error_code = res.SUCCESS
      		res.message =  "Successful transition request command: STOP"
		#print res
		return TransitionRequest(res)
	elif req.command == START:
		var1 = True
		res.success = True
      		res.error_code = res.SUCCESS
      		res.message =  "Successful transition request command: START"
		return TransitionRequest(res)
	elif req.command == RESET:
		var3 =  True 
		res.success = True
      		res.error_code = res.SUCCESS
      		res.message =  "Successful transition request command: RESET"
		return TransitionRequest(res)
	elif req.command == HOLD:
		var4 = True
		res.success = True
      		res.error_code = res.SUCCESS
      		res.message =  "Successful transition request command: HOLD"
		return TransitionRequest(res)
	elif req.command == UNHOLD:
		var5 = True
		res.success = True
      		res.error_code = res.SUCCESS
      		res.message =  "Successful transition request command: UNHOLD"
		return TransitionRequest(res)
	elif req.command == SUSPEND:
		var6 = True
		res.success = True
      		res.error_code = res.SUCCESS
      		res.message =  "Successful transition request command: SUSPEND"
		return TransitionRequest(res)
	elif req.command == UNSUSPEND:
		var7 = True
		res.success = True
      		res.error_code = res.SUCCESS
      		res.message =  "Successful transition request command: UNSUSPEND"
		return TransitionRequest(res)
	elif req.command == ABORT:
		var8 = True
		res.success = True
      		res.error_code = res.SUCCESS
      		res.message =  "Successful transition request command: ABORT"
		return TransitionRequest(res)
	elif req.command == CLEAR:
		var9 = True
		res.success = True
      		res.error_code = res.SUCCESS
      		res.message =  "Successful transition request command: CLEAR"
		return TransitionRequest(res)
	else:
		res.success = False
      		res.error_code = res.INVALID_TRANSITION_REQUEST
      		res.message =   "Invalid transition request command " 
		return TransitionRequest(res)


if __name__ == '__main__':
	rospy.init_node('client_rosmaster')
	# Resetting the time counters every time the connection between the PLC and ROS is initialized
	time_stopped=0
	time_idle=0
	time_starting=0
	time_execute=0
	time_completing=0
	time_complete=0
	time_clearing=0
	time_suspended=0
	time_aborting=0
	time_aborted=0
	time_holding=0
	time_held=0
	time_unholding=0
	time_suspending=0
	time_unsuspending=0
	time_resetting=0
	time_stopping=0

	#s = rospy.Service('transition', Transition, plc_event)

	with HelloClient("opc.tcp://192.168.0.1:4840/freeopcua/server/") as client:

		global_time=time.time() #The connection is active, the time counters start with this value

		while not rospy.is_shutdown():
			
			# Check the state status in PLC
			stopped_0 = client.get_node("ns=3;s=\"PackML_Status\".\"Sts\".\"State\".\"Stopped\"")
			idle_0 = client.get_node("ns=3;s=\"PackML_Status\".\"Sts\".\"State\".\"Idle\"")
			starting_0 = client.get_node("ns=3;s=\"PackML_Status\".\"Sts\".\"State\".\"Starting\"")
			execute_0 = client.get_node("ns=3;s=\"PackML_Status\".\"Sts\".\"State\".\"Execute\"")
			completing_0 = client.get_node("ns=3;s=\"PackML_Status\".\"Sts\".\"State\".\"Completing\"")
			complete_0 = client.get_node("ns=3;s=\"PackML_Status\".\"Sts\".\"State\".\"Complete\"")
			clearing_0 = client.get_node("ns=3;s=\"PackML_Status\".\"Sts\".\"State\".\"Clearing\"")
			suspended_0 = client.get_node("ns=3;s=\"PackML_Status\".\"Sts\".\"State\".\"Suspended\"")
			aborting_0 = client.get_node("ns=3;s=\"PackML_Status\".\"Sts\".\"State\".\"Aborting\"")	
			aborted_0 = client.get_node("ns=3;s=\"PackML_Status\".\"Sts\".\"State\".\"Aborted\"")	  
			holding_0 = client.get_node("ns=3;s=\"PackML_Status\".\"Sts\".\"State\".\"Holding\"")  
			held_0 = client.get_node("ns=3;s=\"PackML_Status\".\"Sts\".\"State\".\"Held\"")
			unholding_0 = client.get_node("ns=3;s=\"PackML_Status\".\"Sts\".\"State\".\"Unholding\"")   
			suspending_0 = client.get_node("ns=3;s=\"PackML_Status\".\"Sts\".\"State\".\"Suspending\"")
			unsuspending_0 = client.get_node("ns=3;s=\"PackML_Status\".\"Sts\".\"State\".\"Unsuspending\"")
			resetting_0 = client.get_node("ns=3;s=\"PackML_Status\".\"Sts\".\"State\".\"Resetting\"")
			stopping_0 = client.get_node("ns=3;s=\"PackML_Status\".\"Sts\".\"State\".\"Stopping\"")

		
			mode2 = client.get_node("ns=3;s=\"PackTags\".\"Status\".\"UnitModeCurrent\"")
			pubm = rospy.Publisher('plcMode', Int8, queue_size=1, latch=True)
			pubm.publish(mode2.get_value())
			#print mode2.get_value()
			#mode3 = client.get_node("ns=3;s=\"PackML_Status\".\"Sts\".\"Mode\"")
			#print mode3.get_value()
			#mode2a = client.get_node("ns=3;s=\"Remote_Data\".\"ModeTransitions\"")
			#thefirst = mode2a.get_value()
			#mode2aa = client.get_node("ns=3;s=\"Remote_Data\".\"DisableStates\"")
			#thefirsta = mode2aa.get_value()
			#print "Neg", thefirst,thefirsta
			#mode5 = client.get_node("ns=3;s=\"Remote_Data\".\"Inp_RemoteModeCmd\"")
			#print mode5.get_value()
			
			#mode4 = client.get_node("ns=3;s=\"PackTags\".\"Status\".\"MaintenanceModeRequest\"")
			#mode4.set_value(True)
			#mode5 = client.get_node("ns=3;s=\"PackTags\".\"Status\".\"enableBooleanInterface\"")
			#mode4.set_value(True)
			mode3 = client.get_node("ns=3;s=\"PackML_Status\".\"Cmd\".\"Mode\"")
			
			#print mode3
			subint = mode3.get_children()
			if var10==0:
				subint[0].set_value(True)
				subint[1].set_value(False)
				subint[2].set_value(False)
			elif var10==1:
				subint[0].set_value(False)
				subint[1].set_value(True)
				subint[2].set_value(False)
			elif var10==2:
				subint[0].set_value(False)
				subint[1].set_value(False)
				subint[2].set_value(True)
			#for i in range (0,len(subint)):
			#	print subint[i].get_value()
			#print "---"

			#again3 = client.get_node("ns=3;s=\"PackML_Status\".\"Sts\".\"Mode\"")
			#print again3.get_children()
			#again4 = again3.get_children()
			#for i in range (0,len(again4)):
			#	print again4[i].get_value()
			#print "---"
			#again4[0].set_value(True)
			#again4[1].set_value(True)

			#Send the status to be updated and displayed in red on the interface
			pub1 = rospy.Publisher('allStatus', AllStatus, queue_size=1, latch=True)
			pub1.publish(stopped_0.get_value(),idle_0.get_value(),starting_0.get_value(),execute_0.get_value(),completing_0.get_value(),complete_0.get_value(),clearing_0.get_value(), suspended_0.get_value(),aborting_0.get_value(),aborted_0.get_value(),holding_0.get_value(),held_0.get_value(),unholding_0.get_value(),suspending_0.get_value(),unsuspending_0.get_value(), resetting_0.get_value(),stopping_0.get_value())
			#print stopped_0.get_value(),idle_0.get_value(),starting_0.get_value(),execute_0.get_value(),completing_0.get_value(),complete_0.get_value(),clearing_0.get_value(), suspended_0.get_value(),aborting_0.get_value(),aborted_0.get_value(),holding_0.get_value(),held_0.get_value(),unholding_0.get_value(),suspending_0.get_value(),unsuspending_0.get_value(), resetting_0.get_value(),stopping_0.get_value()

			#Checking and updating the time counters for each state, approximated to 1 decimal place
			newvals = [stopped_0.get_value(),idle_0.get_value(),starting_0.get_value(),execute_0.get_value(),completing_0.get_value(),complete_0.get_value(),clearing_0.get_value(), suspended_0.get_value(),aborting_0.get_value(),aborted_0.get_value(),holding_0.get_value(),held_0.get_value(),unholding_0.get_value(),suspending_0.get_value(),unsuspending_0.get_value(), resetting_0.get_value(),stopping_0.get_value()]
			
			if newvals[0] == True:
				time_stopped = time_stopped + (time.time()-global_time)
			if newvals[1] == True:
				time_idle = time_idle + (time.time()-global_time)
			if newvals[2] ==True:
				time_starting = time_starting + (time.time()-global_time)
			if newvals[3] ==True:
				time_execute = time_execute+ (time.time()-global_time)
			if newvals[4] ==True:
				time_completing = time_completing + (time.time()-global_time)
			if newvals[5] ==True:
				time_complete = time_complete + (time.time()-global_time)
			if newvals[6] ==True:
				time_clearing = time_clearing + (time.time()-global_time)
			if newvals[7] ==True:
				time_suspended = time_suspended  + (time.time()-global_time)
			if newvals[8] ==True:
				time_aborting = time_aborting + (time.time()-global_time)
			if newvals[9] ==True:
				time_aborted = time_aborted + (time.time()-global_time)
			if newvals[10] ==True:
				time_holding = time_holding + (time.time()-global_time)
			if newvals[11] ==True:
				time_held = time_held  + (time.time()-global_time)
			if newvals[12] ==True:
				time_unholding = time_unholding  + (time.time()-global_time)
			if newvals[13] ==True:
				time_suspending = time_suspending + (time.time()-global_time)
			if newvals[14] ==True:
				time_unsuspending = time_unsuspending  + (time.time()-global_time)
			if newvals[15] ==True:
				time_resetting = time_resetting  + (time.time()-global_time)
			if newvals[16] ==True:
				time_stopping = time_stopping + (time.time()-global_time)
			#print time_stopped,time_idle,time_starting,time_execute,time_completing,time_complete,time_clearing,time_suspended,time_aborting,time_aborted,time_holding,time_held, time_unholding,time_suspending,time_unsuspending,time_resetting,time_stopping

			#Sending the time to the interface to be displayed via topics
			pub1 = rospy.Publisher('allTimes', AllTimes, queue_size=1, latch=True)
			pub1.publish(time_stopped,time_idle,time_starting,time_execute,time_completing,time_complete,time_clearing,time_suspended,time_aborting,time_aborted,time_holding,time_held, time_unholding,time_suspending,time_unsuspending,time_resetting,time_stopping)

			
			#Check if a button has been pressed
	    	    	rospy.sleep(0.005)
			rospy.Subscriber('start_button',Bool,callback_1)
			rospy.sleep(0.005)
			rospy.Subscriber('stop_button',Bool,callback_2)
			rospy.sleep(0.005)
			rospy.Subscriber('reset_button',Bool,callback_3)
			rospy.sleep(0.005)
			rospy.Subscriber('hold_button',Bool,callback_4)
			rospy.sleep(0.005)
			rospy.Subscriber('unhold_button',Bool,callback_5)
			rospy.sleep(0.005)
			rospy.Subscriber('suspend_button',Bool,callback_6)
			rospy.sleep(0.005)
			rospy.Subscriber('unsuspend_button',Bool,callback_7)
			rospy.sleep(0.005)
			rospy.Subscriber('abort_button',Bool,callback_8)
			rospy.sleep(0.005)
			rospy.Subscriber('clear_button',Bool,callback_9)
			rospy.sleep(0.005)
			rospy.Subscriber('mode',Int8,callback_10)
			rospy.sleep(0.005)
			global_time = time.time()
			
			
			#React according to the button press, sending an event (command) to the PLC
			if var2 == True:
				cmdStop = client.get_node("ns=3;s=\"PackML_Status\".\"EM00\".\"Unit\".\"Cmd_Stop\"")
				cmdStop.set_value(True)
				time.sleep(0.25)
				cmdStop.set_value(True)
				time.sleep(0.25)
				cmdStop = client.get_node("ns=3;s=\"PackML_Status\".\"EM00\".\"Unit\".\"Cmd_Stop\"")				
				print("Sending Stop Command . . .")
				time.sleep(0.25)
				#cmdStop_value = cmdStop.get_value()
				#print("Cmd_Stop is set to: ", cmdStop_value)
				var2 = False
			if var1 == True:
				cmdStart = client.get_node("ns=3;s=\"PackML_Status\".\"EM00\".\"Unit\".\"Cmd_Start\"")
       				cmdStart.set_value(True)
				time.sleep(0.25)
				cmdStart.set_value(True)
				time.sleep(0.25)
				cmdStart = client.get_node("ns=3;s=\"PackML_Status\".\"EM00\".\"Unit\".\"Cmd_Start\"")
        			print("Sending Start Command . . .")
        			time.sleep(0.25)
        			#cmdStart_value = cmdStart.get_value()
        			#print("Cmd_Start is set to: ", cmdStart_value)
				var1 = False
			if var3 == True:
				cmdReset = client.get_node("ns=3;s=\"PackML_Status\".\"EM00\".\"Unit\".\"Cmd_Reset\"")
				cmdReset.set_value(True)
				time.sleep(0.25)
				cmdReset.set_value(True)
				time.sleep(0.25)
				cmdReset = client.get_node("ns=3;s=\"PackML_Status\".\"EM00\".\"Unit\".\"Cmd_Reset\"")
				print("Sending Reset Command . . .")
				time.sleep(0.25)
				#cmdReset_value = cmdReset.get_value()
				#print("Cmd_Reset is set to: ", cmdReset_value)
 				var3 = False
			if var4 == True:
				cmdHold = client.get_node("ns=3;s=\"PackML_Status\".\"EM00\".\"Unit\".\"Cmd_Hold\"")
				cmdHold.set_value(True)
				time.sleep(0.25)
				cmdHold.set_value(True)
				time.sleep(0.25)
				cmdHold = client.get_node("ns=3;s=\"PackML_Status\".\"EM00\".\"Unit\".\"Cmd_Hold\"")
				print("Sending Hold Command . . .")
				time.sleep(0.25)
				#cmdHold_value = cmdHold.get_value()
				#print("Cmd_Hold is set to: ", cmdHold_value)
 				var4 = False
			if var5 == True:
				cmdUnhold = client.get_node("ns=3;s=\"PackML_Status\".\"EM00\".\"Unit\".\"Cmd_Unhold\"")
				cmdUnhold.set_value(True)
				time.sleep(0.25)
				cmdUnhold.set_value(True)
				time.sleep(0.25)
				cmdUnhold = client.get_node("ns=3;s=\"PackML_Status\".\"EM00\".\"Unit\".\"Cmd_Unhold\"")
				print("Sending Unhold Command . . .")
				time.sleep(0.25)
				#cmdUnhold_value = cmdUnhold.get_value()
				#print("Cmd_Unhold is set to: ", cmdUnhold_value)
 				var5 = False
			if var6 == True:
				cmdSuspend = client.get_node("ns=3;s=\"PackML_Status\".\"EM00\".\"Unit\".\"Cmd_Suspend\"")
				cmdSuspend.set_value(True)
				time.sleep(0.25)
				cmdSuspend.set_value(True)
				time.sleep(0.25)
				cmdSuspend = client.get_node("ns=3;s=\"PackML_Status\".\"EM00\".\"Unit\".\"Cmd_Suspend\"")
				print("Sending Suspend Command . . .")
				time.sleep(0.25)
				#cmdSuspend_value = cmdSuspend.get_value()
				#print("Cmd_Suspend is set to: ", cmdSuspend_value)
 				var6 = False
			if var7 == True:
				cmdUnsuspend = client.get_node("ns=3;s=\"PackML_Status\".\"EM00\".\"Unit\".\"Cmd_Unsuspend\"")
				cmdUnsuspend.set_value(True)
				time.sleep(0.25)
				cmdUnsuspend.set_value(True)
				time.sleep(0.25)
				cmdUnsuspend = client.get_node("ns=3;s=\"PackML_Status\".\"EM00\".\"Unit\".\"Cmd_Unsuspend\"")
				print("Sending Unsuspend Command . . .")
				time.sleep(0.25)
				#cmdUnsuspend_value = cmdUnsuspend.get_value()
				#print("Cmd_Unsuspend is set to: ", cmdUnsuspend_value)
 				var7 = False
			if var8 == True:
				cmdAbort = client.get_node("ns=3;s=\"PackML_Status\".\"EM00\".\"Unit\".\"Cmd_Abort\"")
				cmdAbort.set_value(True)
				time.sleep(0.25)
				cmdAbort.set_value(True)
				time.sleep(0.25)
				cmdAbort = client.get_node("ns=3;s=\"PackML_Status\".\"EM00\".\"Unit\".\"Cmd_Abort\"")
				print("Sending Abort Command . . .")
				time.sleep(0.25)
				#cmdAbort_value = cmdAbort.get_value()
				#print("Cmd_Abort is set to: ", cmdAbort_value)
 				var8 = False
			if var9 == True:
				cmdClear = client.get_node("ns=3;s=\"PackML_Status\".\"EM00\".\"Unit\".\"Cmd_Clear\"")
				cmdClear.set_value(True)
				time.sleep(0.25)
				cmdClear.set_value(True)
				time.sleep(0.25)
				cmdClear = client.get_node("ns=3;s=\"PackML_Status\".\"EM00\".\"Unit\".\"Cmd_Clear\"")
				print("Sending Clear Command . . .")
				time.sleep(0.25)
				#cmdClear_value = cmdClear.get_value()
				#print("Cmd_Clear is set to: ", cmdClear_value)
 				var9 = False
			
