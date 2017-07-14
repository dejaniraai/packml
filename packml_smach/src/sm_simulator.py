#!/usr/bin/env python
#
#
# Software License Agreement (Apache License)
# Copyright (c) 2017, <Advanced Remanufacturing Technology Centre>
 
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
 
# http://www.apache.org/licenses/LICENSE-2.0
 
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import sys
import logging
import time
import rospy
import smach
import smach_ros
from packml_msgs.msg import AllStatus
from std_msgs.msg import Float64
from std_msgs.msg import Bool

whichstate = 0 #When a button is pressed, checks which is the next state to transition to 
globaltime = 0 #Keep track of how long it has been in a state

# define state Execute
class Execute(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['outcome1','outcome2','outcome3','outcome4','outcome5','outcome6'],
			     input_keys=['time_in'],
                             output_keys=['time_out'])
        
    def execute(self, userdata):
        rospy.loginfo('Executing state Execute')
        # Not in execute anymore
	global whichstate
	global globaltime
	whichstate = 0
	rospy.Subscriber('stop_button',Bool,callback_2)
	rospy.sleep(0.005)
	rospy.Subscriber('hold_button',Bool,callback_4)
	rospy.sleep(0.005)
	rospy.Subscriber('suspend_button',Bool,callback_6)
	rospy.sleep(0.005)
	rospy.Subscriber('abort_button',Bool,callback_8)
        if whichstate == 4:
		userdata.time_out=time.time()
		globaltime = time.time()
           	return 'outcome2'
	elif whichstate == 6:
		userdata.time_out=time.time()
		globaltime = time.time()
	   	return 'outcome3'
	elif whichstate == 8:
		userdata.time_out=time.time()
		globaltime = time.time()
	   	return 'outcome4'
	elif whichstate == 2:
		userdata.time_out=time.time()
		globaltime = time.time()
	   	return 'outcome5'
        else:
		if (globaltime - userdata.time_in) > 3:
			userdata.time_out=time.time()
			globaltime = time.time()
			return 'outcome1' 
		else:
			
			userdata.time_out=userdata.time_in
			return 'outcome6'
           	 

def callback_1(data):
	global whichstate
	if data.data == 1:
		whichstate = 1

def callback_2(data):
	global whichstate
	if data.data == 1:
		whichstate = 2

def callback_3(data):
	global whichstate
	if data.data == 1:
		whichstate = 3


def callback_4(data):
	global whichstate
	if data.data == 1:
		whichstate = 4

def callback_5(data):
	global whichstate
	if data.data == 1:
		whichstate = 5

def callback_6(data):
	global whichstate
	if data.data == 1:
		whichstate = 6

def callback_7(data):
	global whichstate
	if data.data == 1:
		whichstate = 7

def callback_8(data):
	global whichstate
	if data.data == 1:
		whichstate = 8

#------------------------------------------------------------------------------------------------
# define state Idle
class Idle(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['outcome1','outcome2','outcome3','outcome4'],
                             input_keys=['time_in'],
                             output_keys=['time_out'])
        
    def execute(self, userdata):
        rospy.loginfo('Executing state Idle')
         # Not in idle anymore
	global whichstate
	global globaltime
	whichstate = 0
	rospy.Subscriber('start_button',Bool,callback_1)
	rospy.sleep(0.005)
	rospy.Subscriber('stop_button',Bool,callback_2)
	rospy.sleep(0.005)
	rospy.Subscriber('abort_button',Bool,callback_8)
        if whichstate == 1:
		userdata.time_out=time.time()
		globaltime = time.time()
           	return 'outcome1'
	if whichstate == 8:
		userdata.time_out=time.time()
		globaltime = time.time()
           	return 'outcome2'
	if whichstate == 2:
		userdata.time_out=time.time()
		globaltime = time.time()
           	return 'outcome1'
	else:		
		userdata.time_out=userdata.time_in
	   	return 'outcome4'  


#-------------------------------------------------------------------------------------
#Define state stopped
class Stopped(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['outcome1','outcome2','outcome3'],
                             input_keys=['time_in'],
                             output_keys=['time_out'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Stopped')
   	# Not in stopped anymore
	global whichstate
	global globaltime
	whichstate = 0
	rospy.sleep(0.005)
	rospy.Subscriber('reset_button',Bool,callback_3)
	rospy.sleep(0.005)
	rospy.Subscriber('abort_button',Bool,callback_8)
        if whichstate == 3:
		userdata.time_out=time.time()
		globaltime = time.time()
            	return 'outcome1'
	elif whichstate == 8:
		userdata.time_out=time.time()
		globaltime = time.time()
		return 'outcome2'
        else:
		userdata.time_out=userdata.time_in
            	return 'outcome3'

#-------------------------------------------------------------------------------------
# define state Resetting
class Resetting(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['outcome1'],
                             input_keys=['time_in'],
                             output_keys=['time_out'])
        
    def execute(self, userdata):
        rospy.loginfo('Executing state Resetting')
	 # Not in resetting anymore
	global globaltime
	rospy.sleep(3)
	userdata.time_out=userdata.time_in
	globaltime = time.time()
        return 'outcome1'    

#--------------------------------------------------------------------------------------
# define state Starting
class Starting(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['outcome1'],
                             input_keys=['time_in'],
                             output_keys=['time_out'])
        
    def execute(self, userdata):
        rospy.loginfo('Executing state Starting')
	global globaltime
	 # Not in starting anymore
	rospy.sleep(4)
	globaltime = time.time()
	userdata.time_out=userdata.time_in
        return 'outcome1'      

#---------------------------------------------------------------------------------
class Stopping(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['outcome1'],
                             input_keys=['time_in'],
                             output_keys=['time_out'])
        
    def execute(self, userdata):
        rospy.loginfo('Executing state Stopping')
        global globaltime
        # Not in stopping anymore
	rospy.sleep(2)
	userdata.time_out=userdata.time_in
	globaltime = time.time()
        return 'outcome1'      
        

#----------------------------------------------------------------------------------
class Completing(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['outcome1'],
                             input_keys=['time_in'],
                             output_keys=['time_out'])
        
    def execute(self, userdata):
        rospy.loginfo('Executing state Completing')
        global globaltime
        # Not in stopping anymore
	rospy.sleep(2)
	userdata.time_out=userdata.time_in
	globaltime = time.time()
        return 'outcome1'      
     
#--------------------------------------------------------------------------------------
class Complete(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['outcome1','outcome2','outcome3','outcome4'],
                             input_keys=['time_in'],
                             output_keys=['time_out'])
        
    def execute(self, userdata):
        rospy.loginfo('Executing state Complete')

         # Not in idle anymore
	global whichstate
	global globaltime
	whichstate = 0
	rospy.Subscriber('stop_button',Bool,callback_2)
	rospy.sleep(0.005)
	rospy.Subscriber('reset_button',Bool,callback_3)
	rospy.sleep(0.005)
	rospy.Subscriber('abort_button',Bool,callback_8)
        if whichstate == 3:
		userdata.time_out=time.time()
		globaltime = time.time()
           	return 'outcome1'
	if whichstate == 8:
		userdata.time_out=time.time()
		globaltime = time.time()
           	return 'outcome2'
	if whichstate == 2:
		userdata.time_out=time.time()
		pglobaltime = time.time()
           	return 'outcome3'
	else:		
		userdata.time_out=userdata.time_in
	   	return 'outcome4'  

#--------------------------------------------------------------------------------------
class Holding(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['outcome1'],
                             input_keys=['time_in'],
                             output_keys=['time_out'])
        
    def execute(self, userdata):
        rospy.loginfo('Executing state Holding')
        global globaltime
        rospy.sleep(1)
	userdata.time_out=userdata.time_in
	globaltime = time.time()
        return 'outcome1'      

#-----------------------------------------------------------------------------------------
class Held(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['outcome1','outcome2','outcome3','outcome4'],
                             input_keys=['time_in'],
                             output_keys=['time_out'])
        
    def execute(self, userdata):
        rospy.loginfo('Executing state Held')
	global whichstate
	whichstate = 0
         # Not in idle anymore
	rospy.Subscriber('stop_button',Bool,callback_2)
	rospy.sleep(0.005)
	rospy.Subscriber('abort_button',Bool,callback_8)
	global globaltime
        if whichstate == 8:
		userdata.time_out=time.time()
		globaltime = time.time()
           	return 'outcome2'
	if whichstate == 2:
		userdata.time_out=time.time()
		globaltime = time.time()
           	return 'outcome3'
	else:		
		if (globaltime - userdata.time_in)>2:
			userdata.time_out=userdata.time_in
			globaltime = time.time()
	   		return 'outcome1'  
		else:
			userdata.time_out=userdata.time_in
	   		return 'outcome4'  

#--------------------------------------------------------------------------------------
class Unholding(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['outcome1'],
                             input_keys=['time_in'],
                             output_keys=['time_out'])
        
    def execute(self, userdata):
        rospy.loginfo('Executing state Unholding')
        
        # Not in stopping anymore
	global globaltime
	rospy.sleep(1)
	userdata.time_out=userdata.time_in
	globaltime = time.time()
        return 'outcome1'      
        
#---------------------------------------------------------------------------

class Suspending(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['outcome1'],
                             input_keys=['time_in'],
                             output_keys=['time_out'])
        
    def execute(self, userdata):
        rospy.loginfo('Executing state Suspending')
        
        # Not in stopping anymore
	global globaltime
	rospy.sleep(1)
	userdata.time_out=userdata.time_in
	globaltime = time.time()
        return 'outcome1'      
        
#-----------------------------------------------------------------------------------------
class Suspended(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['outcome1','outcome2','outcome3','outcome4'],
                             input_keys=['time_in'],
                             output_keys=['time_out'])
        
    def execute(self, userdata):
        rospy.loginfo('Executing state Suspended')

        global whichstate
	global globaltime
	whichstate = 0
         # Not in idle anymore
	rospy.Subscriber('stop_button',Bool,callback_2)
	rospy.sleep(0.005)
	rospy.Subscriber('abort_button',Bool,callback_8)
        if whichstate == 8:
		userdata.time_out=time.time()
		globaltime = time.time()
           	return 'outcome2'
	if whichstate == 2:
		userdata.time_out=time.time()
		globaltime = time.time()
           	return 'outcome3'
	else:		
		if (globaltime - userdata.time_in)>2:
			userdata.time_out=userdata.time_in
			globaltime = time.time()
	   		return 'outcome1' 
		else:	
			userdata.time_out=userdata.time_in
	   		return 'outcome4' 

#-----------------------------------------------------------------------------------
class Unsuspending(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['outcome1'],
                             input_keys=['time_in'],
                             output_keys=['time_out'])
        
    def execute(self, userdata):
        rospy.loginfo('Executing state Unsuspending')
       	global globaltime
	rospy.sleep(1)
	userdata.time_out=userdata.time_in
	globaltime = time.time()
       	return 'outcome1'      
        
#---------------------------------------------------------------------------

class Aborting(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['outcome1'],
                             input_keys=['time_in'],
                             output_keys=['time_out'])
        
    def execute(self, userdata):
        rospy.loginfo('Executing state Aborting')
        global globaltime
	rospy.sleep(2)
	userdata.time_out=userdata.time_in
	globaltime = time.time()
        return 'outcome1'      
        

#-----------------------------------------------------------------------------------------
class Aborted(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['outcome1'],
                             input_keys=['time_in'],
                             output_keys=['time_out'])
        
    def execute(self, userdata):
        rospy.loginfo('Executing state Aborted')

	global globaltime
	rospy.sleep(1)
	userdata.time_out=userdata.time_in
	globaltime = time.time()
	return 'outcome1'  
        
#---------------------------------------------------------------------------
class Clearing(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['outcome1'],
                             input_keys=['time_in'],
                             output_keys=['time_out'])
        
    def execute(self, userdata):
        rospy.loginfo('Executing state Clearing')

        global globaltime
	rospy.sleep(1.5)
	userdata.time_out=userdata.time_in
	globaltime = time.time()
	return 'outcome1'  
        

########################################################################################
if __name__ == "__main__":
 
    
    rospy.init_node('state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['outcome4'])
    sm.userdata.sm_time = 0
   

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('Stopped', Stopped(), 
                               	transitions={'outcome1':'Resetting', 'outcome2':'Aborting', 'outcome3':'Stopped'}, 
				remapping={'time_in':'sm_time','time_out':'sm_time'})
        smach.StateMachine.add('Resetting', Resetting(), 
                               	transitions={'outcome1':'Idle'}, 
				remapping={'time_in':'sm_time','time_out':'sm_time'})
        smach.StateMachine.add('Idle', Idle(),
                               	transitions={'outcome1':'Starting', 'outcome2':'Aborting', 'outcome3':'Stopping', 'outcome4':'Idle'}, 
				remapping={'time_in':'sm_time','time_out':'sm_time'})
        smach.StateMachine.add('Starting', Starting(),
                               	transitions={'outcome1':'Executing'}, 
				remapping={'time_in':'sm_time','time_out':'sm_time'})
        smach.StateMachine.add('Executing', Execute(),
                               	transitions={'outcome1':'Completing', 'outcome2':'Holding', 'outcome3':'Suspending', 'outcome4':'Aborting', 'outcome5':'Stopping', 'outcome6':'Executing'}, 
				remapping={'time_in':'sm_time','time_out':'sm_time'})
        smach.StateMachine.add('Stopping',Stopping(),
                               	transitions={'outcome1':'Stopped'}, 
				remapping={'time_in':'sm_time','time_out':'sm_time'})
	smach.StateMachine.add('Completing',Completing(),
                               	transitions={'outcome1':'Complete'}, 
				remapping={'time_in':'sm_time','time_out':'sm_time'})
	smach.StateMachine.add('Complete',Complete(),
                               	transitions={'outcome1':'Resetting', 'outcome2':'Aborting', 'outcome3':'Stopping', 'outcome4':'Complete'}, 
				remapping={'time_in':'sm_time','time_out':'sm_time'})
	smach.StateMachine.add('Holding',Holding(),
                               	transitions={'outcome1':'Held'},
				remapping={'time_in':'sm_time','time_out':'sm_time'})
	smach.StateMachine.add('Held',Held(),
                               	transitions={'outcome1':'Unholding', 'outcome2':'Aborting', 'outcome3':'Stopping', 'outcome4':'Held'},
				remapping={'time_in':'sm_time','time_out':'sm_time'})
	smach.StateMachine.add('Unholding',Unholding(),
                               	transitions={'outcome1':'Executing'},
				remapping={'time_in':'sm_time','time_out':'sm_time'})
	smach.StateMachine.add('Suspending',Suspending(),
                               	transitions={'outcome1':'Suspended'},
				remapping={'time_in':'sm_time','time_out':'sm_time'})
	smach.StateMachine.add('Suspended',Suspended(),
                               	transitions={'outcome1':'Unsuspending', 'outcome2':'Aborting', 'outcome3':'Stopping', 'outcome4':'Suspended'},
				remapping={'time_in':'sm_time','time_out':'sm_time'})
	smach.StateMachine.add('Unsuspending',Unsuspending(),
                               	transitions={'outcome1':'Executing'},
				remapping={'time_in':'sm_time','time_out':'sm_time'})
	smach.StateMachine.add('Aborting',Aborting(),
                               	transitions={'outcome1':'Aborted'},
				remapping={'time_in':'sm_time','time_out':'sm_time'})
	smach.StateMachine.add('Aborted',Aborted(),
                               	transitions={'outcome1':'Clearing'},
				remapping={'time_in':'sm_time','time_out':'sm_time'})
	smach.StateMachine.add('Clearing',Clearing(),
                               	transitions={'outcome1':'Stopped'},
				remapping={'time_in':'sm_time','time_out':'sm_time'})



    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('packmlSmach', sm,'/SM_PACKML')
    sis.start()
    
    # Execute SMACH plan
    outcome = sm.execute()
 
    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()


