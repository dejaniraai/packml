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


import time


var1 = False
var2 = False
var3 = False
var4 = False
var5 = False
var6 = False
var7 = False
var8 = False
var9 = False

class HelloClient:
    def __init__(self, endpoint):
        self.client = Client(endpoint)

    def __enter__(self):
        self.client.connect()
        return self.client

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.client.disconnect()


if __name__ == '__main__':
	with HelloClient("opc.tcp://192.168.0.1:4840/freeopcua/server/") as client:
		try:
    			while True:
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
				time.sleep(0.1)

				print("Initial Stopped State: ", stopped_0.get_value())
				print("Initial Idle State: ", idle_0.get_value())
				print("Initial Starting State: ", starting_0.get_value())
				print("Initial Execute State: ", execute_0.get_value())
				print("Initial Completing State: ", completing_0.get_value())
				print("Initial Complete State: ", complete_0.get_value())
				print("Initial Clearing State: ", clearing_0.get_value())
				print("Initial Suspended State: ", suspended_0.get_value())
				print("Initial Aborting State: ", aborting_0.get_value())
				print("Initial Aborted State: ", aborted_0.get_value())
				print("Initial Holding State: ", holding_0.get_value())
				print("Initial Held State: ", held_0.get_value())
				print("Initial Unholding State: ", unholding_0.get_value())
				print("Initial Suspending State: ", suspending_0.get_value())
				print("Initial Unsuspending State: ", unsuspending_0.get_value())
				print("Initial Resetting State: ", resetting_0.get_value())
				print("Initial Stopping State: ", stopping_0.get_value())
				print("======================")

				theinput = raw_input('Enter command: start: t, stop: s, reset: r, abort: a')
				if theinput == 's':
					cmdStop = client.get_node("ns=3;s=\"PackML_Status\".\"EM00\".\"Unit\".\"Cmd_Stop\"")
					cmdStop.set_value(True)
					time.sleep(0.5)
					cmdStop.set_value(True)
					time.sleep(0.5)
					cmdStop = client.get_node("ns=3;s=\"PackML_Status\".\"EM00\".\"Unit\".\"Cmd_Stop\"")				
					print("Sending Stop Command . . .")
					time.sleep(0.5)
					cmdStop_value = cmdStop.get_value()
					print("Cmd_Stop is set to: ", cmdStop_value)
				
				elif theinput == 't':
					cmdStart = client.get_node("ns=3;s=\"PackML_Status\".\"EM00\".\"Unit\".\"Cmd_Start\"")
	       				cmdStart.set_value(True)
					time.sleep(0.5)
					cmdStart.set_value(True)
					time.sleep(0.5)
					cmdStart = client.get_node("ns=3;s=\"PackML_Status\".\"EM00\".\"Unit\".\"Cmd_Start\"")
					print("Sending Start Command . . .")
					time.sleep(0.5)
					cmdStart_value = cmdStart.get_value()
					print("Cmd_Start is set to: ", cmdStart_value)
				
				elif theinput == 'r':
					cmdReset = client.get_node("ns=3;s=\"PackML_Status\".\"EM00\".\"Unit\".\"Cmd_Reset\"")
					cmdReset.set_value(True)
					time.sleep(0.5)
					cmdReset.set_value(True)
					time.sleep(0.5)
					cmdReset = client.get_node("ns=3;s=\"PackML_Status\".\"EM00\".\"Unit\".\"Cmd_Reset\"")
					print("Sending Reset Command . . .")
					time.sleep(0.5)
					cmdReset_value = cmdReset.get_value()
					print("Cmd_Reset is set to: ", cmdReset_value)

				elif theinput == 'a':
					cmdAbort = client.get_node("ns=3;s=\"PackML_Status\".\"EM00\".\"Unit\".\"Cmd_Abort\"")
					cmdAbort.set_value(True)
					time.sleep(0.5)
					cmdAbort.set_value(True)
					time.sleep(0.5)
					cmdAbort = client.get_node("ns=3;s=\"PackML_Status\".\"EM00\".\"Unit\".\"Cmd_Abort\"")
					print("Sending Abort Command . . .")
					time.sleep(0.5)
					cmdAbort_value = cmdAbort.get_value()
					print("Cmd_Abort is set to: ", cmdAbort_value)
		       
		except KeyboardInterrupt:
    			pass
	


