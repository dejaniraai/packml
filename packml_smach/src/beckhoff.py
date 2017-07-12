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
#from packml_msgs.msg import AllStatus
#from packml_msgs.msg import AllTimes
from std_msgs.msg import Bool

import time
import rospy


class HelloClient:
    def __init__(self, endpoint):
        self.client = Client(endpoint)

    def __enter__(self):
        self.client.connect()
        return self.client

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.client.disconnect()


if __name__ == '__main__':
	rospy.init_node('client_rosmaster')
	# Resetting the time counters every time the connection between the PLC and ROS is initialized


	with HelloClient("opc.tcp://10.217.176.40:4840/freeopcua/server/") as client:


		cmdStop = client.get_node("ns=4;s=GVL_MQTT.bZN4S1")
		cmdStop2 = client.get_node("ns=4;s=GVL_MQTT.bZN4S2")
		print cmdStop.get_value()
		print cmdStop2.get_value()
		#cmdStop.set_value(True)
		#time.sleep(2)
		#print cmdStop.get_value()
		"""
		print "-----Emergency"
		cmd1= client.get_node("ns=4;s=GVL_MQTT.bEMG")
		print "bEMG: " +  cmd1.get_value()
		print "-----Zone 1"
		cmd2= client.get_node("ns=4;s=GVL_MQTT.bZN1S1")
		print "bZN1S1: " +  cmd2.get_value()
		cmd3= client.get_node("ns=4;s=GVL_MQTT.bZN1S1")
		print "bZN1S2: " +  cmd3.get_value()
		cmd4= client.get_node("ns=4;s=GVL_MQTT.bZN1PressureMat")
		print "bZN1PressureMat: " + cmd4.get_value()
		cmd5= client.get_node("ns=4;s=GVL_MQTT.nZN1Pallet")
		print "nZN1Pallet: " + cmd5.get_value()
		print "-----Zone 2"
		cmd6= client.get_node("ns=4;s=GVL_MQTT.bZN2S1")
		print "bZN2S1: " +  cmd6.get_value()
		cmd7= client.get_node("ns=4;s=GVL_MQTT.bZN2S1")
		print "bZN2S2: " +  cmd7.get_value()
		cmd8= client.get_node("ns=4;s=GVL_MQTT.nZN2Pallet")
		print "nZN2Pallet: " + cmd8.get_value()
		print "-----Zone 3"
		cmd9= client.get_node("ns=4;s=GVL_MQTT.bZN3S1")
		print "bZN3S1: " +  cmd9.get_value()
		cmd10= client.get_node("ns=4;s=GVL_MQTT.bZN3S1")
		print "bZN3S2: " +  cmd10.get_value()
		cmd11= client.get_node("ns=4;s=GVL_MQTT.bZN3PressureMat")
		print "bZN3PressureMat: " + cmd11.get_value()
		cmd12= client.get_node("ns=4;s=GVL_MQTT.nZN3Pallet")
		print "nZN3Pallet: " + cmd12.get_value()
		print "-----Zone 4"
		cmd13 = client.get_node("ns=4;s=GVL_MQTT.bZN4S1")
		print "bZN4S1" + cmd13.get_value()
		cmd14 = client.get_node("ns=4;s=GVL_MQTT.bZN4S2")
		print "bZN4S2" + cmd14.get_value()
		cmd15= client.get_node("ns=4;s=GVL_MQTT.nZN4Pallet")
		print "nZN4Pallet: " + cmd15.get_value()
		print "-----Zone 5"
		cmd16= client.get_node("ns=4;s=GVL_MQTT.bZN5S1")
		print "bZN5S1: " +  cmd16.get_value()
		cmd17= client.get_node("ns=4;s=GVL_MQTT.bZN5S1")
		print "bZN5S2: " +  cmd17.get_value()
		cmd18= client.get_node("ns=4;s=GVL_MQTT.bZN5PressureMat")
		print "bZN5PressureMat: " + cmd18.get_value()
		cmd19= client.get_node("ns=4;s=GVL_MQTT.nZN5Pallet")
		print "nZN5Pallet: " + cmd19.get_value()
		print "-----Zone 6"
		cmd20= client.get_node("ns=4;s=GVL_MQTT.bZN6S1")
		print "bZN6S1: " +  cmd20.get_value()
		cmd21= client.get_node("ns=4;s=GVL_MQTT.bZN6S1")
		print "bZN6S2: " +  cmd21.get_value()
		cmd22= client.get_node("ns=4;s=GVL_MQTT.nZN6Pallet")
		print "nZN6Pallet: " + cmd22.get_value()
		cmd23= client.get_node("ns=4;s=GVL_MQTT.bZN6Trolley")
		print "bZN6Trolley: " + cmd23.get_value()
		print "-----Inlet"
		cmd24= client.get_node("ns=4;s=GVL_MQTT.bINTS1")
		print "bINTS1" + cmd24.get_value()
		cmd25= client.get_node("ns=4;s=GVL_MQTT.bINTS2")
		print "bINTS1" + cmd25.get_value()
		cmd26= client.get_node("ns=4;s=GVL_MQTT.bINTTrolley")
		print "bINTTrolley" + cmd26.get_value()
		print "-----Times"
		cmd27= client.get_node("ns=4;s=GVL_MQTT.tCell1Cyc")
		print "tCell1Cyc" + cmd27.get_value()
		cmd28= client.get_node("ns=4;s=GVL_MQTT.tCell1Pallet")
		print "tCell1Pallet" + cmd28.get_value()
		cmd29= client.get_node("ns=4;s=GVL_MQTT.bCell1Trigger")
		print "bCell1Trigger" + cmd29.get_value()
		cmd30= client.get_node("ns=4;s=GVL_MQTT.tCell2Cyc")
		print "tCell2Cyc" + cmd30.get_value()
		cmd31= client.get_node("ns=4;s=GVL_MQTT.tCell2Pallet")
		print "tCell2Pallet" + cmd31.get_value()
		cmd32= client.get_node("ns=4;s=GVL_MQTT.bCell2Trigger")
		print "bCell2Trigger" + cmd32.get_value()
		cmd33= client.get_node("ns=4;s=GVL_MQTT.tCell3Cyc")
		print "tCell3Cyc" + cmd33.get_value()
		cmd34= client.get_node("ns=4;s=GVL_MQTT.tCell3Pallet")
		print "tCell3Pallet" + cmd34.get_value()
		cmd35= client.get_node("ns=4;s=GVL_MQTT.bCell3Trigger")
		print "bCell3Trigger" + cmd35.get_value()
		print "-----Full cycle time"
		cmd36= client.get_node("ns=4;s=GVL_MQTT.tFullCyc")
		print "tFullCyc" + cmd36.get_value()
		cmd37= client.get_node("ns=4;s=GVL_MQTT.nCycPallet")
		print "nCycPallet" + cmd37.get_value()
		cmd38= client.get_node("ns=4;s=GVL_MQTT.bCycTrigger")
		print "bCycTrigger" + cmd38.get_value()
		print "-----Productivity"
		cmd39= client.get_node("ns=4;s=GVL_MQTT.nTotalPallet")
		print "nTotalPallet" + cmd39.get_value()
		print "-----Quality"
		cmd40= client.get_node("ns=4;s=GVL_MQTT.nGoodPallet")
		print "nTotalPallet" + cmd40.get_value()
		print "-----Employee satisfaction"
		cmd41= client.get_node("ns=4;s=GVL_MQTT.sCell1Employee")
		print "nGoodPallet" + cmd40.get_value()
		"""

		
