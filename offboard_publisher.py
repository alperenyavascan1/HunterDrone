#!/usr/bin/env python2
#***************************************************************************
#
#   Copyright (c) 2015 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
#***************************************************************************/

#
# @author Andreas Antener <andreas@uaventure.com>
#
# The shebang of this file is currently Python2 because some
# dependencies such as pymavlink don't play well with Python3 yet.
from __future__ import division

PKG = 'px4'

import rospy
import math
import numpy as np



from geometry_msgs.msg import PoseStamped, Quaternion
from mavros_test_common import MavrosTestCommon
from pymavlink import mavutil
from six.moves import xrange
from std_msgs.msg import Header
from std_msgs.msg import String
from threading import Thread
from datetime import datetime
from tf.transformations import quaternion_from_euler
import time


hunterx=0
huntery=0
hunterz=0
rabbitx=0
rabbity=0
rabbitz=0


def checkdist():
    global rabbitx


    rabbitx=rabbitx-50
    distance=math.sqrt((hunterx-rabbitx)*(hunterx-rabbitx)+(huntery-rabbity)*(huntery-rabbity)+(hunterz-rabbitz)*(hunterz-rabbitz))
    rospy.loginfo("%f",distance)
    if(distance<1.0):
       rospy.signal_shutdown("close")




def hunterpose(datahunter):
    global hunterx,huntery,hunterz
    hunterx=datahunter.pose.position.x
    huntery=datahunter.pose.position.y
    hunterz=datahunter.pose.position.z



def rabbitpose(datarabbit):
    global rabbitx,rabbity,rabbitz
    rabbitx=datarabbit.pose.position.x
    rabbity=datarabbit.pose.position.y
    rabbitz=datarabbit.pose.position.z
    checkdist()




class MavrosOffboardPosctlTest(MavrosTestCommon):
    """
    Tests flying a path in offboard control by sending position setpoints
    via MAVROS.

    For the test to be successful it needs to reach all setpoints in a certain time.

    FIXME: add flight path assertion (needs transformation from ROS frame to NED)
    """
    def timeout_handler(signum,frame):   
        raise TimeoutException

    def setUp(self):
        super(MavrosOffboardPosctlTest, self).setUp()

        self.pos = PoseStamped()
        self.radius = 1

        self.pos_setpoint_pub = rospy.Publisher(
            '/uav0/mavros/setpoint_position/local', PoseStamped, queue_size=1)
        rospy.Subscriber('/uav1/mavros/local_position/pose', PoseStamped, hunterpose) 
        rospy.Subscriber('/uav0/mavros/local_position/pose', PoseStamped, rabbitpose) 
        self.sendmsg= 1
        # send setpoints in seperate thread to better prevent failsafe
        self.pos_thread = Thread(target=self.send_pos, args=())
        self.pos_thread.daemon = True
        self.pos_thread.start()
        self.pub = rospy.Publisher('chatter', String, queue_size=10)


    def tearDown(self):
        super(MavrosOffboardPosctlTest, self).tearDown()

    #
    # Helper methods
    #
    def send_pos(self):
        rate = rospy.Rate(10)  # Hz
        self.pos.header = Header()
        self.pos.header.frame_id = "base_footprint"

        while not rospy.is_shutdown():
            self.pos.header.stamp = rospy.Time.now()
            self.pos_setpoint_pub.publish(self.pos)
            try:  # prevent garbage in console output when thread is killed
                rate.sleep()
            except rospy.ROSInterruptException:
                pass

    def is_at_position(self, x, y, z, offset):
        """offset: meters"""
	now = datetime.now()
	current_time = now.strftime("%H:%M:%S")
        rospy.logdebug(
            "current position | x:{0:.2f}, y:{1:.2f}, z:{2:.2f}"+"Current Time =" +str(current_time).format(
                self.local_position.pose.position.x, self.local_position.pose.
                position.y, self.local_position.pose.position.z))

        desired = np.array((x, y, z))
        pos = np.array((self.local_position.pose.position.x,
                        self.local_position.pose.position.y,
                        self.local_position.pose.position.z))
        return np.linalg.norm(desired - pos) < offset

    def reach_position(self, x, y, z, timeout):
        """timeout(int): seconds"""
        # set a position setpoint
        self.pos.pose.position.x = x
        self.pos.pose.position.y = y
        self.pos.pose.position.z = z
	now = datetime.now()

	current_time = now.strftime("%H:%M:%S")
        rospy.loginfo(
            "attempting to reach position | x: {0}, y: {1}, z: {2} | current position x: {3:.2f}, y: {4:.2f}, z: {5:.2f}"+"Current Time =" +str(current_time).
            format(x, y, z, self.local_position.pose.position.x,
                   self.local_position.pose.position.y,
                   self.local_position.pose.position.z))

        # For demo purposes we will lock yaw/heading to north.
        yaw_degrees = 0  # North
        yaw = math.radians(yaw_degrees)
        quaternion = quaternion_from_euler(0, 0, yaw)
        self.pos.pose.orientation = Quaternion(*quaternion)

        # does it reach the position in 'timeout' seconds?
        loop_freq = 10  # Hz
        rate = rospy.Rate(loop_freq)
        reached = False
        for i in xrange(timeout * loop_freq):
            if self.is_at_position(self.pos.pose.position.x,
                                   self.pos.pose.position.y,
                                   self.pos.pose.position.z, self.radius):
                rospy.loginfo("position reached | seconds: {0} of {1}".format(
                    i / loop_freq, timeout))
                reached = True
                break
            pos = np.array((self.local_position.pose.position.x,
                self.local_position.pose.position.y,
                self.local_position.pose.position.z)).tostring()
            self.publish_pos()
            #self.pub.publish(pos)
            try:
                rate.sleep()
            except rospy.ROSException as e:
                self.fail(e)

        self.assertTrue(reached, (
            "took too long to get to position | current position x: {0:.2f}, y: {1:.2f}, z: {2:.2f} | timeout(seconds): {3}".
            format(self.local_position.pose.position.x,
                   self.local_position.pose.position.y,
                   self.local_position.pose.position.z, timeout)))

    #
    # Test method
    #
    def test_posctl(self):
        """Test offboard position control"""

        # make sure the simulation is ready to start the mission
        self.wait_for_topics(60)
        self.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND,
                                   10, -1)

        self.log_topic_vars()
        self.set_mode("OFFBOARD", 5)
        self.set_arm(True, 5)

        rospy.loginfo("run mission")
        positions = (
            (70, 0.00, 10),
            (69, 6.24, 12),
            (68, 8.72, 12),
            (67, 10.54, 12),
            (66, 12.00, 11),
            (65, 13.23, 10),
            (64, 14.28, 11),
            (63, 15.20, 12),
            (62, 16.00, 12),
            (61, 16.70, 12),
            (60, 17.32, 11),
            (59, 17.86, 10),
            (57, 18.73, 12),
            (50, 20.00, 12),
            (49, 19.97, 12),
            (44, 19.08, 12),
            (43, 18.73, 12),
            (42, 18.33, 11),
            (41, 17.86, 10),
            (40, 17.32, 11),
            (39, 16.70, 12),
            (38, 16.00, 12),
            (37, 15.20, 12),
            (36, 14.28, 11),
            (35, 13.23, 10),
            (34, 12.00, 11),
            (33, 10.54, 12),
            (32, 8.72, 12),
            (31, 6.24, 12),
            (30, 0.00, 11),
            (31, -6.24, 10),
            (32, -8.72, 11),
            (33, -10.54, 12),
            (34, -12.00, 12),
            (35, -13.23, 12),
            (36, -14.28, 11),
            (37, -15.20, 10),
            (38, -16.00, 11),
            (39, -16.70, 12),
            (40, -17.32, 12),
            (41, -17.86, 12),
            (42, -18.33, 11),
            (43, -18.73, 10),
            (44, -19.08, 11),
            (50, -20.00, 11),
            (56, -19.08, 11),
            (57, -18.73, 12),
            (58, -18.33, 12),
            (59, -17.86, 12),
            (60,-17.32, 11),
            (61, -16.70, 10),
            (62, -16.00, 11),
            (63, -15.20, 12),
            (64, -14.28, 12),
            (65, -13.23, 12),
            (66, -12.00, 11),
            (67, -10.54, 10),
            (68, -8.72, 11),
            (69, -6.24, 12),

        )
        while True:
            for i in xrange(len(positions)):
                self.reach_position(
                    positions[i][0],
                    positions[i][1],
                    positions[i][2],
                    300)
                self.publish_pos()
                time.sleep(0.2)
                

        self.set_mode("AUTO.LAND", 5)
        self.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND,
                                   45, 0)
        self.set_arm(False, 5)
    def publish_pos(self):
        t = time.time()%100
        pos = np.array((self.local_position.pose.position.x,
                        self.local_position.pose.position.y,
                        self.local_position.pose.position.z,t)
                        )
	now = datetime.now()

	current_time = now.strftime("%H:%M:%S")
        posstr = pos.tostring()


        if(self.sendmsg<100):
            self.pub.publish(posstr)
            rospy.loginfo("published "+str(pos)+"Current Time =" +str(current_time)+" "+str(self.sendmsg))
            self.sendmsg=self.sendmsg+1

        pass

if __name__ == '__main__':
    import rostest
    rospy.init_node('hostile', anonymous=True)

    rostest.rosrun(PKG, 'mavros_offboard_posctl_test',
                   MavrosOffboardPosctlTest)
