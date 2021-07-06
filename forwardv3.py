#!/usr/bin/env python

import math
import rospy
import time

from mavros_msgs.msg import MountControl
from geometry_msgs.msg import PoseStamped, Quaternion, TwistStamped, Twist
from std_msgs.msg import Header
from std_msgs.msg import String

offset_data = [0.0, 0.0, -1, 0.0]
regions=[6,6,-1]
search=0
alphadrone=0
betadrone=0 
alphadrone1=1
betadrone1=0
alphagimbal=0
betagimbal=0
found=True
rabbitx=0
rabbity=0
rabbitz=0

def callback1(data):
    global new_offset_arrived
    global offset_data
    global search
    searchold=offset_data[0]
    new_offset_data= True
    offset_data[0] = float(data.data.split(' ')[0])
    offset_data[1] = float(data.data.split(' ')[1])
    offset_data[2] = float(data.data.split(' ')[2])
    offset_data[3] = float(data.data.split(' ')[3])
    regions.append(float(data.data.split(' ')[2]))
    



def callpose(data1):
    global alphadrone
    global betadrone
    global alphadrone1
    global betadrone1
    alphadrone1=1
    betadrone1=0
    test=data1.pose.orientation.x*data1.pose.orientation.y +data1.pose.orientation.z*data1.pose.orientation.w
    unit=data1.pose.orientation.x*data1.pose.orientation.x +data1.pose.orientation.y*data1.pose.orientation.y+data1.pose.orientation.z*data1.pose.orientation.z+data1.pose.orientation.w*data1.pose.orientation.w
    yaw=math.asin(2*test/unit)
    alpha1=alphadrone1
    alphadrone1=alphadrone1*math.cos(yaw)
    if(data1.pose.orientation.z>0.5 or data1.pose.orientation.z<-0.5):
       alphadrone1=-1*alphadrone1
    betadrone1=alpha1*math.sin(yaw)    


    wdegree=2*math.acos(data1.pose.orientation.w)
    zdegree=2*math.asin(data1.pose.orientation.z)
    k=0
    alphadrone=math.cos(wdegree)
    
    if((math.degrees(wdegree)<180 and zdegree<0) or (math.degrees(wdegree)>180 and zdegree>0)):   
       betadrone=-1*math.sqrt(1-(alphadrone*alphadrone))
    else:
       betadrone=math.sqrt(1-(alphadrone*alphadrone))


def rabbitpose(datarabbit):
    global rabbitx,rabbity,rabbitz
    rabbitx=datarabbit.pose.position.x
    rabbity=datarabbit.pose.position.y
    rabbitz=datarabbit.pose.position.z




def move_forward():
    searched_angle = 0
    pub = rospy.Publisher('/uav1/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
    gimbal_setangle_pub = rospy.Publisher('/uav1/mavros/mount_control/command', MountControl, queue_size=10)
    rospy.init_node('velocity', anonymous=True)
    rospy.Subscriber('/uav1/mavros/local_position/pose', PoseStamped, callpose) 
    rospy.Subscriber("twistexample", String, callback1)
    rospy.Subscriber('/uav0/mavros/local_position/pose', PoseStamped, rabbitpose) 
    pos_setpoint_pub = rospy.Publisher('/uav1/mavros/setpoint_position/local', PoseStamped, queue_size=1)
    msg = TwistStamped()
    rate = rospy.Rate(10) # 10hz
    msg.header=Header()
    msg.twist=Twist()
    gimbal_message = MountControl()
    gimbal_message.header = Header()

    gimbal_message.mode = 2
    pos = PoseStamped()
    pos.header = Header()
    a=0
    I=0.0
    e_prev=0
    Mv=0.0
    Iang=0
    eang_prev=0
    Mvang=0
    t=0
    gimbal_yaw=0
    gimbal_pitch=0
    xspeed=0
    yspeed=0
    lastRegion=6
    k=0
    j=0
    while not rospy.is_shutdown():
       msg.header.stamp= rospy.Time.now()
       msg.header.seq=1
       gimbal_message.header.stamp= rospy.Time.now()
       gimbal_message.header.seq=1
       pos.header.frame_id = "base_footprint"
       pos.header.stamp = rospy.Time.now()

     
       gimbal_message.roll = 0.0
       rospy.loginfo("%f",rabbitz)
       if(rabbitz<2):
          pos.pose.position.x=0
          pos.pose.position.y=0
          pos.pose.position.z=3
          pos_setpoint_pub.publish(pos)
          rate.sleep()  
          time.sleep(0.33)


       else:
        if(offset_data[0] == 0.0):
            for i in reversed(regions):

                if(i != -1):
                    lastRegion = i
                    break
                k=0
            while(offset_data[2] == -1):
             k=k+1
             if(k<5):
                rospy.loginfo("pre-search")
                j=0
             else:
                if(j>35):
                   lastRegion=6
                   gimbal_pitch=0
                j=j+1
                rospy.loginfo("search")
                if(lastRegion == 1):
                    gimbal_pitch= (gimbal_pitch+5) %45
                    gimbal_yaw -= 10              
                elif(lastRegion == 2):
                    gimbal_pitch= (gimbal_pitch+5) %45
                elif(lastRegion == 3):
                    gimbal_pitch= (gimbal_pitch+5) %45
                    gimbal_yaw += 10
                elif(lastRegion == 4):
                    gimbal_yaw -= 10
                elif(lastRegion == 5):
                    gimbal_yaw -= 10
                elif(lastRegion == 6):
                    gimbal_yaw += 10
                elif(lastRegion == 7):
                    gimbal_yaw -= 10
                    gimbal_pitch = max(gimbal_pitch-5, -45)
                elif(lastRegion == 8):
                    gimbal_pitch = max(gimbal_pitch-5, -45)
                    gimbal_yaw += 10
                elif(lastRegion == 9):
                    gimbal_pitch = max(gimbal_pitch-5, -45)
                    gimbal_yaw += 10
                else:
                    gimbal_yaw += 10
                rospy.loginfo("%f",lastRegion)
                
                msg.twist.linear.x = msg.twist.linear.x/1.3;
                msg.twist.linear.y = msg.twist.linear.y/1.3;
                msg.twist.linear.z = 0;
                msg.twist.angular.x = 0.0
                msg.twist.angular.y = 0.0
                msg.twist.angular.z = 0
                gimbal_message.pitch = gimbal_pitch
                gimbal_message.yaw = gimbal_yaw
             gimbal_setangle_pub.publish(gimbal_message)
             pub.publish(msg)
             rate.sleep()  
             time.sleep(0.33)



        if(offset_data[0]!=0): 
           
            rospy.loginfo("track")         
            e=gimbal_pitch
            P=0.05*e
            I=I+0.002*e*(0.33)
            D=0.001*(e-e_prev)/0.33
            Mv = P + I + D
            gimbal_pitch+=(offset_data[1]-0.5)*-30
            xspeed=alphadrone1*math.cos(math.radians(-1*gimbal_yaw))-betadrone1*math.sin(math.radians(-1*gimbal_yaw))
            yspeed=alphadrone1*math.sin(math.radians(-1*gimbal_yaw))+betadrone1*math.cos(math.radians(-1*gimbal_yaw))
            msg.twist.linear.x = 5*(xspeed)*(0.5-offset_data[3]);
            msg.twist.linear.y = 5*(yspeed)*(0.5-offset_data[3]);
            msg.twist.linear.z = Mv
            e_prev=e        

            eang=(gimbal_yaw%360)
            if(eang>180):
               eang=eang-360
            Pang=0.003*eang
            Iang=Iang+0.0001*eang*0.33
            Dang=0.0002*(eang-eang_prev)/0.33 
            Mvang=Pang+Iang+Dang
            gimbal_yaw+=(offset_data[0]-0.5)*50
            msg.twist.angular.x = 0.0
            msg.twist.angular.y = 0.0
            msg.twist.angular.z = -1*Mvang
            eang_prev=eang
        
            gimbal_message.pitch = gimbal_pitch
            gimbal_message.yaw = gimbal_yaw
            pub.publish(msg)
            gimbal_setangle_pub.publish(gimbal_message)
	
            rate.sleep()
            time.sleep(0.33)





if __name__ == '__main__':
    try:
        move_forward()
    except rospy.ROSInterruptException:
        pass
