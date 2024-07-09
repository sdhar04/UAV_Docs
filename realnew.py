import rospy
import math
import time
import matplotlib.pyplot as plt
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32

class FCU_connection:

    def __init__(self):
        rospy.init_node('mavros_fcu')
        self.vel_pub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel_unstamped", Twist, queue_size=100)
        self.z_vel_pub=rospy.Publisher("/model_output_z_vel",Float32,queue_size=100)
        self.rate = rospy.Rate(20)  # 20 Hz

        self.laser_down_sub = rospy.Subscriber("/spur/laser/scan2", LaserScan, self.laser_down_callback)
        self.hz_laser_1 = rospy.Subscriber("/spur/laser/scan3", LaserScan, self.laser_hz1_callback)
        self.hz_laser_2 = rospy.Subscriber("/spur/laser/scan4", LaserScan, self.laser_hz2_callback)
        self.laser_front_sub = rospy.Subscriber("/spur/laser/scan", LaserScan, self.laser_front_callback)
        self.mapped_axes_sub=rospy.Subscriber('/mapped_axes', Float32MultiArray, self.mapped_axes_callback)

        self.min_distance=0
        
         
        #initialise front altitude variables
        self.farr=0
        self.farh=0
        self.fard=10
        
        # forward velocity
        self.f_vel=1

        #mapped axes parameters
        self.vel_x=0
        self.vel_y=0
        self.angular_vel_z=0

        # PID controller parameters
        if self.farh >=0:
        	self.Kp = 0.6*math.exp(self.f_vel)/self.fard
        else:
        	self.Kp = 0.2*math.exp(self.f_vel)/self.fard
        self.Ki = 0.05
        self.Kd = 0.10
        
        self.k1 = 0.0016
        self.k2 = 0.2
        
        # Target altitude
        self.target_altitude = 3.0
        self.prev_alt=0

        # Initialize bottom distance altitude
        self.bottom_altitude = 0.0
        self.bottomr=0
        self.bottom_angle=0
               
        # Variables for PID controller
        self.prev_error = 0.0
        self.integral = 0.0
        
        # arrays
        self.rangess_d=[]
        self.rangess_f=[]
        self.rangess=[]
        
        self.hz_array=[]
        self.hz1_ranges=[]
        self.hz2_ranges=[]
        
    def mapped_axes_callback(self, msg):
        # Print the received mapped axes values
        if(len(msg.data)!=0):
            self.vel_y=msg.data[1]
            self.vel_x=msg.data[3]
            self.angular_vel_z=msg.data[0]
        else:
            self.vel_x=0
            self.vel_y=0
            self.angular_vel_z=0

    def laser_down_callback(self, data):
        self.rangess_d=list(data.ranges)
        
    def laser_hz1_callback(self,data):
        self.hz1_ranges=list(data.ranges)

    def laser_hz2_callback(self,data):
        self.hz2_ranges=list(data.ranges)

    def laser_front_callback(self, data):
        self.rangess_f=list(data.ranges)            

    def pid_control(self):
        # Error calculation
        error = self.target_altitude 
    
        # Proportional term
        P = self.Kp * error
        # Integral term
        self.integral =(self.integral + error) * self.rate.sleep_dur.to_sec()
        I = self.Ki * self.integral
        # Derivative term
        derivative = (error - self.prev_error) / self.rate.sleep_dur.to_sec()
        D = self.Kd * derivative
        # PID output
        output = P + I + D
        
        # Update previous error for next iteration
        self.prev_error = error
        
        # Apply the output to velocity
        vel_msg = Twist()
        vel_msg.linear.x = self.vel_x
        # if self.fard<5:
        #     vel_msg.linear.y = self.f_vel+self.vel_y + (1/self.fard-1)*1
        # elif self.fard<3:
        #     vel_msg.linear.y = self.f_vel+self.vel_y + (1/self.fard-1)*2
        # else:
        #     vel_msg.linear.y = self.f_vel+self.vel_y
        self.hz_array.extend(self.hz1_ranges)
        self.hz_array.extend(self.hz2_ranges)
        if(len(self.hz_array) != 0):
            for i in range(len(self.hz_array)):
                if math.isinf(self.hz_array[i]) or math.isnan(self.hz_array[i]):
                    self.hz_array[i]=10
            self.min_distance=min(self.hz_array)
            f_rep = self.repulsive_potential(self.min_distance,5)
            vel_msg.linear.y = self.f_vel + f_rep
            if vel_msg.linear.y < 0:
                vel_msg.linear.y=0
        else:
            vel_msg.linear.y=self.f_vel
        vel_msg.linear.z = output
        vel_msg.angular.x = 0.0
        vel_msg.angular.y = 0.0
        vel_msg.angular.z = self.angular_vel_z
                
        print("output: ",output)
        print("hz_vel: ",vel_msg.linear.y)

        # Publish the velocity
        self.vel_pub.publish(vel_msg)
        self.z_vel_pub.publish(float(output))
    
    def repulsive_potential(self,near_d,q_star):
        k_r = 30
        U_rep = 1/2*k_r*(1/near_d - 1/q_star)**2
        del_near_d = near_d/math.sqrt(near_d)
        del_U_rep = k_r * (1/q_star - 1/near_d) * (1/math.pow(near_d,2)) * del_near_d
        return del_U_rep

    def calculator(self):

        # Initialize arrays to store sine and cosine values
        h_values = []
        d_values = []
        wh_values=[]
        self.hz_array=[]

        # Calculate angle increment
        angle_increment = (45 + 90) / len(self.rangess)

        # Iterate through each element of the array
        for i, distance in enumerate(self.rangess):
            # Convert index to angle
            angle = -90 + (i * angle_increment)

            # Check if the distance is infinity
            if math.isinf(distance):
                continue
            else:
                # Calculate sine and cosine values
                h_value = distance*math.sin(math.radians(angle))
                d_value = distance*math.cos(math.radians(angle))

                # Append values to new arrays
                h_values.append(h_value)
                d_values.append(d_value)
                
        if(len(h_values)==0):
            h_values.append(-5)
            d_values.append(1)
        
        # plt.clf()  # Clear previous plot
        # plt.plot(d_values, h_values, 'o')  # Plot new data
        # plt.xlabel('X Axis Label')
        # plt.ylabel('Y Axis Label')
        # plt.title('Plot of Y vs X')
        # plt.grid(True)  # Add grid lines
        # plt.pause(0.1)
        d = min(d_values)
        for i in range(0,len(h_values)):
            x=d_values[i]
            if (x<d+3):
                wh_values.append(h_values[i])
            else:
                wh_values.append((((self.k1*math.log(self.k2*self.f_vel))*math.pow(x-d-3,3))+1)*h_values[i])
        
        ind = 0    #ind variable to store the index of maximum value in the list
        max_element = wh_values[0]

        for i in range (1,len(wh_values)): #iterate over array
            if math.isinf(h_values[i]):
                continue
            elif wh_values[i] > max_element: #to check max value
                max_element = wh_values[i]
                ind = i
            
        return h_values[ind],d_values[ind]

    def maintain_altitude(self):
        time.sleep(1)
        plt.figure()
        while not rospy.is_shutdown():
            self.rangess=self.rangess_d+self.rangess_f
            self.farh,self.fard=self.calculator()
            
            self.target_altitude=self.farh+3
                
            print("fard: ",self.fard)
            print("farh: ",self.farh)
            print("tar alt: ",self.target_altitude)
            print(" ")
            
            self.pid_control()
            self.rate.sleep()


if __name__ == '__main__':
    fcu1 = FCU_connection()
    fcu1.maintain_altitude()



