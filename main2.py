import math
from math import atan2, sin, cos, radians
import time
import csv
import signal
import matplotlib.pyplot as plt 
import matplotlib.image as mpimg
from get_points_from_map import get_points_from_map
from write_to_arduino import send_arduino_cmd_motor,init_arduino_line
from convert_to_meter_space import convert_to_meter_space
import sys
import serial
import os
import time
import numpy as np
import sensors_feedback as sensors



class State():
    def __init__(self, x_, y_, theta_):
        if not (x_ == None or y_ == None or theta_ == None):
            self.x = x_
            self.y = y_
            self.theta = theta_
        else:
            self.x = 0
            self.y = 0
            self.theta = 0

    def __str__(self):
        return str(self.x)+","+str(self.y)+","+str(self.theta)
#R = 0.0325
#L = 0.1 
R = 1/150
L = 0.1     
class Controller():
    def __init__(self, start_, goal_, R_ = R, L_ = L, kP_left = 1,kP_right= 1, kI = 0.01, kD = 0.1, dT = 5, v=1.0):
        self.current = start_
        self.goal = goal_
        self.R = R_ #in meter
        self.L = L_ #in meter

        self.E = 0  #Cummulative error
        self.old_e = 0 # Previous error

        self.kP_left = kP_left
        self.kP_right = kP_right
        self.Ki = kI
        self.Kd = kD

        self.desiredV = v
        self.dt = dT #in second
        return
    #def please_calibrate_imu(self):

    def please_rectify(self,v,vmin,vmax):
        if v < vmin:
            v=vmin
        elif v > vmax:
            v=vmax
        else:
            v=v
        return v
    
    def uniToDiff(self, v, w):
        #vR = (2*v + w*self.L)/(2*self.R)
        #vL = (2*v - w*self.L)/(2*self.R)
        vMax=60
        vMin=0

        if w < 0:
            vL = vMax
            vR = abs(w)#((vMax-vMin)/(0+1))*(w+1)
        if w > 0:
            #vL = ((vMax-vMin)/(0-1))*(w-1)
            vL = abs(w)#((vMax-vMin)/(0+1))*(1-w)

            vR = vMax
        if w ==0:
            vL=vMax
            vR=vMax
        return vR, vL

    
    def diffToUni(self, vR, vL):
        v = self.R/2*(vR+vL)
        w = self.R/self.L*(vR-vL)
        return v, w
    def calibrate_target_angle(self,g_theta):
        if self.goal.x >= self.current.x and self.goal.y >= self.current.y:
            g_theta=90-g_theta
        elif self.goal.x <= self.current.x and self.goal.y >= self.current.y:
            if g_theta > 0:
                g_theta=g_theta-180
            g_theta=270+abs(g_theta)
        elif self.goal.x >= self.current.x and self.goal.y <= self.current.y:
            g_theta=90+abs(g_theta)
        elif self.goal.x <= self.current.x and self.goal.y <= self.current.y:
            g_theta=270-abs(g_theta)
        if g_theta > 180:
            g_theta=-(360-g_theta)
        return g_theta
            

    def iteratePID(self):
        #Difference in x and y
        #lat, lon=43.135636, 6.009978        #x-bulding
        #lat,lon = 43.1364982,6.0124686 #M BUILDING
        #lat,lon = 43.137408, 6.004399 #basic fit gym
        #lat,lon = 43.1364155,6.0083219 #estererl 2
        #ylGPS,xlGPS=43.133200, 6.006932
        #yuGPS,xuGPS=43.138136, 6.023519
        
        #self.goal.x, self.goal.y=mapping(lon,lat,xlGPS,ylGPS,xuGPS,yuGPS)
        d_x = self.goal.x - self.current.x
        d_y = self.goal.y - self.current.y

        #print(f"current x: {self.current.x},current y: {self.current.y}")
        #print(f"goal x: {self.goal.x},goal y: {self.goal.y}")
        bound=3 #in degrees
        vmax_left=60
        vmax_right=60
        vmin_left=0
        vmin_right=0
        #Angle from robot to goal
        g_theta = atan2(d_y, d_x)
        g_theta=math.degrees(g_theta)
        #print(f"g_theta: {g_theta}")
        g_theta=self.calibrate_target_angle(g_theta)
        self.g_theta=g_theta
        #Error between the goal angle and robot angle
        alpha = g_theta - self.current.theta
        #alpha = self.current.theta - g_theta
        self.alpha=alpha
        

        #alpha = g_theta - math.radians(90)
        #e = atan2(sin(alpha), cos(alpha))
        #print(f"alpha: {alpha}")
        if alpha < -bound or   alpha>180  :
            vR=vmax_right
            e=abs(alpha)
            e_P_left = e
            #e_I = self.E + e
            #e_D = e - self.old_e
            vL = vmax_left-self.kP_left*e_P_left #+ self.Ki*e_I + self.Kd*e_D
            vL=self.please_rectify(vL,vmin_left,vmax_left)
            #self.E = self.E + e
            #self.old_e = e
        elif alpha > bound or   alpha< -180 :
            vL=vmax_left
            e=abs(alpha)
            e_P_right = e
            #e_I = self.E + e
            #e_D = e - self.old_e
            vR = vmax_right-self.kP_right*e_P_right #+ self.Ki*e_I + self.Kd*e_D
            vR=self.please_rectify(vR,vmin_right,vmax_right)
            #self.E = self.E + e
            #self.old_e = e
        elif alpha > -bound and alpha < bound:
            vL=vmax_left
            vR=vmax_right


        return vL,vR

    def fixAngle(self, angle):
        return atan2(sin(angle), cos(angle))
        

    def makeAction(self, v, w):
        x_dt = v*cos(self.current.theta)
        y_dt = v*sin(self.current.theta)
        theta_dt = w

        self.current.x = self.current.x + x_dt * self.dt
        self.current.y = self.current.y + y_dt * self.dt
        self.current.theta = self.fixAngle(self.current.theta + self.fixAngle(theta_dt * self.dt))
        return
    

    def isArrived(self):
        #tolerance=25
        tolerance=5
        tolerance_theta=0.1
        #print("Arrive check:",str(abs(self.current.x - self.goal.x)), str(abs(self.current.y - self.goal.y)))
        if abs(self.current.x - self.goal.x) < tolerance and abs(self.current.y - self.goal.y) < tolerance or\
             (abs(self.current.x - self.goal.x) + abs(self.current.y - self.goal.y)< tolerance):
            return True
        else:
            return False
    

    def runPID(self, parkour=None):
        #x = [self.current.x]
        #y = [self.current.y]
        #theta = [self.current.theta]
        ws=[]
        x=[]
        y=[]
        theta=[]
        f = open('data_logger.csv', 'a')
        data_logger = csv.writer(f)
        try:
        
            arduino, data = init_arduino_line()
            
        except: #(RuntimeError, TypeError, NameError,ValueError):
            print("a7a arduino")
        try:
            #pass
            ser_gps, ser_imu = sensors.initialization()
            print("connected with GPS and IMU")
        except: #(RuntimeError, TypeError, NameError,ValueError):
            #pass
            print("a7a no connection with GPS and IMU")
        vL0=0
        vR0=0
        self.ref=0
        self.first_time=0
        
        while(not self.isArrived()) or manual_stop!=1:
            
            #print(f"V={v}")
            #self.makeAction(v, w)
            
            #current_x=self.current.x
            #current_y=self.current.y
            #current_theta=self.current.theta
            #print(f"current x {current_x}")
            try:
                pass
                #print(f)
                import a7a
                lat,lon,heading = sensors.get_sensors_data(ser_gps,ser_imu)
                #print(f"sensor data; lat:{lat},lon:{lon},heading:{heading}")
            except:
                #pass
                #heading=-(sensors.get_heading_angle(ser_imu)-90)
                heading=sensors.get_heading_angle(ser_imu)
                heading=-(heading-self.ref)
                #if heading < -180:
                if heading < 0:
                    heading=360+heading
                if heading > 180:
                    heading=-(360-heading)
              

                if self.first_time==0:
                    print("IMU: Wait, I am waking up")
                    time.sleep(5)
                
                
                    info = input("Is this north, sir?")
                    heading=sensors.get_heading_angle(ser_imu)
                    self.ref=heading
                    self.first_time=1

                
            #university
            #ylGPS,xlGPS=43.133200, 6.006932
            #yuGPS,xuGPS=43.138136, 6.023519

            #the lake
            xlGPS,ylGPS=-3.026018,48.207114
            xuGPS,yuGPS=-3.009049,48.213114
            try:
                current_x,current_y=mapping(lon,lat,xlGPS,ylGPS,xuGPS,yuGPS)
            except:
                print("a7a GPS is not running")
                #lat,lon = 43.1364155,6.0083219 #estererl 2
                #lat,lon = 43.1363754,6.0096848 #estererl 1
                lat,lon = 48.209619, -3.024597 #point A in the lake
                
                #current_x,current_y=lon,lat
                current_x,current_y=mapping(lon,lat,xlGPS,ylGPS,xuGPS,yuGPS)
            #current_x=lon
            #current_       y=lat
            
            current_theta=heading
            #print(f"current x {current_x}")
            self.current.x=current_x
            self.current.y=current_y
            self.current.theta=current_theta
            
            x.append(current_x)
            y.append(current_y)
            theta.append(current_theta)
            vL, vR = self.iteratePID()

            #vR,vL=self.uniToDiff(v, w)
            #ws.append(w)
            vR=int(vR)
            vL=int(vL)
            #if vR != vssR0 and vL != vL0:
            #print("Bonjour")
            #print(f"vL:{vL},vR:{vR}")
            
            #
            send_arduino_cmd_motor(arduino,vL,vR)
            #print("Saluut")
            vR0=vR
            vL0=vL

            
            #time.sleep(0.5)

            
            
            
           
            
            #print(f"x: {round(self.current.x,2)},y: {round(self.current.y,2)},theta: {round(self.current.theta,2)},w: {round(w,5)},v: {round(v,2)},vR: {vR},vL: {vL}")
            #print(f"x: {round(current_x,2)},y: {round(current_y,2)},theta: {round(current_theta,2)},vR: {vR},vL: {vL}")
            if vL > vR:
                turning= "turning right"
            elif vL < vR:
                turning= "turning left"
            else:
                turning = "moving forward"
            print(f"target angle:{round(self.g_theta,3)}, heading: {round(current_theta,3)},error: {round(self.alpha,3)};vR: {vR},vL: {vL}, {turning}")

            row=['x',round(self.current.x,2),'y',round(self.current.y,2),'theta',round(self.current.theta,2),'vR',vR,'vL',vL,turning]
            data_logger.writerow(row)
            if parkour:

                parkour.drawPlot(x, y, theta)
                #time.sleep(self.dt)

            # Print or plot some things in here
            # Also it can be needed to add some max iteration for error situations and make the code stable.
            #print(self.current.x, self.current.y, self.current.theta)
        #print(ws)
        #send_arduino_cmd_motor(arduino, 0, 0 )
        f.close()
        return x, y, theta


def trackRoute(start, targets):
    current = start
    x = []
    y = []
    theta = []
    f = open('data_logger.csv', 'w')
    data_logger = csv.writer(f)
    f.close()
    for target in targets:
        controller = Controller(current, target)
        x_, y_, theta_ = controller.runPID()
        x.extend(x_)
        y.extend(y_)
        theta.extend(theta_)
        current = controller.current
    return x, y, theta

def mapping(xGPS,yGPS,xlGPS,ylGPS,xuGPS,yuGPS):
    im =mpimg.imread("map.png")
    # <class 'numpy.ndarray'>
    xl=0
    yl=0
    xu=im.shape[1]
    yu=im.shape[0]
    #xGPS,yGPS = convert_to_meter_space(xGPS,yGPS)
    #xlGPS,ylGPS = convert_to_meter_space(xlGPS,ylGPS)
    #xuGPS,yuGPS = convert_to_meter_space(xuGPS,yuGPS)

    xMapped=((xGPS-xlGPS)*(xu-xl))/(xuGPS-xlGPS)
    yMapped=((yGPS-ylGPS)*(yu-yl))/(yuGPS-ylGPS)
    
    #xMapped = xGPS-xlGPS
    #yMapped = yGPS-ylGPS
    return xMapped,yMapped

def convert_points2GPS(xp,yp,xlGPS,ylGPS,xuGPS,yuGPS):
    im =mpimg.imread("map.png")
    # <class 'numpy.ndarray'>
    xl=0
    yl=0
    xu=im.shape[1]
    yu=im.shape[0]
    xMapped=((xp-xl)*(xuGPS-xlGPS))/(xu-xl)+xlGPS
    yMapped=((yp-yl)*(yuGPS-ylGPS))/(yu-yl)+ylGPS
    return xMapped,yMapped
points=get_points_from_map()
#Starting point of the code
def main():
    # Define dt time, create controller, define start and goal points
    # In every iteration, get an action from PID and make the action,
    # after that, sleep for dt. Repeat that loop until reaching the goal state.
    simulateParkour=True

    #The lake
    xlGPS,ylGPS=-3.026018,48.207114
    xuGPS,yuGPS=-3.009049,48.213114
    
    
    #Toulon
    #ylGPS,xlGPS=43.129895, 5.989882
    #yuGPS,xuGPS=43.143249, 6.032166

    #University
    #ylGPS,xlGPS=43.133200, 6.006932
    #yuGPS,xuGPS=43.138136, 6.023519





    #xlGPS,ylGPS=-10,-5
    #xuGPS,yuGPS=10,5
    
    #start = State(-20.0, 15.0, math.radians(90))
    #ystart,xstart=43.136342, 6.009679
    #ystart,xstart = 43.135660, 6.009961 # X building 
    
    #ystart,xstart = 43.136609, 6.013436 # M building on the right
    #ystart,xstart = 43.136501, 6.011888 # M building on the left
    ystart,xstart = 43.136449, 6.008503
    xMapped_start,yMapped_start=mapping(xstart,ystart,xlGPS,ylGPS,xuGPS,yuGPS)
    #xMapped_start,yMapped_start=mapping(-10,-5,xlGPS,ylGPS,xuGPS,yuGPS)
    start = State(xMapped_start, yMapped_start, 0)
     
    #targets = [State(0, 20, 0),  State(20, 10, 0), State(0, 5, 0), State(-10, -15, 0), State(0, -10, 0), State(8, -10, 0) ]
    
    #targets = [State(10, 10, 0)]

    xMapped1,yMapped1=mapping(-3.017740,48.209594,xlGPS,ylGPS,xuGPS,yuGPS)
    xMapped2,yMapped2=mapping(-3.024597,48.209619,xlGPS,ylGPS,xuGPS,yuGPS)
    #xMapped1,yMapped1=mapping(-5,-5,xlGPS,ylGPS,xuGPS,yuGPS)
    #xMapped2,yMapped2=mapping(3,4,xlGPS,ylGPS,xuGPS,yuGPS)
    #targets = [State(xMapped1, yMapped1, 0),State(xMapped2, yMapped2, 0) ]
    targets=[]
    for i in range(len(points)):
        xp=points[i][0]
        yp=points[i][1]
        xMapped0,yMapped0=convert_points2GPS(xp,yp,xlGPS,ylGPS,xuGPS,yuGPS)
        xMapped,yMapped=mapping(xMapped0,yMapped0,xlGPS,ylGPS,xuGPS,yuGPS)
        targets.append(State(xMapped, yMapped, 0))
        
        #print(f"{xMapped},{yMapped}")


    
    #simulateParkour=False
    if simulateParkour:
        from Parkour import Parkour

        x, y, theta = trackRoute(start, targets)
        print(len(x))
        print(len(y))
        print(len(theta))

    
        parkour = Parkour()
        parkour.drawPlot(x, y, theta, show=True)
if __name__ == "__main__":
    main()