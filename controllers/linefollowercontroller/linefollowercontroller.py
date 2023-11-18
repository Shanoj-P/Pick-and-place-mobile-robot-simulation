"""linefollowercontroller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Camera,DistanceSensor,Motor
import time
import cv2
import numpy as np

if __name__ == '__main__':

    # create the Robot instance.
    robot = Robot()
    
    # get the time step of the current world.
    timestep = 32
    
   #initialize wheel motors
    r1 = robot.getDevice('R1')
    r2 = robot.getDevice('R2')
    l1 = robot.getDevice('L1')
    l2 = robot.getDevice('L2')
    
       
    r1.setPosition(float('inf'))
    r2.setPosition(float('inf'))
    l1.setPosition(float('inf'))
    l2.setPosition(float('inf'))
    
    #initialise range sensors
    ir1 = robot.getDevice('IR1')
    ir2 = robot.getDevice('IR2')
    ir3 = robot.getDevice('IR3')
    ir4 = robot.getDevice('IR4')
    ir5 = robot.getDevice('IR5')
    
    dst = robot.getDevice('dstSensor')
    dst1 = robot.getDevice('dstSensor1')
    
    ir1.enable(timestep)
    ir2.enable(timestep)
    ir3.enable(timestep)
    ir4.enable(timestep)
    ir5.enable(timestep)
    
    dst.enable(timestep)
    dst1.enable(timestep)
    
    # initialise arm motors
    arm1 = robot.getDevice('link1')
    arm1.setPosition(float('inf'))
    arm1.setVelocity(0)
    arm2 = robot.getDevice('joint1')
    arm2.setPosition(float('inf'))
    arm2.setVelocity(0)
    arm3 = robot.getDevice('joint2')
    arm3.setPosition(float('inf'))
    arm3.setVelocity(0)
    grJoint = robot.getDevice('gripperJoint')
    grJoint.setPosition(float('inf'))
    grJoint.setVelocity(0)
    grJoint1 = robot.getDevice('gripperJoint1')
    grJoint1.setPosition(float('inf'))
    grJoint1.setVelocity(0)
    grJoint2 = robot.getDevice('gripperJoint2')
    grJoint2.setPosition(float('inf'))
    grJoint2.setVelocity(0)
    
    # initialise camera
    camera = robot.getDevice('camera')
    camera.enable(10)

    #initialise variables
    max_speed = 100
    cSpeed = 5
    rsp = 0
    lsp = 0
    p = 0
    i = 0
    d = 0
    previousError = 0
    t_end = time.time()+9.3
    delay = 0
    delay1 = 0
    delay2 = 0
    delay3 = 0
    object_detected = False
    pocket = 0
    
    while time.time() < t_end-8 :
        r1.setVelocity(rsp)
        r2.setVelocity(rsp)
        l1.setVelocity(lsp)
        l2.setVelocity(lsp)
        
    
    
            
       
    
    # - perform simulation steps until Webots is stopping the controller
    while robot.step(timestep) != -1:
        # calibrate sensor
        if time.time() > t_end-8 and time.time() < t_end-7.9:
            ir1Min = ir1.getValue()
            ir2Min = ir2.getValue()
            ir3Min = ir3.getValue()
            ir4Min = ir4.getValue()
            ir5Min = ir5.getValue()
            ir1Max = ir1.getValue()
            ir2Max = ir2.getValue()
            ir3Max = ir3.getValue()
            ir4Max = ir4.getValue()
            ir5Max = ir5.getValue()
            print('initialisation completed')
            
        
        if t_end > time.time() and time.time() >t_end-8:
            r1.setVelocity(-15)
            r2.setVelocity(-15)
            l1.setVelocity(15)
            l2.setVelocity(15)
            value_Ir1 = ir1.getValue()
            value_Ir2 = ir2.getValue()
            value_Ir3 = ir3.getValue()
            value_Ir4 = ir4.getValue()
            value_Ir5 = ir5.getValue()
            
            if ir1Min > value_Ir1:
                ir1Min = value_Ir1
            if ir1Max < value_Ir1:
                ir1Max = value_Ir1
                
            if ir2Min > value_Ir2:
                ir2Min = value_Ir2
            if ir2Max < value_Ir2:
                ir2Max = value_Ir2
                
            if ir3Min > value_Ir3:
                ir3Min = value_Ir3
            if ir3Max < value_Ir3:
                ir3Max = value_Ir3
                
            if ir4Min > value_Ir4:
                ir4Min = value_Ir4
            if ir4Max < value_Ir4:
                ir4Max = value_Ir4
                
            if ir5Min > value_Ir5:
                ir5Min = value_Ir5
            if ir5Max < value_Ir5:
                ir5Max = value_Ir5
            
        
        
        # calculate threshold value
        thresholdIr1 = (ir1Min + ir1Max) / 2  
        thresholdIr2 = (ir2Min + ir2Max) / 2  
        thresholdIr3 = (ir3Min + ir3Max) / 2  
        thresholdIr4 = (ir4Min + ir4Max) / 2  
        thresholdIr5 = (ir5Min + ir5Max) / 2             
        print(thresholdIr1)
                
        value_Ir1 = ir1.getValue()
        value_Ir2 = ir2.getValue()
        value_Ir3 = ir3.getValue()
        value_Ir4 = ir4.getValue()
        value_Ir5 = ir5.getValue()
        
        image = camera.getImage()
    
        # Get image properties
        width = camera.getWidth()
        height = camera.getHeight()
    
        # Convert the image to a NumPy array
        np_arr = np.frombuffer(image, dtype=np.uint8)
    
        # Reshape the array to image dimensions
        image_bgr = np_arr.reshape((height, width, 4))[:, :, :3]
    
        # Convert the image to HSV color space
        hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)
    
        # Define lower and upper bounds for green color in HSV
        lower_green = np.array([40, 40, 40])
        upper_green = np.array([80, 255, 255])
    
        # Create a mask using the green color range
        mask = cv2.inRange(hsv, lower_green, upper_green)
    
        # Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        image_bgr_um = cv2.UMat(image_bgr)
    
        # Draw contours on the original image
        cv2.drawContours(image_bgr_um, contours, -1, (0, 255, 0), 2)
        image_bgr = cv2.UMat.get(image_bgr_um)
        green_detected = len(contours) > 0
        print("Green Color Detected:", green_detected)
        
        # start pick and place algorithm
        if green_detected is True:
            obstDist = dst.getValue()
            object_area = cv2.contourArea(contours[0])
            object_size = np.sqrt(object_area)
            print(obstDist)
            if obstDist < 415:
                r1.setVelocity(0)
                r2.setVelocity(0)
                l1.setVelocity(0)
                l2.setVelocity(0)
                arm2.setPosition(-0.5)
                arm2.setVelocity(1)
                arm3.setPosition(0.16) 
                arm3.setVelocity(1)
                if delay == 0:
                    delay = time.time()+3
                if time.time() > delay and time.time() < delay + 3:
                    grJoint1.setPosition((object_size/180))
                    grJoint1.setVelocity(1.5)
                    grJoint2.setPosition((-object_size/180))
                    grJoint2.setVelocity(1.5)
                if time.time() > (delay+3):
                    arm2.setPosition(0.2)
                    arm2.setVelocity(4)
                    pocket = 1
                     
                continue
                
        if pocket == 1:
            stnDist = dst1.getValue()
            print('stnDtist {}',stnDist)
            if stnDist < 1000:
                r1.setVelocity(0)
                r2.setVelocity(0)
                l1.setVelocity(0)
                l2.setVelocity(0)
                arm1.setPosition(1.45)
                arm1.setVelocity(2)
                if delay == 0:
                    delay = time.time()+4
                if time.time() > delay and time.time() < delay + 4:
                    grJoint1.setPosition(0)
                    grJoint1.setVelocity(1.5)
                    grJoint2.setPosition(0)
                    grJoint2.setVelocity(1.5)
                if time.time() > (delay+4) and time.time() < delay + 7:
                    arm3.setPosition(0)
                    arm3.setVelocity(2.5)
                    arm1.setPosition(0)
                    arm1.setVelocity(2.5)
                    pocket = 0
                continue
                
        print('pocket',pocket)
                
             

    
        # line follower algorithm
        if value_Ir1 > thresholdIr1 and (value_Ir5 < thresholdIr5) and t_end < time.time() :
            lsp = 20
            rsp = -50
            print('right')
            r1.setVelocity(rsp)
            r2.setVelocity(rsp)
            l1.setVelocity(lsp)
            l2.setVelocity(lsp)
        elif value_Ir1 < thresholdIr1 and (value_Ir5 > thresholdIr5) and t_end < time.time():
            lsp = -50
            rsp = 20
            print('left')
            r1.setVelocity(rsp)
            r2.setVelocity(rsp)
            l1.setVelocity(lsp)
            l2.setVelocity(lsp)
        elif t_end < time.time():
            kp = 0.1
            kd = 0
            ki = 0
            error = value_Ir2 - value_Ir4
            p = error
            i = i + error
            d = error - previousError
            pid = ((kp*p)+(ki*i)+(kd*d))
            previousError = error
            print(pid)
            lsp = cSpeed + pid
            rsp = cSpeed - pid
            
            if lsp > max_speed:
                lsp = max_speed
            elif lsp < 0:
                lsp = 0
            if rsp > max_speed:
                rsp = max_speed
            elif rsp < 0:
                rsp = 0
            print('left speed: {}, right speed: {}'.format(lsp,rsp))
        
            r1.setVelocity(rsp)
            r2.setVelocity(rsp)
            l1.setVelocity(lsp)
            l2.setVelocity(lsp)
        delay = 0
   
    