"""
Honda Odyssey is the cars name
August Horii
ATCS PROJECT for a self driving car
"""

#This is just for myself: when youre pulling or commiting on the new device, use 

#git add .
#git commit -m "Updated robot sensors and logic"
#git push
#to commit

#git pull
#to get latest code


import math
from vehicle import Driver
from controller import Keyboard, Lidar, Camera




#Constants
TIME_STEP = 10
MAX_SPEED = 150.0
TURN_ANGLE = 0.5

#Initialization
driver = Driver()
driver.setSteeringAngle(0.0)
driver.setCruisingSpeed(0.0)

keyboard = Keyboard()
keyboard.enable(TIME_STEP)


#Initialization of the Lidar sensor
lidar = driver.getDevice("lidar") 
if lidar is not None:
    lidar.enable(TIME_STEP)
    lidar.enablePointCloud()
else:
    print("Twin your lidar is NOT there")

#The Initialization of the Right Camera 
camera = driver.getDevice("camera")
if camera is not None:
    camera.enable(TIME_STEP)
else:
    print("Your right camera is NOT plugged in Twin")


#The Initialization of the left Camera
Left_Camera = driver.getDevice("Left_Camera")
if Left_Camera is not None: 
    Left_Camera.enable(TIME_STEP)
else:
    print("Your left camera is NNOT plugged in Twin")

#Goal: Implementing the a second camera into autopilot by possibly adding a WAYPOINT


#to start in manual mode:
current_speed = 0.0
autodrive = False
last_error = 0.0
last_steering = 0.0
integral = 0.0
last_average_x = 0.0
was_blind = False 
last_visible_cameras = 0
#Adding a new variable: angle filter buffer
#angle_filter_buffer = [0.0, 0.0, 0.0]
angle_filter_buffer = [0.0, 0.0, 0.0]

#------------------------------------------------------
# TRAFFIC LIGHTS
#______________________________________________________
            
#F'n detect_traffic_light(image, width, height):

    #set red_pixels = 0
    #set green_pixels = 0
    #set yellow_pixels = 0
    #Only scan the top 30% of the image IF I dont add a new camera
    #for each y from 0 to height * 0.3:
        #for each x from 0 to width:
            #get r, g, b values of pixel at (x, y)
            #Check what color they are:
                    
            #if pixel is very red (high R, low G, low B):
                #red_pixels += 1
            #elif pixel is very green (high G, low R, low B):
                #green_pixels += 1
            #elif pixel is very yellow (high R AND high G, low B):
                #yellow_pixels += 1
            
    #what light we see based on pixel count
    #need enough pixels to avoid false positives
    
    #if red_pixels > 20:
        #return "RED"
    #elif green_pixels > 20:
        #return "GREEN"
    #elif yellow_pixels > 20:
        #return "YELLOW"
    #else:
        #return None  # no light detected         



print("Use the up/down/left/right buttons to move")
print("Press A to start the AUTOPILOT")





#Main Loop
while driver.step() != -1:
    key = keyboard.getKey()

    
    current_steering = 0.0
    #Identifying keys
    up = False; down = False; right = False; left = False  
    
    while key != -1:
        #Adding purpose to these delinquites
        if key == Keyboard.UP: up = True
        if key == Keyboard.DOWN: down = True
        if key == Keyboard.LEFT: left = True
        if key == Keyboard.RIGHT: right = True
        #Toggle on and off for the autopilot:
        if key == ord('A'): autodrive = not autodrive
        key = keyboard.getKey()
    
    if not autodrive:
        #boring manual coding part
        if up:
            current_speed += 5
            if current_speed > MAX_SPEED:
                current_speed = MAX_SPEED
        elif down:
            current_speed -= 5
            if current_speed < -MAX_SPEED:
                current_speed = -MAX_SPEED
        else:
            current_speed *= 0.9
            if abs(current_speed) < 0.1:
                current_speed = 0.0 
    
        
        if left:
            current_steering = -TURN_ANGLE
        elif right:
            current_steering = TURN_ANGLE

    else:
    
    #DUAL CAMERA LANE KEEPING
        current_speed = 10.0
        
        error = 0.0
        visible_cameras = 0
        width = camera.getWidth()
        height = camera.getHeight()
        start_y = int(height * 0.4)
        
        
    #Preconditions:
        #The vehicle is in autopilot mode
        #Both camera and left camera are enabled and returning image arrays
        #start_y is defined to prevent vanishing point confusions
    #Postconditions:
        #visible_cameras has a count 0-2 of how many lane lines were found
        #error has the averaged offset of the vehicle to the visible lines
        #if lines are visible, PID math results in the current_steering
        #if no lines are there, then like the code before, the current steering results in last_steering
        
        #SET total_error = 0, visible_cameras = 0
        
        #PROCESS RIGHT CAMERA:
            #scan the bottom right like before
            #Code stays the same as previous
        pixel_count_right = 0
        sum_x_r = 0
        if camera is not None:
            image_r = camera.getImageArray()  
            
            #if result is "RED":
                #brake fully
                #set speed to 0
                #print "RED LIGHT"
    
            #elif result is "YELLOW":
                #reduce speed by by half
                #print "YELLOW LIGHT"
    
            #elif result is "GREEN":
                #continue with speed
                #print "GREEN LIGHT"
                    
            for y in range(start_y, height, 2):
                for x in range(int(width * 0.5), width, 2):
                    r, g, b = image_r[x][y]
                    is_white = (r + g + b) > 450 and abs(r - g) < 30 and abs(r - b) < 30
                    if is_white:
                        sum_x_r += x
                        pixel_count_right += 1  
          
                
            if pixel_count_right > 0:
                avg_x_r = sum_x_r / pixel_count_right
                target_x_r = width * 0.77
                error += (avg_x_r - target_x_r) / width
                
                if avg_x_r > width * 0.85:
                    error *= 1.5
                
                visible_cameras += 1
                last_average_x = avg_x_r
        #PROCESS LEFT CAMERA:
            #Continue the code with the Right camera and average out error
        pixel_count_left = 0
        sum_x_l = 0
        if Left_Camera is not None:
            image_l = Left_Camera.getImageArray()
            for y in range(start_y, height, 2):
                for x in range(0, int(width * 0.4), 2):
                    r, g, b = image_l[x][y]
                    is_white = (r + g + b) > 450 and abs(r - g) < 30 and abs(r - b) < 30
                    if is_white:
                        sum_x_l += x
                        pixel_count_left += 1

            
            if pixel_count_left > 0:
                avg_x_l = sum_x_l / pixel_count_left
                target_x_l = width * 0.15
                error += (avg_x_l - target_x_l) / width
                visible_cameras += 1    
        #DUAL PID
            #If visible_cameras > 0:
            #average error = total_error / visible_cameras
            #Apply the pid formula to error to get the steering angle just like before
            #Possibly reduce the speed on each corner depending on later tests
            #save the errors for memory
        if visible_cameras > 0:
            error = error / visible_cameras
            #look only 3 frame-angles to smoothe out movement of the car
            angle_filter_buffer.pop(0)
            angle_filter_buffer.append(error)
            error = sum(angle_filter_buffer) / 3.0
            
            if was_blind or visible_cameras != last_visible_cameras:
                last_error = error
                was_blind = False
            if (error > 0) != (last_error > 0):
                integral = 0.0
            integral += error
            integral = max(min(integral, 30.0), -30.0) 
            integral *= 0.9
            #Turn
            p_term = error * 0.4
            #memory
            i_term = integral * 0.05
            #overshoot
            d_term = (error - last_error) * 3.0
            
            current_steering = p_term + i_term + d_term
            
            if current_steering - last_steering > 0.15:
                current_steering = last_steering + 0.15
            elif current_steering - last_steering < -0.15:
                current_steering = last_steering - 0.15
                
            turn_factor = 1.0 - abs(current_steering) * 1.2
            current_speed = max(15.0, 20.0 * turn_factor)
            last_error = error
            last_steering = current_steering
            driver.setBrakeIntensity(0.0)
            
            pos_r = (avg_x_r / width) if pixel_count_right > 0 else 0.0
            pos_l = pos_l = (avg_x_l / width) if (pixel_count_left > 0 and pixel_count_right == 0) else 0.0
            print(f"Cameras active: {visible_cameras} ||| L_Line is at: {pos_l:.2f} ||| R_Line is at: {pos_r:.2f} ||| Steer: {current_steering:.2f}")

            
        #ELSE: for the blind spots
            #Hold steering wheel to the last known location/ange
            #set constant speed for recovery
        else:
            
            was_blind = True
            last_visible_cameras = 0
            driver.setBrakeIntensity(0.0)
            if pos_r > 0.5:
                last_loc = 1
            else: 
                last_loc = -1
            current_steering = abs(last_steering) * 20
            current_speed = 10.0
            
            
            #target_x_r = width * 0.75
            #displacement = (last_average_x - target_x_r) / width
            #recovery_steering = last_steering + (displacement * 1.5)
            #recovery_steering = max(-0.5, min(0.5, recovery_steering))
            #current_speed = max(5.0, 15.0 - abs(displacement) * 30.0)
            
            #current_steering = recovery_steering
            
             
            print(f"Cameras active: {visible_cameras} ||| Line lost ||| L_Line is at: {pos_l:.2f} ||| R_Line is at: {pos_r:.2f} ||| Steer: {current_steering:.2f} ||| Speed: {current_speed:.1f} ||| Error: {error:.3f} ||| P:{p_term:.2f} I:{i_term:.2f} D:{d_term:.2f}")

    
    

            
            
            # -----------------------------------------------------
            # TIME FOR SAFTEY ATTRIBUTES
            # _____________________________________________________
    
        obstacle_detected = False 
            
        if lidar is not None:
            
            range_image = lidar.getRangeImage()
            lidar_width = lidar.getHorizontalResolution()
            lidar_layers = lidar.getNumberOfLayers()
                
            #Only Checking the objects that are infront of the car / the middle 20%
            center_start = int(lidar_width * 0.4)
            center_end = int(lidar_width * 0.6)
            ycenter_start = int(lidar_layers * 0.1)
            ycenter_end = int(lidar_layers)
             
            for y in range(ycenter_start, ycenter_end):
                for x in range(center_start, center_end):
                    index = x + (y * lidar_width)
                    distance = range_image[index]
                    #If the object is less that or equal to 10 meters away from the car
                    if distance < 5:
                        obstacle_detected = True
                        break
                if obstacle_detected:
                    break
                
                #override cameraas:
            if obstacle_detected:
                if current_speed > 30.0:
                    driver.setBrakeIntensity(0.0)
                    current_steering = 0.5
                    print("High speed, swerving right")
                else:    
                    driver.setBrakeIntensity(1.0)
                    current_speed = 0.0
                    current_steering = 0.0
                    print("OH MY GOD YOURE ABOUT TO CRASH")
                
            #______________________________________________________
            
            
    #Safety clamp for the Honda steering limits
    current_steering = max(-0.5, min(0.5, current_steering))
        
    driver.setCruisingSpeed(current_speed)
    driver.setSteeringAngle(current_steering)

