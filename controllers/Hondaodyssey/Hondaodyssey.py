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
from controller import Keyboard, Lidar, Camera, GPS, Compass



 
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

def init_device(name, *fns):
    dev = driver.getDevice(name)
    if dev:
        for fn in fns: fn(dev)
    else:
        print(f"Device '{name}' NOT found")
    return dev

lidar = init_device("lidar", lambda d: d.enable(TIME_STEP), lambda d: d.enablePointCloud())
camera = init_device("camera", lambda d: d.enable(TIME_STEP))
Left_Camera = init_device("Left_Camera", lambda d: d.enable(TIME_STEP))
Top_Camera = init_device("Top_Camera", lambda d: d.enable(TIME_STEP))
gps = init_device("gps", lambda d: d.enable(TIME_STEP))
compass = init_device("compass", lambda d: d.enable(TIME_STEP))
#__________________________________________________________
#Variables
#----------------------------------------------------------

TARGET_WAYPOINT = [50.0, -100.0]
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
#update: im adding 2 more buffers for top and bottom parallel jits
angle_filter_buffer = [0.0, 0.0, 0.0]
traffic_state = "GO"
#Update 2: new ghost memory blocks:
memory_bottom_x_r = 0.0
memory_top_x_r = 0.0
frames_missing_r = 0
memory_bottom_x_l = 0.0
memory_top_x_l = 0.0
frames_missing_l = 0
last_loc = 1
frames_since_red = 0
nav_steering_desire = 0.0
distance_to_target = 999.0
calibrated_x_r = None
calibrated_x_l = None

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
        #return None  
        #no light detected 

def color_match(r, g, b, target, tol=35):
    return abs(r-target[0]) < tol and abs(g-target[1]) < tol and abs(b-target[2]) < tol

def detect_traffic_light(image, width, height):
    zone_size = 2
#green:
#63, 192, 123
#75, 207, 124
#yellow:
#209, 179, 99
#211, 182, 115
#222, 192, 90
#red:
#209, 61, 75
#210, 78, 93
#223, 79, 81
    target_red = (215, 75, 85)
    target_yellow = (215, 185, 100)
    target_green = (70, 200, 125)
    tolerance = 35
    for zone_y in range(0, height, zone_size): 
        for zone_x in range(0, width, zone_size):
            red = 0; green = 0; yellow = 0
            for y in range(zone_y, min(zone_y + zone_size, height)):
                for x in range(zone_x, min(zone_x + zone_size, width)):
                    r, g, b = image[x][y]
                    if color_match(r, g, b, target_red): red += 1
                    elif color_match(r, g, b, target_yellow): yellow += 1
                    elif color_match(r, g, b, target_green): green += 1
                    
            if red >= 2:
                return "RED"
            elif green >= 2:
                return "GREEN"
            elif yellow >= 2:
                return "YELLOW"
    return None

def compute_nav(gps_dev, compass_dev, target):
    pos = gps_dev.getValues()    
    comp = compass_dev.getValues()
    heading = math.atan2(comp[0], comp[2])
    dx, dz  = target[0] - pos[0], target[1] - pos[2]
    dist = math.sqrt(dx**2 + dz**2)
    diff = math.atan2(dx, dz) - heading
    while diff >  math.pi: diff -= 2 * math.pi
    while diff < -math.pi: diff += 2 * math.pi
    return max(-0.5, min(0.5, diff * 0.5)), dist

def read_keys(kb):
    up = down = left = right = toggle = False
    k = kb.getKey()
    while k != -1:
        if k == Keyboard.UP: up = True
        if k == Keyboard.DOWN: down = True
        if k == Keyboard.LEFT: left = True
        if k == Keyboard.RIGHT: right = True
        if k == ord('A'): toggle = True
        k = kb.getKey()
    return up, down, left, right, toggle

def is_white(r, g, b):
    return (r + g + b) > 450 and abs(r - g) < 30 and abs(r - b) < 30
    
def scan_camera(image, x_start, x_end, width, height, memory_x, calibrated_x, frames_missing, label):
    start_y = int(height * 0.3)
    count = total = 0
    for y in range(start_y, height, 2):
        for x in range(x_start, x_end, 2):
            r, g, b = image[x][y]
            if is_white(r, g, b): total += x; count += 1

        avg_x = 0.0
        if count > 0:
        avg_x = total / count
        if memory_x != 0.0 and abs(avg_x - memory_x) > width * 0.25:
            count = 0; avg_x = 0.0

        if count > 0:
            if calibrated_x is None: calibrated_x = avg_x
            memory_x = avg_x; frames_missing = 0
            err = (calibrated_x - avg_x) / width
            if avg_x > width * 0.85: err *= 1.5
            return err, memory_x, calibrated_x, frames_missing, count, avg_x

        if frames_missing < 6 and memory_x != 0.0:
            frames_missing += 1
            target = calibrated_x or (width * (0.77 if label=="R" else 0.15))
            print(f"{label} memory: frame {frames_missing}/6")
            return (target - memory_x) / width, memory_x, calibrated_x, frames_missing, 1, memory_x

        return 0.0, memory_x, calibrated_x, frames_missing, 0, 0.0

def update_traffic(light, state, frames_red):
    if light == "RED": return "STOP", 0
    elif light == "GREEN": return "GO", 0
    elif light == "YELLOW": return ("STOP" if state == "STOP" else "SLOW"), 0
    if state == "STOP":
        frames_red += 1
        if frames_red > 150: return "GO", 0
    return state, frames_red

print("Use the up/down/left/right buttons to move")
print("Press A to start the AUTOPILOT")

#Main Loop
while driver.step() != -1:
    key = keyboard.getKey()
    
    current_steering = 0.0
    if gps is not None and compass is not None:
        nav_steering_desire, distance_to_target = compute_nav(gps, compass, TARGET_WAYPOINT)
    
    #Identifying keys
    up = False; down = False; right = False; left = False  

    
    
    up, down, left, right, toggle = read_keys(keyboard)
    if toggle: autodrive = not autodrive
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
        start_y = int(height * 0.3)
        
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
        err_r, memory_bottom_x_r, calibrated_x_r, frames_missing_r, pixel_count_right, avg_x_r = \
            scan_camera(camera.getImageArray(), int(width*0.25), width, width, height,
                memory_bottom_x_r, calibrated_x_r, frames_missing_r, "R")   
        
        if pixel_count_right > 0:
            error += err_r
            visible_cameras += 1
            last_average_x = avg_x_r 
        #PROCESS LEFT CAMERA:
            #Continue the code with the Right camera and average out error
            
        err_l, memory_bottom_x_l, calibrated_x_l, frames_missing_l, pixel_count_left, avg_x_l = \
            scan_camera(Left_Camera.getImageArray(), 0, int(width*0.65), width, height,
                memory_bottom_x_l, calibrated_x_l, frames_missing_l, "L")
        
        if pixel_count_left > 0:
            error += err_l
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
            error = sum(angle_filter_buffer) / len(angle_filter_buffer)
            if abs(error) < 0.02:
                error = 0.0
            
            if was_blind or visible_cameras != last_visible_cameras:
                last_error = error
                was_blind = False
            if (error > 0) != (last_error > 0):
                integral = 0.0
            integral += error
            integral = max(min(integral, 30.0), -30.0) 
            integral *= 0.9
            #Turn
            p_gain = 0.6 + min(abs(error) * 1.5, 0.6)
            p_term = error * p_gain
            #memory
            i_term = integral * 0.05
            #overshoot
            d_term = (error - last_error) * 1.5

            
            current_steering = p_term + i_term + d_term
            
            rate_limit = min(0.15 + abs(error) * 0.4, 0.30)
            if current_steering - last_steering > rate_limit:
                current_steering = last_steering + rate_limit
            elif current_steering - last_steering < -rate_limit:
                current_steering = last_steering - rate_limit
                
            turn_factor = 1.0 - abs(current_steering) * 2.0
            current_speed = max(17.0, 25.0 * turn_factor)
            last_error = error
            last_steering = current_steering
            driver.setBrakeIntensity(0.0)
            
            pos_r = (avg_x_r / width) if pixel_count_right > 0 else 0.0
            pos_l = (avg_x_l / width) if pixel_count_left  > 0 else 0.0
            print(f"Cameras active: {visible_cameras} ||| L_Line is at: {pos_l:.2f} ||| R_Line is at: {pos_r:.2f} ||| Steer: {current_steering:.2f}")
 

        #ELSE: for the blind spots
            #Hold steering wheel to the last known location/ange
            #set constant speed for recovery
        else:
            
            was_blind = True
            last_visible_cameras = 0
            driver.setBrakeIntensity(0.0)
            if last_average_x > (width * 0.5):
                last_loc = 1
            else: 
                last_loc = -1
            current_steering = abs(last_steering) * 2 + 1 #* last_loc
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
    
        
        if Top_Camera is not None:
            image_top = Top_Camera.getImageArray()
            top_width = Top_Camera.getWidth()
            top_height = Top_Camera.getHeight()
        
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
        light = detect_traffic_light(image_top, top_width, top_height)
        traffic_state, frames_since_red = update_traffic(light, traffic_state, frames_since_red)
        
        if traffic_state == "STOP":
            driver.setBrakeIntensity(1.0)
            current_speed = 0.0
            current_steering = nav_steering_desire
            if light is not None:
                print(f"red light ||| GPS: {nav_steering_desire:.2f}")
        elif traffic_state == "SLOW":
            current_speed = max(5.0, current_speed * 0.5)
            current_steering = nav_steering_desire
            if light is not None:
                print(f"slow ||| GPS: {nav_steering_desire:.2f}")
        elif traffic_state == "GO" and light is not None:
            driver.setBrakeIntensity(0.0)
            if light is not None:
                current_steering = nav_steering_desire
                last_steering = max(-0.5, min(0.5, current_steering))
                print(f"green ||| GPS: {nav_steering_desire:.2f}")
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
                    print("OH MY GOD YOURE ABOUT TO CRASH")
       
            #______________________________________________________
            
            
    #Safety clamp for the Honda steering limits
    if autodrive:
        if visible_cameras == 0 and traffic_state == "GO":
           
            print(f"blind recovery. Dist: {distance_to_target:.1f}")
        if distance_to_target < 3.0:
            current_speed = 0.0
            driver.setBrakeIntensity(1.0)
            print("You reached your destination")
    current_steering = max(-0.5, min(0.5, current_steering))
        
    driver.setCruisingSpeed(current_speed)
    driver.setSteeringAngle(current_steering)