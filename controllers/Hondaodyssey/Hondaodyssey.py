"""
Honda Odyssey is the cars name
August Horii
ATCS PROJECT for a self driving car
"""
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


#Lidar
lidar = driver.getDevice("lidar") 
if lidar is not None:
    lidar.enable(TIME_STEP)
    lidar.enablePointCloud()
else:
    print("Twin your lidar is NOT there")

camera = driver.getDevice("camera")
if camera is not None:
    camera.enable(TIME_STEP)
else:
    print("Your camera is NOT plugged in Twin")




#to start in manual mode:
current_speed = 0.0
autodrive = False
last_error = 0.0
last_steering = 0.0
integral = 0.0
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
        #the AUTOPILOT area
        current_speed = 10.0 # Constant cruising speed
            
        if camera is not None:
            image = camera.getImageArray()
            width = camera.getWidth()
            height = camera.getHeight()
            sum_x = 0
            pixel_count = 0
            
            #Scan the bottom section of the screen
            start_y = 0
            end_y = height
            #STOP GOING TO THE LEFT
            for y in range(start_y, end_y, 2):
                for x in range(0, int(width * 0.5), 2): # Look at the WHOLE road
                    #Finding the pixel colors:
                    r, g, b = image[x][y]
                    brightness = r + g + b
                    #if the pixels are very bright:
                    is_white = brightness > 450 and abs(r - g) < 30 and abs(r - b) < 30
                    
                    #High Red and Green, very low Blue
                   # is_yellow = r > 150 and g > 130 and b < 100 and (r - b) > 50
                    if is_white: # or is_yellow:
                        sum_x += x
                        pixel_count += 1
            #left lane keeping logic
            if pixel_count > 0:
                average_x = sum_x / pixel_count
                target_x = width * 0.2  
                
                error = (target_x - average_x) / width 
                
                if (error > 0) != (last_error > 0):
                    integral = 0.0
                    
                integral += error
                integral = max(min(integral, 30.0), -30.0) 
                
                
                #P=Kp * E(t)
                p_term = error * 3.0
                i_term = integral * 0.05
                d_term = (error - last_error) * 20.0
                
                current_steering = p_term + i_term + d_term
                
                turn_factor = 1.0 - abs(current_steering) * 1.2
                current_speed = max(15.0, 20.0 * turn_factor)
                
                last_error = error
                last_steering = current_steering
                driver.setBrakeIntensity(0.0)
            else:
            #FIND THE LINEEEEE
                driver.setBrakeIntensity(0.0)
                if abs(last_error) > 0.05:
                    current_steering = -(last_error * 2.0)
                else:
                    current_steering = 0.0
                current_steering = max(-0.4, min(0.4, current_steering))
                     # Straighten the wheel so it doesn't swerve
                current_speed = 23.0 # Keep rolling forward
                
            print(f"I see {pixel_count} pixels. Steering: {current_steering}")
            
            # -----------------------------------------------------
            # TIME FOR SAFTEY ATTRIBUTES
            # _____________________________________________________
            obstacle_detected = False 
            
            if lidar is not None:
                range_image = lidar.getRangeImage()
                lidar_width = lidar.getHorizontalResolution()
                
                #Only Checking the objects that are infront of the car / the middle 20%
                center_start = int(lidar_width * 0.4)
                center_end = int(lidar_width * 0.6)
                
                for i in range(center_start, center_end):
                    distance = range_image[i]
                    #If the object is less that or equal to 10 meters away from the car
                    if distance < 5:
                        obstacle_detected = True
                        break
                
                #override cameraas:
                if obstacle_detected:
                    driver.setBrakeIntensity(1.0)
                    current_speed = 0.0
                    print("OH MY GOD YOURE ABOUT TO CRASH")
            #______________________________________________________
            
            
    #Safety clamp for the Honda steering limits
    current_steering = max(-0.5, min(0.5, current_steering))
        
    driver.setCruisingSpeed(current_speed)
    driver.setSteeringAngle(current_steering)




"""
            if pixel_count > 5:
                average_x = sum_x / pixel_count
                target_x = width * 0.2  
                
                error = (average_x - target_x) / width 
                
                if (error > 0) != (last_error > 0):
                    integral = 0.0
                    
                integral += error
                integral = max(min(integral, 30.0), -30.0) 
                
                p_term = error * 3.0
                i_term = integral * 0.05
                d_term = (error - last_error) * 10.0
                
                current_steering = p_term + i_term + d_term
            else:
                #FIND THE LINE TWIN
                driver.setBrakeIntensity(0.2)
                current_steering = last_steering
                current_speed = 4.0


current_speed = 10.0 # Constant cruising speed
            
        if camera is not None:
            image = camera.getImageArray()
            width = camera.getWidth()
            height = camera.getHeight()
            sum_x = 0
            pixel_count = 0
            far_sum_x = 0; far_pixels = 0
            near_sum_x = 0; near_pixels = 0
            
            
            #Scan the bottom section of the screen
            start_y = int(height * 0.5)
            end_y = int(height * 0.9)
            #STOP GOING TO THE LEFT
            start_x = int(width * 0.4)
            for y in range(start_y, end_y, 2):
                for x in range(start_x, width, 2):
                    #Finding the pixel colors:
                    r, g, b = image[x][y]
                    brightness = r + g + b
                    
                    #if the pixels are very bright:
                    if brightness > 400: 
                        if y < height * 0.7:  
                            far_sum_x += x
                            far_pixels += 1
                        else:                 
                            near_sum_x += x
                            near_pixels += 1
            
            #left lane keeping logic
            if pixel_count > 0:
                near_x = near_sum_x / near_pixels
                target_x = width * 0.85 
                
                error = (near_x - target_x) / width #Error = how far it is from the center
                
                #to steer towards the area smoothly:
                p_term = error * 2.0
                d_term = (error - last_error) * 5.0
                curve_error = 0.0
                if far_pixels > 0:
                    far_x = far_sum_x / far_pixels
                    curve_error = (far_x - near_x) / width
                
                # Combine normal steering with curve anticipation
                current_steering = p_term + d_term + (curve_error * 3.0)
                
                last_error = base_error
            else:
                current_steering = 0.0
            print(f"I see {pixel_count} pixels. Steering: {current_steering}")
            
    #Safety clamp for the Honda steering limits
    if current_steering > 0.5:
        current_steering = 0.5
    elif current_steering < -0.5:
        current_steering = -0.5
        
    driver.setCruisingSpeed(current_speed)
    driver.setSteeringAngle(current_steering)




    else:
        #the AUTOPILOT area
        #JUST LANE KEEPING FOR NOW
        current_speed = 20.0 # Constant cruising speed
            
        if camera is not None:
            image = camera.getImageArray()
            width = camera.getWidth()
            height = camera.getHeight()
            sum_x = 0
            pixel_count = 0
            y = int(height * 0.8)
            
            for x in range(width):
                #Finding the pixel colors:
                r, g, b = image[x][y]
                brightness = r + g + b
                
                #if the pixels are very bright:
                if brightness > 400: 
                    sum_x += x
                    pixel_count += 1
            
            #left lane keeping logic
            if pixel_count > 0:
                average_x = sum_x / pixel_count
                center_of_screen = width / 2 
                error = (average_x - center_of_screen) / width #Error = how far it is from the center
                #to steer towards the area:
                current_steering = error * 2
            else:
                current_steering = 0.0
                print(f"I see {pixel_count} bright pixels. Steering: {current_steering}")
        
    driver.setCruisingSpeed(current_speed)
    driver.setSteeringAngle(current_steering)





    elif up and right:
        current_steering = TURN_ANGLE
        current_speed = MAX_SPEED
    elif up and left:
        current_steering = -TURN_ANGLE
        current_speed = MAX_SPEED
    elif down and right:
        current_steering = TURN_ANGLE
        current_speed = -MAX_SPEED
    elif down and left:
        current_steering = -TURN_ANGLE
        current_speed = -MAX_SPEED

    
    # Check speed (UP/DOWN)
    if key == Keyboard.UP:
        current_speed = MAX_SPEED
    elif key == Keyboard.DOWN: 
        current_speed = -MAX_SPEED
        
    # Check steering (LEFT/RIGHT)
    if key == Keyboard.LEFT:
        current_steering = -TURN_ANGLE
    elif key == Keyboard.RIGHT:
        current_steering = TURN_ANGLE
"""
       
       

        