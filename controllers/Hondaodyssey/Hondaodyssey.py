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
            start_y = int(height * 0.6)
            end_y = int(height * 0.9)
            #STOP GOING TO THE LEFT
            start_x = int(width * 0.4)
            for y in range(start_y, end_y, 2):
                for x in range(0, width, 2):
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
                target_x = width * 0.85 
                
                error = (average_x - target_x) / width #Error = how far it is from the center
                
                #to steer towards the area smoothly:
                p_term = error * 2.0
                d_term = (error - last_error) * 5.0
                current_steering = p_term + d_term
                
                last_error = error
            else:
                pass
            print(f"I see {pixel_count} pixels. Steering: {current_steering}")
    #Safety clamp for the Honda steering limits
    if current_steering > 0.5:
        current_steering = 0.5
    elif current_steering < -0.5:
        current_steering = -0.5
        
    driver.setCruisingSpeed(current_speed)
    driver.setSteeringAngle(current_steering)





"""
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
"""
       
"""
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
       
       

        