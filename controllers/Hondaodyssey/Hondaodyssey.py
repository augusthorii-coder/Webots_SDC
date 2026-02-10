"""
Honda Odyssey is the cars name
August Horii
ATCS PROJECT for a self driving car
"""
"""
import math

from vehicle import Driver

driver = Driver()
driver.setSteeringAngle(0.2)
driver.setCruisingSpeed(20)

while driver.step() != -1:
  angle = 0.3 * math.cos(driver.getTime())
  driver.setSteeringAngle(angle)
  """
  import math
from vehicle import Driver
from controller import Keyboard

#my constants:
TIME_STEP = 64
MAX_SPEED = 20.0 #assuming this is km/h
TURN_ANGLE = 0.5 #Radians

driver=Driver()
driver.setSteeringAngle(0.0)
driver.setCruisingSpeed(0.0)

keyboard = Keyboard()
keyboard.enable(TIME_STEP)

#Time for the movement
while driver.step() != -1:


#Variables for the sensor input
    key = keyboard.getKey()
    
    current_speed = 0.0
    current_steering = 0.0
    
#Pressing if statements:
        #For moving front and backwards
    if key == Keyboard.UP:
        current_speed = MAX_SPEED
    elif key == Keyboard.DOWN: 
        current_speed = -MAX_SPEED
    else:
        current_speed = 0.0
 
         #For turning left and right
    if key == Keyboard.LEFT:
        current_steering = -TURN_ANGLE
    elif key == Keyboard.RIGHT:
        current_steering = TURN_ANGLE
    else:
        current_steering = 0.0
        
        #activation
driver.setCruisingSpeed(current_speed)
    driver.setSteeringAngle(current_steering)