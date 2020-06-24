# Securitron9000
Proof-of-concept security robot with emotional displays

This project showcases my prototyping, design, embedded systems programming, and control systems skills.  
  
Small "Security" robot implemented using two TI Tiva TM4C123 Microcontrollers. This robot can respond to real time sensory input and react with different expressions and movements accordingly. This was accomplished using GPIO pins, Pulse Width Modulation, periodic interrupts, edge triggered interrupts, and Analog-Digital Conversion. This project implements different sensors, motors, and LEDs in a robot to perform some of the tasks that a security guard would enact. A security guard checks if something is near and goes on alert. If necessary, a security guard will fend off anything or anyone that gets too close to the area he/she should be securing. An Ultrasonic Range Finder and a IR emitter/receiver pair were used as sensors when someone got too close or touched the robot. The RGB LEDs on the head changed colors to display different emotions.  

 - Green: Happy face ­ when robot does not detect anything
 - Red: Angry face ­ when robot is in defensive stance or performing an action
 - Blue: Sad face ­ when robot does not detect anything for a period of time and remains idle.  
 
Alert Stance:
[![Alert Stance](https://drive.google.com/uc?export=view&id=1Srieu43bsRPaLhKfefjAZgmzX9IyLwKN)](https://www.youtube.com/watch?v=OCv8j-X7UC4&feature=emb_title)

Attack Mode
![Alert Stance](https://drive.google.com/uc?export=view&id=15N6N613nXAvLW47pHhU9mX_9v5Sga5wl)https://www.youtube.com/watch?v=gSRbzAf-oV8


When something or someone triggers the robot's sensors, servo motors in the arms react and take a defensive stance. If the object remains there for a length of time, the arms move in an attacking motion.  
There are two microcontrollers used, one for the head with all the LEDs and another for the body with all the sensors and servo motors that send out signals to the head to change color.  
  
A PCB was also designed to manage the power into the servos and sensors.
