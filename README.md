# UAV_Teensy
Please make sure to read the entire README.md file **BEFORE** doing anything. Thank you!

If you want to use this software please "fork" and "star" this project to make sure your version doesn't change and to get notified of any updates. Feel free to report issues to be fixed. Enjoy

****description comming at the end of this project: 26/6/2020***

***IMPORTANT!!!***
 - ***Make sure all inputs and sensors are set in the same direction as the code is written***
 - ***TAKE YOUR F''''' PROPELLERS OFF!***
 - ***DON'T BE STUPID***

UAV PID control with teensy 3.6

# Main Goal 
This project will be made highly scaleble, so it's not restricted to a Hexacopter as documented in this project it can also be applyed to Quad- Hexa- octo- and so on..- copter.

# Files 
- 3 DoF UAV - Template 
- 6 DoF UAV control - Folder "GPS" including the World to body conversion block as separate component this will make it very easy for you to impelement different features like "home lock" "return to home" "switch coordinate system" and so on... 

## simulations files 
- Hexacopter sim
- Quadcopter sim

# The UAV used for testing in this project 
Hardware:
- Teensy 3.6
- teensy prop shield
- Taranis Transmitter/ reciver
- Propeller: carbon 13.5x5.5
- Motor: turnigy 3508-700

UAV specs:

- Configuration: hex x copter
- Weight without batteries: 2000g
- Arm length: 400mm

## secondary UAV test
Hardware:
- Teensy 3.6
- teensy prop shield

UAV specs:

- Configuration: quad x copter


Data processing tools used:
- [Python](https://www.python.org/) used for extracting data.
- [Processing](https://processing.org/) used for real-time data visualization.
- [Matlab](https://www.mathworks.com/) used for simulation. 

# Licence
Using this code is **completely at your own risk!!!**
This is an open source project so you can use it as you please.


