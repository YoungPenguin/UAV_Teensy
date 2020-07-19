# Dev UAV
Hello there stranger and welcome to the dark-side...

For all my embedded systems dudes - this Flight controller will be witten in ***pure C***: Latest start August! 

Please make sure to read the entire README.md file **BEFORE** doing anything. Thank you!
Remember this is the beta branch with all the on-going development. 


# Update
Im currently re-writing the code with header files and proper use of classes and such... also im working on a big improvement to the control loop architecture, the currently implemented is just a simple PI-Lead with the Lead in the feedback path. This will be updated with a cascade approach with some feedforward. The future control architecture can be seen in the figure below. This will still only be for stabilization the GPS implementation will come later....

![Untitled Diagram](https://user-images.githubusercontent.com/56176145/87570808-73198580-c6c9-11ea-930f-5fbc4cdc08ac.png)




# Main Goal 
This project will be made highly scaleble, so it's not restricted to a Hexacopter as documented in this project it can also be applyed to Quad- Hexa- octo- and so on..- copter.

The future development will be some different stuff... look in the updates section



Data processing tools used:
- [Python](https://www.python.org/) used for extracting data.
- [Processing](https://processing.org/) used for real-time data visualization.
- [Matlab](https://www.mathworks.com/) used for simulation. 


***IMPORTANT!!!***
 - ***Make sure all inputs and sensors are set in the same direction as the code is written***
 - ***TAKE YOUR F''''' PROPELLERS OFF!***
 - ***DON'T BE STUPID***


# Licence
Using this code is **completely at your own risk!!!**
This is an open source project so you can use it as you please.

videos included 

## simulations files 
If you have a valid matlab licence you can use:
- Hexacopter sim
- Quadcopter sim

If you don't. No need to worry, i will make simulation in python soon....

# The UAV used for testing in this project 
Hardware:
- Teensy 3.6
- teensy prop shield
- Taranis Transmitter/ reciver
- Propeller: carbon 13.5x5.5
- Motor: turnigy 3508-700
- 4s 5000mAh
- hobbywing x-rotor 40A 2-6S Lipo (No BEC)

UAV specs:

- Configuration: hex x copter
- Weight without batteries: 2000g
- Arm length: 350mm

## secondary UAV test
Hardware:
- Teensy 3.5
- teensy prop shield
- Taranis Transmitter/ reciver
- Propeller: carbon 10x4.5
- Motor: turnigy 3508-700
- 4s 5000mAh
- hobbywing x-rotor 40A 2-6S Lipo (No BEC)

UAV specs:

- Configuration: quad x copter
- Weight without batteries: 1400g
- Arm length: 250mm

