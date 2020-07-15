# UAV_Teensy
For all my embedded systems dudes - this Flight controller will be witten in ***pure C***: Latest start August! 

Please make sure to read the entire README.md file **BEFORE** doing anything. Thank you!

If you want to use this software please "fork" and "star" this project to make sure your version doesn't change and to get notified of any updates. Feel free to report issues to be fixed. Enjoy

****description comming a little bit after the end of this project: 5/7/2020, just need to vibe a little then its comming. Latest start August***


***IMPORTANT!!!***
 - ***Make sure all inputs and sensors are set in the same direction as the code is written***
 - ***TAKE YOUR F''''' PROPELLERS OFF!***
 - ***DON'T BE STUPID***



# Update


![alt text](https://app.diagrams.net/?lightbox=1&highlight=0000ff&edit=_blank&layers=1&nav=1#R7VtRk5owEP41PF4HAog%2BqnfXdubaaXsP7fUtlVTSQcJArNpf30QCSJJTPMWo44wPZN3EsPt9m90FLXc8W77PYBp9IiGKLWCHS8u9twBwPAAs%2FrHDVSEJgFcIphkOhVIteMb%2FkBDaQjrHIcobipSQmOK0KZyQJEET2pDBLCOLptpvEjd%2FNYVTpAieJzBWpd9xSKNC2gdBLf%2BA8DQqf9npDYpvZrBUFkvkEQzJohCtb859sNxxRggtrmbLMYq58Uq7FBZ4fOXbamMZSmibCcPPD6Pn8deXyfDnE3bIt0%2Br4Oud0xObo6vyjlHIDCCGJKMRmZIExg%2B1dJSReRIivqzNRrXOEyEpEzpM%2BAdRuhLehHNKmCiis1h8i5aY%2Fti4fuFLvfPF6H4pVl4PVuUgodnqx%2BZgYxYf1tPWo3JecX%2F8pl61W%2BkgMs8maJuxBP5gNkV0mx6o3Mt4gcgMsQ2xiRmKIcV%2FmxuBAqDTSq%2F2IbsQbtzDpaB%2Fc%2BnxXeod6NL11GGWwdWGQkpwQvONlb9wAVMow6croqAIns5AovgOfde2JTgVO6jBVd3K2%2FEmbvovjOfCDDOYzOF6XpLOqYLGJtYWEaboOYVrNy3YIdLElVgcZRQtt3tbdY6Y4ElGLMeLOp73hCjaDOXCeB0w1L4xtDVDQUuGHkrQwzyqcOCLcdgDOXj02uG%2B1xnsHcUmN9hvhXObg%2BkVEJwG956Ce9XFccxydbQb8jBPiwT%2BN15yhx%2BDA67Mgb7KAaDhAOiMA%2B6NA605ELTlgGuSA4Ea%2Bz%2FePSEYGj8BPDnz0ZwA%2FVOeAE5weeivAf9SrnEa9Pfbov%2FQavNNpUnQ3680kfU9Z0cpE2zV76aU6atUjmFivoLp2buJ7OjOse5yuQvsGzWZbJ%2BOyWUv88wPsnKbG%2FB%2FfDSOfc9vYh%2BYrmI0Ke65I99ce61tfy0wCny1haX62GgZIx%2B2pssYZ3AjQXsStG1hHZrIHeZStYd1XiSokp5zIUH5JPNGgjYkcFuSoG%2BUBO6Zk8CXS3rjJLiV9HuQoG1X1%2BjDDEft6p5BHSA%2FzgCB4ULAUbsFKhWScMhfQmGjSQzzHE8kHHeOyTc%2FQNgwq68xaynbr8u0sy2kuKvglJi1pV80CHYsVBhCWWjfvpc70P9Op30p4B8Tal3lALuhZlsGoebK%2BePgjVBTqjGnI6hJG%2FaPizRdvu%2Fxjz9OI2wBtoDNrmmEKKxGaY6Flow%2FFtmplP%2FEeJpwODKgoIwJePzHExgPxRczHIZFPoBy%2FA%2F%2BWi%2FFMScsw9b1R5Z%2Fz9diKUBeZANHOk98%2BV2ZgXqeaHuqnaVS5YZ0HgkJtYKRcE1wX%2FlDyEEv5sb%2FlbGrKb%2BqHNdOl7uVa16HZx0pRGtahhVndYHm%2BJ7VNZhGl29o%2Bem6rxr6tAzSxbQrsHNJytLOmtT3tHbWlcmXb2dXChzm7ax7v%2BYK7AykAG2btrMuv74CO8sZr3E89xQ7312%2BleXK0zia1TeSrgHNjtz61CTsp7Wz2gC6fDQ7yuP2zqzMhvW%2Fn4oCtv4PmfvwHw%3D%3D)




UAV PID control with teensy 3.6

# Main Goal 
This project will be made highly scaleble, so it's not restricted to a Hexacopter as documented in this project it can also be applyed to Quad- Hexa- octo- and so on..- copter.

# Files 
- UAV_Flight_Controller . This is the main controller with all its files, discreption is inside the files
- Processing. Just a script with a 3D model of live data, this can be found at https://www.pjrc.com/store/prop_shield.html
- matlab. Some matlab and simulink models of the Drone
- dev trash. just files from erly stages in the development... year just skip that one.

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


Data processing tools used:
- [Python](https://www.python.org/) used for extracting data.
- [Processing](https://processing.org/) used for real-time data visualization.
- [Matlab](https://www.mathworks.com/) used for simulation. 

# Licence
Using this code is **completely at your own risk!!!**
This is an open source project so you can use it as you please.


