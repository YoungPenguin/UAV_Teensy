# UAV_Teensy
Hello there stranger...

Please be aware that there are currently **2 different branches** on this git repo. Switch to the **dev branch** if you want to get the latest updates and want to know what is currently under development.  

![Farmers Market Finder Demo](unavngivet.mp4)


Please make sure to read the entire README.md file **BEFORE** doing anything. Thank you!

If you want to use this software please "fork" and "star" this project to make sure your version doesn't change and to get notified of any updates. Feel free to report issues to be fixed. Enjoy

# Files 
- UAV_Flight_Controller . This is the main controller with all its files, discreption is inside the files

You need to edit the PID values specific to your UAV.
It's currently setup with a PI-Lead with the Lead in the feedback path.
The sampling rate is 400 Hz limited by the ESCs and the teensy prop shield.
Please make sure to edit the "loop_time" this is defined by the number of clock cycles which 
is dependent on the board 



***IMPORTANT!!!***
 - ***Make sure all inputs and sensors are set in the same direction as the code is written***
 - ***TAKE YOUR F''''' PROPELLERS OFF!***
 - ***DON'T BE STUPID***
 
 
# Licence
Using this code is **completely at your own risk!!!**
This is an open source project so you can use it as you please.


