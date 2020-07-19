/*
 * This block can be used to convert the error from the GPS int he World frame to the UAVs Body fixed frame
 * You can take this and add a PID regulation.
 * This will allow for features such as home-lock, return-to-home, trajectory control.
 * Have fun implementing new features, and ofc. don't forget to share on github 
*/

float WorldToBody(float & x_error, float & y_error, float Heading) {
  x_error = x_error * cos(Heading) - y_error * sin(Heading);
  y_error = y_error * cos(Heading) + x_error * sin(Heading);
}
