/*
   This block can be used to convert the error from the GPS int he World frame to the UAVs Body fixed frame
   You can take this and add a PID regulation.
   This will allow for features such as home-lock, return-to-home, trajectory control.
   Have fun implementing new features, and ofc. don't forget to share on github
*/

class WorldToBody {
  public:
    // Constructor
    PID() {
    };

    PID(float* Input, float* Output, float* Setpoint, float* terms) {
      previous_error = 0.0;
      integral = 0.0;

      PID_input = Input;
      PID_output = Output;
      PID_setpoint = Setpoint;

      Kp = &terms[P];
      Ki = &terms[I];
      Kd = &terms[D];

      windupGuard = &terms[WG];
    };



    float WorldToBody(float & x_error, float & y_error, float Heading) {
      x_error = x_error * cos(Heading) - y_error * sin(Heading);
      y_error = y_error * cos(Heading) + x_error * sin(Heading);
    }
