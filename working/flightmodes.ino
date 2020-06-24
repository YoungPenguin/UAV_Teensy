void flightmodes() {
  
// flight mode 0  
flightflag = ((input_pin[0] < 1000) && (input_pin[1] > 1700) && (input_pin[2] > 1700) && (input_pin[3] > 1700)) ? 0 : flightflag;

// flight mode 1
flightflag = (((input_pin[0] < 1100) && (input_pin[1] < 1100) && (input_pin[2] > 1700) && (input_pin[3] < 1100) && (imu.available()))) ? 1 : flightflag;

// failsafe
flightflag = ((input_pin[0] < 1000) && (input_pin[1] > 1700) && (input_pin[2] < 1000) && (input_pin[3] > 1700)) ? 2 : flightflag;
  
}
