/*
   This block is just the different **magic stick positions** you are able to do
   on the controller to enter a specific flight mode
   You can change this or just add more in the same manner.
*/

int flightmodes(int a, int input1, int input2, int input3, int input4) {
  if ((input1 < 1000) && (input2 > 1700) && (input3 > 1700) && (input4 > 1700))a = 0;
  if ((input1 < 1100) && (input2 < 1100) && (input3 > 1700) && (input4 < 1100) && (imu.available()))a = 1;
  if ((input1 < 1000) && (input2 > 1700) && (input3 < 1000) && (input4 > 1700))a = 2;
  return a;
}
