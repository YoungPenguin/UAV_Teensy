/*
 * This flight mode i just the default Dis-armed mode
 * Nothing to see here, just put your own motor varibles  
 * to zeroe here and you will be fine
*/

void flightMode0() {
  if (input_pin[0] < 1000) {
    stopAll();
  }
}
