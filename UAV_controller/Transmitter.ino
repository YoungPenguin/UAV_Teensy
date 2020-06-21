void blink() {
  counter[5] = micros();
  ///////////////////////////////////////Channel 1
  if (GPIOB_PDIR & 2) { //pin 17 (1B 16)
    if (last_CH_state[0] == 0) {
      last_CH_state[0] = 1;        //Store the current state into the last state for the next loop
      counter[0] = counter[5]; //Set counter_1 to current value.
    }
  }
  else if (last_CH_state[0] == 1) {
    last_CH_state[0] = 0;                     //Store the current state into the last state for the next loop
    input_pin[0] = counter[5] - counter[0]; //We make the time difference. Channel 1 is current_time - timer_1.
  }

  ///////////////////////////////////////Channel 2
  if (GPIOB_PDIR & 1) { //pin 16
    if (last_CH_state[1] == 0) {
      last_CH_state[1] = 1;
      counter[1] = counter[5];
    }
  }
  else if (last_CH_state[1] == 1) {
    last_CH_state[1] = 0;
    input_pin[1] = counter[5] - counter[1];
  }
  ///////////////////////////////////////Channel 3
  if (GPIOC_PDIR & 1) {//pin 15, D2                  pin D10 - B00000100
    if (last_CH_state[2] == 0) {
      last_CH_state[2] = 1;
      counter[2] = counter[5];
    }
  }
  else if (last_CH_state[2] == 1) {
    last_CH_state[2] = 0;
    input_pin[2] = counter[5] - counter[2];
  }
  ///////////////////////////////////////Channel 4
  if (GPIOD_PDIR & 2) { //pin 14
    if (last_CH_state[3] == 0)  {
      last_CH_state[3] = 1;
      counter[3] = counter[5];
    }
  }
  else if (last_CH_state[3] == 1) {
    last_CH_state[3] = 0;
    input_pin[3] = counter[5] - counter[3];
  }
  ///////////////////////////////////////Channel 7
  if (GPIOC_PDIR & 2) { //pin 22 (1B 16)

    if (last_CH_state[4] == 0) {
      last_CH_state[4] = 1;       
      counter[4] = counter[5];
    }
  }
  else if (last_CH_state[4] == 1) {
    last_CH_state[4] = 0;                   
    input_pin[4] = counter[5] - counter[4]; 
  }
}
