// Basic Receiver FAIL SAFE
// check for 500-2500us and 10-330Hz (same limits as pololu)

void failsafe() {

  /*

            if(pwmPeriod[i] > 100000)                  // if time between pulses indicates a pulse rate of less than 10Hz
            {
              failsafe_flag = true;
            }
            else if(pwmPeriod[i] < 3000)               // or if time between pulses indicates a pulse rate greater than 330Hz
            {
              failsafe_flag = true;
            }

            if(PW[i] < 500 || PW[i] > 2500)           // if pulswidth is outside of the range 500-2500ms
            {
              failsafe_flag = true;
            }

        else if (micros() - pwmTimer[i] > 100000)     // if there is no new pulswidth measurement within 100ms (10hz)
        {
          failsafe_flag = true;
        }
  */
}
