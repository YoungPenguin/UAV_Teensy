// Basic Receiver FAIL SAFE
// check for 500-2500us and 10-330Hz (same limits as pololu)

boolean failsafe(int CH){

   int i = CH-1;
   boolean failsafe_flag = LOW;
        
       if(pwmFlag[i] == 1)                             // if a new pulse has been measured.
         {
            pwmFlag[i] = 0;                            // set flag to zero
      
            if(pwmPeriod[i] > 100000)                  // if time between pulses indicates a pulse rate of less than 10Hz   
            {
              failsafe_flag = HIGH;                       
            }
            else if(pwmPeriod[i] < 3000)               // or if time between pulses indicates a pulse rate greater than 330Hz   
            {
              failsafe_flag = HIGH;                             
            }

            if(PW[i] < 500 || PW[i] > 2500)           // if pulswidth is outside of the range 500-2500ms
            {
              failsafe_flag = HIGH;                        
            }   
         }
        else if (micros() - pwmTimer[i] > 100000)     // if there is no new pulswidth measurement within 100ms (10hz)
        {
          failsafe_flag = HIGH;                      
        }

    return failsafe_flag;   
}
