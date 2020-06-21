/* Basic Receiver FAIL SAFE
   check for no new pulses
   This has to be changed accordenly to your setup */

void failsafe() {
  /* This is setup on the taranis transmitter, it pulses 1000 to CH1 and 2000 to CH7 (5) which will shot-off all motors // this is the safest way for us, not the UAV

    However depending on your project, you might have to set some logic in here. For example, puls the FC every 10ms and if no pulse is found go to fail safe

  */
}
