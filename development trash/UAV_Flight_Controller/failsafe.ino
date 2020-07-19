/* 
 *  Basic Receiver FAIL SAFE
 *  This has to be changed accordenly to your setup 
 *  In this case the fail-safe is setup on the transmitter side
 */

void failsafe() {
  while (true) {
    stopAll();
    Serial.println("You are in Fail-Safe please restart your device and tjeck for damage!");

    // the use of delay is only here because we are in an infinite loop so it does not matter... else it's a big nono
    PORTB |= B00100000;
    delay(1000);
    PORTB &= B11011111;
    delay(1000);
  }
}
