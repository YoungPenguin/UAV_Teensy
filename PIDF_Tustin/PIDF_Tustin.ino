
void PIDF_Tustin(float kp, float tauI, float tauD, float a, float error, float & old_error, float & old_I, float & old_Lead, float & PIDF_OUT) {
  float K = 2 / T;
  float K1 = -a * tauD + K / (a * tauD + K);
  float K2 = tauD + K / (a * tauD + K);
  float f = 1 / K * kp / tauI;

  float P_term = kp * error;
  float I_term = (error + old_error) * f + old_I;
  float Lead_term = (error - old_error) * K2 - K1 * old_Lead;

  old_I = I_term;
  old_Lead = Lead_term;
  old_error = error;

  PIDF_OUT = P_term + I_term + Lead_term;
}
