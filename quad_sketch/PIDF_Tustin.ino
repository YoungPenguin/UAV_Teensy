
// function
float PIDF_Tustin(float kp, float K1, float K2, float f, float error, float & old_error, float & old_I, float & old_Lead) {
  float P_term = kp * error;
  float I_term = (error + old_error) * f + old_I;
  float Lead_term = (error - old_error) * K2 - K1 * old_Lead;

  old_I = I_term;
  old_Lead = Lead_term;
  old_error = error;

  float PIDF_OUT = P_term + I_term + Lead_term;
  return PIDF_OUT;
}
