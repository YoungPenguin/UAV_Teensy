
// set values
float T = 0.0025;
float a = 0.001;
float tauD[3] = {0.08, 0.08, 0.08};
float tauI = {0.08, 0.08, 0.08};
float kp = {1.3, 1.3, 1.3};
float K = 2 / T;
float K1[3];
float K2[3];
float f[3];

// compute in setup
K1[0] = -2 * a * tauD[0] + K / (2 * a * tauD[0] + K);
K1[1] = -2 * a * tauD[1] + K / (2 * a * tauD[1] + K);
K1[2] = -2 * a * tauD[2] + K / (2 * a * tauD[2] + K);

K2[0] = a * tauD[0] * kp[0] / (2 * a * tauD[0] + T);
K2[1] = a * tauD[1] * kp[1] / (2 * a * tauD[1] + T);
K2[2] = a * tauD[2] * kp[2] / (2 * a * tauD[2] + T);

f[0] = 1 / K * kp[0] / tauI[0];
f[1] = 1 / K * kp[1] / tauI[1];
f[2] = 1 / K * kp[2] / tauI[2];



// function
void PIDF_Tustin(float kp, float K1, float K2, float f, float error, float & old_error, float & old_I, float & old_Lead, float & PIDF_OUT) {

  float P_term = kp * error;
  float I_term = (error + old_error) * f + old_I;
  float Lead_term = (error - old_error) * K2 - K1 * old_Lead;

  old_I = I_term;
  old_Lead = Lead_term;
  old_error = error;

  PIDF_OUT = P_term + I_term + Lead_term;
}
