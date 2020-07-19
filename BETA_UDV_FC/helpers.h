class helpers {
  public:
    helpers();
    void discretize(float* Kp, float* Ki, float* Kd);
    void dataVector(float* data);
    unsigned int StickPos(uint8_t* a, int* input1, int* input2, int* input3, int* input4);
  private:
    float *Kp, *Ki, *Kd, *a;
    float *sampleTime;
    float *AntiWindup;
    float *f1, *f2, *K1, *K2, *K3;
};
