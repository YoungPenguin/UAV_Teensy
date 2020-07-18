/*
 * Just some helper function like anti-windup and so....
*/

class helpers {
  public:
    int anti_windup(float a, float b, float c) {
      a = (a < b) ? b : a;
      a = (a > c) ? c : a;
      return a;
    }
};
