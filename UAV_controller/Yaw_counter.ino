float Yaw_counter(float yaw_difference) {
  if (yaw_difference < -20)
  {
    yaw = 1;
  }
  else if (yaw_difference > 20)
  {
    yaw = -1;
  }
  else {
    yaw = yaw_difference;
  }
  total_yaw += yaw;
return total_yaw;
}
