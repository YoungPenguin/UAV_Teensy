float Yaw_counter(float yaw_difference) {
  yaw = (yaw_difference < -20) ? 1 : yaw_difference;
  yaw = (yaw_difference > 20) ? -1 : yaw_difference;
  total_yaw += yaw;
  return total_yaw;
}
