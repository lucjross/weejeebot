/*
bool findPosition_OLD(void *) {
  // North is x-axis
  float headingInv = 360 - averageHeading();
  // Adjust for board orientation
  float theta = headingInv + (360 - BOARD_ORI_deg);
  if (theta > 360)
    theta -= 360;

  // need to get both x and y possibilities for each sensor
  // (not sure which wall)
  int16_t xy[4][2];
  uint16_t adj_dists[4] = {-1, -1, -1, -1};
  uint8_t uss_offset = (uint8_t) (theta / 90);
  uint8_t uss_rotated[4];
  
  for (uint8_t uss = 0; uss < 4; uss++) {
    uss_rotated[uss] = (uss + uss_offset) % 4;
    
    if (uss_distances[uss] != -1 {
      uint16_t dist = uss_distances[uss] + USS_DIST_ADJ_cm;
//    xy[uss][X_IDX] = cos(theta) * dist;
//    xy[uss][Y_IDX] = sin(theta) * dist;
    
      adj_dists[uss] = dist;
    }
  }

  uint16_t max_fb_uss = adj_dists[USS_F] > adj_dists[USS_B] ? USS_F : USS_B;
  uint16_t max_lr_uss = adj_dists[USS_L] > adj_dists[USS_R] ? USS_L : USS_R;
  uint16_t max_uss = adj_dists[max_fb_uss] > adj_dists[max_lr_uss] ?
      max_fb_uss : max_lr_uss;

  uint16_t board_x = 0, board_y = 0;
  while () {
    int16_t wall_left_dist = BOARD_LEFT_WALL_X - board_x,
        wall_right_dist = (-BOARD_RIGHT_WALL_X) + board_x;

    uint16_t q1_theta_rad = radians(theta % 45);
    uint8_t uss = uss_rotated[USS_F];
    if (adj_dists[uss] != -1) {
      uint16_t test_y = tan(q1_theta_rad) * x;
      uint16_t test_dist;
      if (test_y < BOARD_TOP_Y) {
        test_dist = test_y / sin(q1_theta_rad);
      }
      else {
        test_dist = (q1_theta_rad
      }
    }
  }
}
*/
