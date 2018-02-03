function runSim()

%carLength, noise, position_theta, position_x, position_y, velocity, steering_angle
  
  [carLength, noise, position_theta, position_x, position_y, velocity, steering_angle] = acerkmannSim([0.4, 0, 0, 0, 0, 2, (pi / 4)]);
  for i = 0:100
    [carLength, noise, position_theta, position_x, position_y, velocity, steering_angle] = acerkmannSim([carLength, noise, position_theta, position_x, position_y, velocity, steering_angle]);
  endfor
endfunction