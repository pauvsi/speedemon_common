function [carLength, noise, position_theta, position_x, position_y, velocity, steering_angle] = ackermannSim(paramVector)
  %some params
  
  carLength =  paramVector(1); %metersvasdfadsasdf
  noise = paramVector(2);
  position_theta = paramVector(3);
  position_x = paramVector(4);
  position_y = paramVector(5);
  
  %velocity
  velocity = paramVector(6); %m/s rear
  steering_angle = paramVector(7); %rads
  delta_time = 1; %secs
  
  %angle
  delta_theta = (velocity/carLength) * tan(steering_angle);
  
  %odometry
  delta_x = velocity * cos(position_theta + ((delta_theta * delta_time) / 2.0));
  delta_y = velocity * sin(position_theta + ((delta_theta * delta_time) / 2.0));
  
  %pose estimate update
  position_x = position_x + delta_x * delta_time;
  position_y = position_y + delta_y * delta_time;
  position_theta = position_theta + delta_theta * delta_time;
  
  %actual vector
  ackermannVector = [position_x, position_y, cos(position_theta), sin(position_theta)];
  
  quiver(position_x, position_y, cos(position_theta), sin(position_theta));
  xlim([-5 10]);
  ylim([-5 10]);
endfunction
  
 
  












