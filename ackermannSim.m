function [carLength, noise, position_theta, position_x, position_y, velocity, steering_angle] = ackermannSim(carLength, noise, position_theta, position_x, position_y, velocity, steering_angle, delta_time)

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
    xlim([-10 10]);
    ylim([-10 10]);
  
end
  
 
  












