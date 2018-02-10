function runSim()
    %params
    carLength = 0.4;
    noise = 0;
    position_theta = 0;
    position_x = 0;
    position_y = 0;

    %velocity
    velocity = 2; %m/s rear
    steering_angle = pi/10; %rads
    dt = .05;

    for t = 0:1000
        [carLength, noise, position_theta, position_x, position_y, velocity, steering_angle] = ackermannSim(carLength, noise, position_theta, position_x, position_y, velocity, steering_angle, dt);
        
%         if(mod(t,2) == 0)
%         steering_angle = steering_angle + pi/30;
%         else
%         steering_angle = steering_angle - pi/20;
%         end
   
        drawnow;
%         pause(0.05)
    end
end