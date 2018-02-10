function runSim()
    %params
    carLength = 0.4;
    noise = 0;
    position_theta = 0;
    position_x = 0;
    position_y = 0;

    %velocity
    velocity = 2; %m/s rear
    steering_angle = pi/4; %rads
    dt = 0.05;

    for t = 0:100
        [carLength, noise, position_theta, position_x, position_y, velocity, steering_angle] = ackermannSim(carLength, noise, position_theta, position_x, position_y, velocity, steering_angle, dt);
        drawnow;
    end
end