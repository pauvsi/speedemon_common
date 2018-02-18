function runSim()
    %params
    carLength = 0.4;
    noise = 0;
    position_theta = 0;
    position_x = 0;
    position_y = 0;
    mass = 50;
    T = 1;
    state = [position_x,position_y];
    

    %velocity
    velocity = -2; %m/s rear
    steering_angle = pi/4; %rads
    dt = 0.05;

    for t = 0:100
        [mass, carLength, noise, position_theta, state, velocity, steering_angle] = ackermannSim(carLength, mass, noise, position_theta, state, velocity, steering_angle, dt);
        
        xlim([-10 10]);
        ylim([-10 10]);
        drawnow;
    end
end