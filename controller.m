%inputs desired and estimated parameters and outputs body frame
%acceleration (forward acceleration) and steering angle

%test
sum_error_dt = [0; 0];
est_state = [0; 0; 1; 1; 1; 1];
% carLength = 0.4;
steering_angle = pi/5;
velocity = 2;
desired_state = [.2; .3; 1.5; .6; 1; 1];

[T, phi] = controlle(.05, sum_error_dt, est_state, steering_angle, desired_state)

function [T, phi, sum_error_dt] = controlle(dt, sum_error_dt, est_state, steering_angle, desired_state)
%     radius = carLength/sin(steering_angle);
%     a_centripetal = velocity*velocity/radius;
    
    rotation_matrix = [cos(steering_angle), -sin(steering_angle);
                        sin(steering_angle), cos(steering_angle)];
    
    error = rotation_matrix * ([desired_state(1); desired_state(2)] - [est_state(1); est_state(2)]);

%     est_velocity = [velocity * sin(steering_angle); velocity * cos(steering_angle)];

    error_dot = [desired_state(3); desired_state(4)] - [est_state(3); est_state(4)];
    
    K_P = [1, 1; 1, 1];
    K_D = [1, 0; 0, 1];
    K_i = [1, 0; 0, 1];

    sum_error_dt = sum_error_dt + error * dt;

    output = [desired_state(5); desired_state(6)] + K_P * error + K_i * sum_error_dt + K_D * error_dot;

    T = output(1);
    phi = output(2);
end