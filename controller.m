%inputs desired and estimated parameters and outputs body frame
%acceleration (forward acceleration) and steering angle

%test
% sum_error_dt = [0; 0];
% est_state = [0; 0; 0; 1; 1; 1; 1]; %x, y, theta(body frame to global frame), bdx, bdy, bax, bay
% desired_state = [0; 0; 0; 1; 1; 1; 1]; %x, y, theta(body frame to global frame), bdx, bdy, bax, bay
% 
% [T, phi, sum_error_dt] = controlle(.05, sum_error_dt, est_state, desired_state)

function [T, phi, integral] = controller(dt, integral, est_state, desired_state)
%     radius = carLength/sin(steering_angle);
%     a_centripetal = velocity*velocity/radius;
    
    %parameters
    carLength = 0.4;
    mass = 4;
    min_turning_rad = 1;
    min_thrust = -20;
    max_thrust = 20;
    
    theta = est_state(3); %angle between body frame and global axes
    rotation_matrix = [cos(theta), -sin(theta); 
                        sin(theta), cos(theta)];
    
    error = transpose(rotation_matrix) * ([desired_state(1); desired_state(2)] - [est_state(1); est_state(2)]);

    error_dot = [desired_state(4); desired_state(5)] - [est_state(4); est_state(5)];
    
    %gain coefficients
    K_P = [0.1, 0; 0, 0.1];
    K_D = [0.1, 0; 0, 0.1];
    K_i = [0.1, 0; 0, 0.1];

    integral = integral + error * dt; %updates integral term
    
    output = [desired_state(6); desired_state(7)] + K_P * error + K_i * integral + K_D * error_dot
    
    radius = 0;
    
    %if centripetal acceleration is 0
    if(output(2) == 0)
        radius = 1e12
    else
        radius = est_state(4)^2/output(2)
    end
    
    T = mass * output(1); %thrust
    
    %constrain
    if(abs(radius) < min_turning_rad)
        radius = min_turning_rad;
    end
    
    if(T < min_thrust)
        T = min_thrust;
    elseif(T > max_thrust)
        T = max_thrust;
    end
    
    phi = atan(carLength/radius); %steering angle
end