close all
    
    %params
    carLength = 0.4;
    carWidth = 0.2;
    mass = 4;
    global_state = [1, 1, 0, 0, 0, 0, 0]'; %x y theta dx dy ax ay
    dt = 0.01;
    integral = zeros(2, 1);
    

    for t = 0:dt:10
        
        desired_state = [1.5, 1, 0, 0, 0, 0, 0]'; % x y theta bdx bdy bax bay
        
        
        T = 0;
        phi = 0;
        
        est_state = sense(global_state)
        theta = global_state(3);
        transMatrix = [cos(theta), -sin(theta), global_state(1); sin(theta), cos(theta), global_state(2); 0, 0, 1]
        
        [T, phi, integral] = controller(dt, integral, est_state, desired_state);
        
        [global_state] = ackermannSim([T;phi], global_state, dt, mass, carLength)
        backRight = transMatrix * [0 , carWidth / 2 , 1]';
        backLeft = transMatrix * [0, -carWidth / 2 , 1]';
        frontRight = transMatrix * [carLength, carWidth / 2, 1]';
        frontLeft = transMatrix * [carLength, -carWidth / 2, 1]';        
        clf
        hold on
        %quiver(global_state(1, 1) + carWidth/2, global_state(2, 1) + carLength, cos(global_state(3)) + cos(phi), sin(global_state(3)) + sin(phi))
        
        quiver(backRight(1), backRight(2), cos(global_state(3)) * 0.2, sin(global_state(3)) * 0.2) %back right
        quiver(backLeft(1), backLeft(2), cos(global_state(3)) * 0.2, sin(global_state(3)) * 0.2) %back left
        quiver(frontRight(1), frontRight(2), cos(global_state(3) + phi) * 0.2, sin(global_state(3) + phi) * 0.2) %front right
        quiver(frontLeft(1), frontLeft(2), cos(global_state(3) + phi)* 0.2, sin(global_state(3) + phi) * 0.2) %front left

        

        
 
        xlim([0 5]);
        ylim([0 5]);
        drawnow;
    end
