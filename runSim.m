
    %params
    carLength = 0.4;
    mass = 4;
    global_state = [1, 1, 0, 0, 0, 0, 0]'; %x y theta dx dy ax ay
    dt = 0.01;
    integral = zeros(2, 1);

    for t = 0:dt:10
        
        desired_state = [1.5, 1, 0, 0, 0, 0, 0]'; % x y theta bdx bdy bax bay
        
        T = 0;
        phi = 0;
        
        est_state = sense(global_state)
        
        [T, phi, integral] = controller(dt, integral, est_state, desired_state);
        
        [global_state] = ackermannSim([T;phi], global_state, dt, mass, carLength)        

  
        quiver(global_state(1, 1), global_state(2, 1), cos(global_state(3)), sin(global_state(3)))
 
        xlim([0 5]);
        ylim([0 5]);
        drawnow;
    end
