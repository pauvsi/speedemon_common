function [globalState] = ackermannSim(desiredControlVector, mass, carLength, globalState, dt)
    
    %globalState = [x, y, dx, dy, T, phi) 
    %desiredControlVector = [T, phi]
    globalState(5) = globalState(5) + 0.98 * (desiredControlVector(1) - globalState(5)); %T, x, T
    globalState(6) = globalState(6) + 0.5 * (desiredControlVector(2) - globalState(6)); %phi, Y, phi
    
    theta = atan2(globalState(4), globalState(3)); % intertial dy, dx
    R = [cos(theta) , -sin(theta); sin(theta) , cos(theta)]; %rotation matrix from inertial coordinate frame to body coordinate frame
    
    r = carLength / tan(globalState(6); %turn radius (denom is phi)
    
    if (abs(r) < 1.1)
      r = 1.1;
    end
    
    if (globalState(5) < -20)
      globalState(5) = -20;
    end
    
    if (globalState(5) > 20)
      globalState(5) = 20;
    end
      
    intertialAcceleration = R([globalState(5) / mass; 0] + [0 ; globalState(3)^2 / r);
    
    globalState(1:2, 1) = globalState(1:2, 1) + dt * globalState(3:4, 1) + 0.5 * dt * dt * inertialAcceleration;
    globalState(3:4, 1) = globalState(3:4, 1) + intertialAcceleration * dt;
  
end
  
 
  












