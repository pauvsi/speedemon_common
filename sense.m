function [state_est] = sense(global_state)
  
  state_est = zeros(7, 1);
  
  state_est(1, 1) = global_state(1);
  state_est(2, 1) = global_state(2);
  
  theta = global_state(3);
  
  R = [cos(theta), -sin(theta); sin(theta), cos(theta)];
  
  state_est(3) = theta;
  
  state_est(4:5, 1) = R' * global_state(3:4, 1);
  
  state_est(6:7, 1) = R' * zeros(2, 1); %TODO fix me you c__t
  
  
end