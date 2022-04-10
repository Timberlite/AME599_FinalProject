% Simulation Parameter
Kp_ground = 1e6;
Kd_ground = 1e3;
mu_static = 1.0;
mu_dynamic = 1.0;
dt = 0.005;
Horizon = 10;
dt_MPC = 0.04;


% Robot Parameter
l_trunk_hip_offset_x = 0.1805;
l_trunk_hip_offset_y = 0.047;
l_hip = 0.0838; % hip length
l_thigh = 0.2; % thigh length
l_calf = 0.2;  % calf length
Torque_max = 33.5;
Torque_min = -33.5;

% Robot Initial State
P_trunk = [0, 0, 0.11];               % [x y z]
R_trunk = eul2rotm([0, 0, 0]);  % [yaw pitch roll]
q0 = [0, 1.09, -2.69653369433;    % [FL_hip FL_thigh FL_calf
      0, 1.09, -2.69653369433;    %  FR_hip FR_thigh FR_calf
      0, 1.09, -2.69653369433;    %  RL_hip RL_thigh RL_calf
      0, 1.09, -2.69653369433];   %  RR_hip RR_thigh RR_calf]
        
        
        