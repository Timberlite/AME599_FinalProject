% Simulation Parameter
global sim_params;
sim_params.Kp_ground = 1e6;
sim_params.Kd_ground = 1e3;
sim_params.mu_static = 1.5;
sim_params.mu_dynamic = 1.4;
sim_params.g = 9.81;
sim_params.dt = 0.001;
sim_params.Horizon = 10;
sim_params.dt_MPC = 0.04;
sim_params.setup_time = 2.0;
sim_params.gait_cycle = 0.4;
sim_params.default_trunk_height = 0.3;
sim_params.default_step_height = 0.08;

% Robot Parameter
global robot_params;
robot_params.m_trunk = 6.0;
robot_params.m_hip = 0.696;
robot_params.m_thigh = 1.013;
robot_params.m_calf = 0.166;
% robot_params.I_b = diag([0.0158533, 0.0377999, 0.0456542]);
robot_params.I_b = diag([0.016839930, 0.056579028, 0.064713601]);
robot_params.l_trunk_hip_offset_x = 0.1805;
robot_params.l_trunk_hip_offset_y = 0.047;
robot_params.l_hip = 0.0838; % hip length
robot_params.l_thigh = 0.2; % thigh length
robot_params.l_calf = 0.2;  % calf length
robot_params.Torque_max = 33.5;
robot_params.Torque_min = -33.5;

% Robot Initial State
% lay on the ground
P_trunk = [-0.03, 0, 0.11];           % [x y z]
R_trunk = eul2rotm([0, 0, 0]);    % [yaw pitch roll]
q0 = [0, 1.09, -2.69653369433;    % [FL_hip FL_thigh FL_calf
      0, 1.09, -2.69653369433;    %  FR_hip FR_thigh FR_calf
      0, 1.09, -2.69653369433;    %  RL_hip RL_thigh RL_calf
      0, 1.09, -2.69653369433];   %  RR_hip RR_thigh RR_calf]
        
        
        