% Simulation Parameter
global sim_params;
sim_params.Kp_ground = 1e6;
sim_params.Kd_ground = 1e3;
sim_params.mu_static = 1.9;
sim_params.mu_dynamic = 1.8;
sim_params.g = 9.81;
sim_params.dt = 0.001;
sim_params.Horizon = 20;
sim_params.dt_MPC = 0.015;
sim_params.setup_time = 2.0;
sim_params.gait_cycle = 0.3;
sim_params.default_trunk_height = 0.3;
sim_params.default_step_height = 0.10;
sim_params.obstacle_height = 0.3;

% Robot Parameter
global robot_params;
robot_params.m_trunk = 6.0;
robot_params.m_hip = 0.696;
robot_params.m_thigh = 1.013;
robot_params.m_calf = 0.166;
robot_params.I_b = diag([0.016839930, 0.056579028, 0.064713601]);
robot_params.l_trunk_hip_offset_x = 0.1805;
robot_params.l_trunk_hip_offset_y = 0.047;
robot_params.l_hip = 0.0838; % hip length
robot_params.l_thigh = 0.2; % thigh length
robot_params.l_calf = 0.2;  % calf length
robot_params.Torque_max = 33.5;
robot_params.Torque_min = -33.5;

robot_params.q_max_hip = 0.802851455917;
robot_params.q_min_hip = -0.802851455917;
robot_params.q_max_thigh = 4.18879020479;
robot_params.q_min_thigh = -1.0471975512;
robot_params.q_max_calf = -0.916297857297;
robot_params.q_min_calf = -2.69653369433;
robot_params.dq_max = 21;
robot_params.dq_min = -21;

% Robot Initial State
% lay on the ground
P_trunk = [-0.35, 0, 0.30];           % [x y z]
R_trunk = eul2rotm([0, 0, 0]);    % [yaw pitch roll]
q0 = [0, pi/4, -pi/2;    % [FL_hip FL_thigh FL_calf
      0, pi/4, -pi/2;    %  FR_hip FR_thigh FR_calf
      0, pi/4, -pi/2;    %  RL_hip RL_thigh RL_calf
      0, pi/4, -pi/2];   %  RR_hip RR_thigh RR_calf]
        
        
        