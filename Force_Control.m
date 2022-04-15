function F = Force_Control(t, P_trunk, Q_trunk, V_trunk, omega_trunk, P_foot)
    global sim_params;
    setup_time = sim_params.setup_time;      % (s) time for initial settling
    gait_cycle = sim_params.gait_cycle;      % (s) time for one gait cycle
    g = sim_params.g;
    N = sim_params.Horizon;
    
    Euler_trunk = (quat2eul(Q_trunk.','XYZ')).';
    
    % trotting
    gait1 = blkdiag(zeros(3),eye(3),eye(3),zeros(3));
    gait2 = blkdiag(eye(3),zeros(3),zeros(3),eye(3));
    trotting_gait = blkdiag(kron(eye(N/2),gait1),kron(eye(N/2),gait2));
    
    if t<setup_time
        % standing with QP
        F = QP_Control(P_trunk, Q_trunk, V_trunk, omega_trunk, P_foot);

    else
        t = t-setup_time;
        
        if t<2
            P_trunk_des = [0;0;0.3];
            Euler_trunk_des = [0;0;0];
            V_trunk_des = [0;0;0];
            omega_trunk_des = [0;0;0];
            trajectory = Compute_Trajectory(P_trunk_des, Euler_trunk_des, V_trunk_des, omega_trunk_des);
            Q = diag([400 400 2500 1600 3600 100 4 4 25 16 36 1 0]);
            Alpha = 0.001;
        elseif t<6
            P_trunk_des = [P_trunk(1);0;0.3];
            Euler_trunk_des = [0;0;0];
            V_trunk_des = [1;0;0];
            omega_trunk_des = [0;0;0];
            trajectory = Compute_Trajectory(P_trunk_des, Euler_trunk_des, V_trunk_des, omega_trunk_des);
            Q = diag([100 400 2500 1600 3600 100 400 4 25 16 36 1 0]);
            Alpha = 0.001;
        elseif t<8
            P_trunk_des = [4;0;0.3];
            Euler_trunk_des = [0;0;0];
            V_trunk_des = [0;0;0];
            omega_trunk_des = [0;0;0];
            trajectory = Compute_Trajectory(P_trunk_des, Euler_trunk_des, V_trunk_des, omega_trunk_des);
            Q = diag([400 400 2500 1600 3600 100 4 4 25 16 36 1 0]);
            Alpha = 0.001;
        elseif t<12
            P_trunk_des = [P_trunk(1);0;0.3];
            Euler_trunk_des = [0;0;0];
            V_trunk_des = [-1;0;0];
            omega_trunk_des = [0;0;0];
            trajectory = Compute_Trajectory(P_trunk_des, Euler_trunk_des, V_trunk_des, omega_trunk_des);
            Q = diag([100 400 2500 1600 3600 100 400 4 25 16 36 1 0]);
            Alpha = 0.001;
        elseif t<16
            P_trunk_des = [0;0;0.3];
            Euler_trunk_des = [0;0;0];
            V_trunk_des = [0;0;0];
            omega_trunk_des = [0;0;0];
            trajectory = Compute_Trajectory(P_trunk_des, Euler_trunk_des, V_trunk_des, omega_trunk_des);
            Q = diag([400 400 2500 1600 3600 100 4 4 25 16 36 1 0]);
            Alpha = 0.001;    
        elseif t<20
            P_trunk_des = [0;P_trunk(2);0.3];
            Euler_trunk_des = [0;0;0];
            V_trunk_des = [0;0.5;0];
            omega_trunk_des = [0;0;0];
            trajectory = Compute_Trajectory(P_trunk_des, Euler_trunk_des, V_trunk_des, omega_trunk_des);
            Q = diag([400 100 2500 1600 3600 100 4 400 25 16 36 1 0]);
            Alpha = 0.001;
        elseif t<22
            P_trunk_des = [0;2;0.3];
            Euler_trunk_des = [0;0;0];
            V_trunk_des = [0;0;0];
            omega_trunk_des = [0;0;0];
            trajectory = Compute_Trajectory(P_trunk_des, Euler_trunk_des, V_trunk_des, omega_trunk_des);
            Q = diag([400 400 2500 1600 3600 100 4 4 25 16 36 1 0]);
            Alpha = 0.001;
        elseif t<26
            P_trunk_des = [0;P_trunk(2);0.3];
            Euler_trunk_des = [0;0;0];
            V_trunk_des = [0;-0.5;0];
            omega_trunk_des = [0;0;0];
            trajectory = Compute_Trajectory(P_trunk_des, Euler_trunk_des, V_trunk_des, omega_trunk_des);
            Q = diag([400 100 2500 1600 3600 100 4 400 25 16 36 1 0]);
            Alpha = 0.001;
        elseif t<30
            P_trunk_des = [0;0;0.3];
            Euler_trunk_des = [0;0;0];
            V_trunk_des = [0;0;0];
            omega_trunk_des = [0;0;0];
            trajectory = Compute_Trajectory(P_trunk_des, Euler_trunk_des, V_trunk_des, omega_trunk_des);
            Q = diag([400 400 2500 1600 3600 100 4 4 25 16 36 1 0]);
            Alpha = 0.001;
        elseif t<34
            P_trunk_des = [0;0;0.3];
            Euler_trunk_des = [0;0;Euler_trunk(3)];
            V_trunk_des = [0;0;0];
            omega_trunk_des = [0;0;0.5];
            trajectory = Compute_Trajectory(P_trunk_des, Euler_trunk_des, V_trunk_des, omega_trunk_des);
            Q = diag([400 400 2500 1600 3600 100 4 4 25 16 36 400 0]);
            Alpha = 0.001;
        elseif t<38
            P_trunk_des = [0;0;0.3];
            Euler_trunk_des = [0;0;Euler_trunk(3)];
            V_trunk_des = [0;0;0];
            omega_trunk_des = [0;0;-0.5];
            trajectory = Compute_Trajectory(P_trunk_des, Euler_trunk_des, V_trunk_des, omega_trunk_des);
            Q = diag([400 400 2500 1600 3600 100 4 4 25 16 36 400 0]);
            Alpha = 0.001;
        else
            P_trunk_des = [0;0;0.3];
            Euler_trunk_des = [0;0;0];
            V_trunk_des = [0;0;0];
            omega_trunk_des = [0;0;0];
            trajectory = Compute_Trajectory(P_trunk_des, Euler_trunk_des, V_trunk_des, omega_trunk_des);
            Q = diag([400 400 2500 1600 3600 100 4 4 25 16 36 1 0]);
            Alpha = 0.001;
        end
        
      
        t = rem(t, gait_cycle);
        F = MPC_Control(t, P_trunk, Q_trunk, V_trunk, omega_trunk, P_foot, trajectory, trotting_gait, Q, Alpha);
    end
end

