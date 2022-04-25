function F = Force_Control(t, P_trunk, Q_trunk, V_trunk, omega_trunk, P_foot)
    global sim_params;
    setup_time = sim_params.setup_time;      % (s) time for initial settling
    gait_cycle = sim_params.gait_cycle;      % (s) time for one gait cycle
    default_trunk_height = sim_params.default_trunk_height;
    
    Euler_trunk = (quat2eul(Q_trunk.','XYZ')).';
    
%     % trotting
%     gait1 = blkdiag(zeros(3),eye(3),eye(3),zeros(3));
%     gait2 = blkdiag(eye(3),zeros(3),zeros(3),eye(3));
%     trotting_gait = blkdiag(kron(eye(5),gait1),kron(eye(5),gait2));
    
    % bounding
    gait1 = blkdiag(eye(3),eye(3),zeros(3),zeros(3));
    gait2 = blkdiag(eye(3),eye(3),eye(3),eye(3));
    gait3 = blkdiag(zeros(3),zeros(3),eye(3),eye(3));
    bounding_gait = blkdiag(kron(eye(9),gait1),gait2,gait2,kron(eye(9),gait3));
    
    if t<setup_time
        % standing with QP
        F = QP_Control(P_trunk, Q_trunk, V_trunk, omega_trunk, P_foot);

    else
        t = t-setup_time;
        
%         % Task2
%         if t<4
%             P_trunk_des = [0;0;default_trunk_height];
%             Euler_trunk_des = [0;0;0];
%             V_trunk_des = [0;0;0];
%             omega_trunk_des = [0;0;0];
%             trajectory = Compute_Trajectory(P_trunk_des, Euler_trunk_des, V_trunk_des, omega_trunk_des);
%             Q = diag([400 400 3600 400 400 100 4 4 25 0.01 0.01 1 0]);
%         elseif t<8
%             P_trunk_des = [P_trunk(1);0;default_trunk_height];
%             Euler_trunk_des = [0;0;0];
%             V_trunk_des = [1;0;0];
%             omega_trunk_des = [0;0;0];
%             trajectory = Compute_Trajectory(P_trunk_des, Euler_trunk_des, V_trunk_des, omega_trunk_des);
%             Q = diag([400 400 3600 400 400 100 400 400 1 0.01 0.01 0.01 0]);
%         elseif t<12
%             P_trunk_des = [P_trunk(1);0;default_trunk_height];
%             Euler_trunk_des = [0;0;0];
%             V_trunk_des = [0;0;0];
%             omega_trunk_des = [0;0;0];
%             trajectory = Compute_Trajectory(P_trunk_des, Euler_trunk_des, V_trunk_des, omega_trunk_des);
%             Q = diag([400 400 3600 400 400 100 4 4 25 0.01 0.01 1 0]);
%         elseif t<16
%             P_trunk_des = [P_trunk(1);0;default_trunk_height];
%             Euler_trunk_des = [0;0;0];
%             V_trunk_des = [-1;0;0];
%             omega_trunk_des = [0;0;0];
%             trajectory = Compute_Trajectory(P_trunk_des, Euler_trunk_des, V_trunk_des, omega_trunk_des);
%             Q = diag([400 400 3600 400 400 100 400 400 1 0.01 0.01 0.01 0]);
%         elseif t<20
%             P_trunk_des = [0;0;default_trunk_height];
%             Euler_trunk_des = [0;0;0];
%             V_trunk_des = [0;0;0];
%             omega_trunk_des = [0;0;0];
%             trajectory = Compute_Trajectory(P_trunk_des, Euler_trunk_des, V_trunk_des, omega_trunk_des);
%             Q = diag([400 400 3600 400 400 100 4 4 25 0.01 0.01 1 0]);  
%         elseif t<24
%             P_trunk_des = [0;P_trunk(2);default_trunk_height];
%             Euler_trunk_des = [0;0;0];
%             V_trunk_des = [0;0.5;0];
%             omega_trunk_des = [0;0;0];
%             trajectory = Compute_Trajectory(P_trunk_des, Euler_trunk_des, V_trunk_des, omega_trunk_des);
%             Q = diag([400 400 3600 400 400 100 400 400 1 0.01 0.01 0.01 0]);
%         elseif t<28
%             P_trunk_des = [0;P_trunk(2);0.3];
%             Euler_trunk_des = [0;0;0];
%             V_trunk_des = [0;0;0];
%             omega_trunk_des = [0;0;0];
%             trajectory = Compute_Trajectory(P_trunk_des, Euler_trunk_des, V_trunk_des, omega_trunk_des);
%             Q = diag([400 400 3600 400 400 100 4 4 25 0.01 0.01 1 0]);  
%         elseif t<32
%             P_trunk_des = [0;P_trunk(2);default_trunk_height];
%             Euler_trunk_des = [0;0;0];
%             V_trunk_des = [0;-0.5;0];
%             omega_trunk_des = [0;0;0];
%             trajectory = Compute_Trajectory(P_trunk_des, Euler_trunk_des, V_trunk_des, omega_trunk_des);
%             Q = diag([400 400 3600 400 400 100 400 400 1 0.01 0.01 0.01 0]);
%         elseif t<36
%             P_trunk_des = [0;0;default_trunk_height];
%             Euler_trunk_des = [0;0;0];
%             V_trunk_des = [0;0;0];
%             omega_trunk_des = [0;0;0];
%             trajectory = Compute_Trajectory(P_trunk_des, Euler_trunk_des, V_trunk_des, omega_trunk_des);
%             Q = diag([400 400 3600 400 400 100 4 4 25 0.01 0.01 1 0]);  
%         elseif t<42
%             P_trunk_des = [0;0;default_trunk_height];
%             Euler_trunk_des = [0;0;Euler_trunk(3)];
%             V_trunk_des = [0;0;0];
%             omega_trunk_des = [0;0;1.31];
%             trajectory = Compute_Trajectory(P_trunk_des, Euler_trunk_des, V_trunk_des, omega_trunk_des);
%             Q = diag([400 400 3600 400 400 100 4 4 25 0.01 0.01 1 0]);  
%         else
%             P_trunk_des = [0;0;default_trunk_height];
%             Euler_trunk_des = [0;0;0];
%             V_trunk_des = [0;0;0];
%             omega_trunk_des = [0;0;0];
%             trajectory = Compute_Trajectory(P_trunk_des, Euler_trunk_des, V_trunk_des, omega_trunk_des);
%             Q = diag([400 400 3600 400 400 100 4 4 25 0.01 0.01 1 0]);  
%         end
        
        % Task3
        if t<1.6
            P_trunk_des = [P_trunk(1);0;default_trunk_height];
            Euler_trunk_des = [0;0;0];
            V_trunk_des = [1+1.75*t;0;0];
            omega_trunk_des = [0;0;0];
            trajectory = Compute_Trajectory(P_trunk_des, Euler_trunk_des, V_trunk_des, omega_trunk_des);
            Q = diag([100 400 4900 400 400 100 4 1 1 0.01 0.01 0.01 0]);
        else
            P_trunk_des = [P_trunk(1);0;default_trunk_height];
            Euler_trunk_des = [0;0;0];
            V_trunk_des = [3.8;0;0];
            omega_trunk_des = [0;0;0];
            trajectory = Compute_Trajectory(P_trunk_des, Euler_trunk_des, V_trunk_des, omega_trunk_des);
            Q = diag([100 400 4900 400 400 100 4 1 1 0.01 0.01 0.01 0]);
        end
      
        t = rem(t, gait_cycle);
        F = MPC_Control(t, P_trunk, Q_trunk, V_trunk, omega_trunk, P_foot, trajectory, bounding_gait, Q);
    end
end

