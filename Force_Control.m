function F = Force_Control(t, P_trunk, Q_trunk, V_trunk, omega_trunk, P_foot)
    global sim_params;
    setup_time = sim_params.setup_time;      % (s) time for initial settling
    gait_cycle = sim_params.gait_cycle;      % (s) time for one gait cycle
    default_trunk_height = sim_params.default_trunk_height;
    
    Euler_trunk = (quat2eul(Q_trunk.','XYZ')).';
    
    % trotting
    gait1 = blkdiag(zeros(3),eye(3),eye(3),zeros(3));
    gait2 = blkdiag(eye(3),zeros(3),zeros(3),eye(3));
    trotting_gait = blkdiag(kron(eye(10),gait1),kron(eye(10),gait2));
    
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
            Q = diag([400 400 3600 400 400 100 4 4 25 0.01 0.01 1 0]);
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
        
%         % Task3
%         if t<1.6
%             P_trunk_des = [P_trunk(1);0;default_trunk_height];
%             Euler_trunk_des = [0;0;0];
%             V_trunk_des = [1+1.75*t;0;0];
%             omega_trunk_des = [0;0;0];
%             trajectory = Compute_Trajectory(P_trunk_des, Euler_trunk_des, V_trunk_des, omega_trunk_des);
%             Q = diag([100 400 4900 400 400 100 4 1 1 0.01 0.01 0.01 0]);
%         else
%             P_trunk_des = [P_trunk(1);0;default_trunk_height];
%             Euler_trunk_des = [0;0;0];
%             V_trunk_des = [3.8;0;0];
%             omega_trunk_des = [0;0;0];
%             trajectory = Compute_Trajectory(P_trunk_des, Euler_trunk_des, V_trunk_des, omega_trunk_des);
%             Q = diag([100 400 4900 400 400 100 4 1 1 0.01 0.01 0.01 0]);
%         end

%         % Task4
%         if t<4.8
%             P_trunk_des = [0.5*t-0.43;0;default_trunk_height-0.01];
%             Euler_trunk_des = [0;0;0];
%             V_trunk_des = [0.5;0;0];
%             omega_trunk_des = [0;0;0];
%             trajectory = Compute_Trajectory(P_trunk_des, Euler_trunk_des, V_trunk_des, omega_trunk_des);
%             Q = diag([100 400 4900 400 400 100 100 60 1 0.01 0.01 0.01 0]);
%         elseif t<9.8
%             P_trunk_des = [0.5*t-0.43;0;default_trunk_height+0.01];
%             Euler_trunk_des = [0;0;0];
%             V_trunk_des = [0.5;0;0];
%             omega_trunk_des = [0;0;0];
%             trajectory = Compute_Trajectory(P_trunk_des, Euler_trunk_des, V_trunk_des, omega_trunk_des);
%             Q = diag([100 400 4900 400 400 100 100 60 1 0.01 0.01 0.01 0]);
%         elseif t<11.2
%             P_trunk_des = [0.5*t-0.43;0;default_trunk_height];
%             Euler_trunk_des = [0;0;0];
%             V_trunk_des = [0.5;0;0];
%             omega_trunk_des = [0;0;0];
%             trajectory = Compute_Trajectory(P_trunk_des, Euler_trunk_des, V_trunk_des, omega_trunk_des);
%             Q = diag([100 400 4900 400 400 100 100 60 1 0.01 0.01 0.01 0]);
%         elseif t<12.8
%             P_trunk_des = [0.5*t-0.43;0;default_trunk_height+0.02];
%             Euler_trunk_des = [0;0;0];
%             V_trunk_des = [0.5;0;0];
%             omega_trunk_des = [0;0;0];
%             trajectory = Compute_Trajectory(P_trunk_des, Euler_trunk_des, V_trunk_des, omega_trunk_des);
%             Q = diag([100 400 4900 400 400 100 100 60 1 0.01 0.01 0.01 0]);
%         elseif t<17.2
%             P_trunk_des = [0.5*t-0.43;0;default_trunk_height];
%             Euler_trunk_des = [0;0;0];
%             V_trunk_des = [0.5;0;0];
%             omega_trunk_des = [0;0;0];
%             trajectory = Compute_Trajectory(P_trunk_des, Euler_trunk_des, V_trunk_des, omega_trunk_des);
%             Q = diag([100 400 4900 400 400 100 100 80 1 0.01 0.01 0.01 0]);
%         elseif t<20.4
%             P_trunk_des = [0.5*t-0.43;0;0.25*(t-17.2)+default_trunk_height-0.02];
%             Euler_trunk_des = [0;-0.464;0];
%             V_trunk_des = [0.5;0;0.25];
%             omega_trunk_des = [0;0;0];
%             trajectory = Compute_Trajectory(P_trunk_des, Euler_trunk_des, V_trunk_des, omega_trunk_des);
%             Q = diag([100 400 4900 400 400 100 100 120 40 0.01 0.01 0.01 0]);
%         elseif t<21.2
%             P_trunk_des = [0.5*t-0.43;0;0.125*(t-20.4)+0.8+default_trunk_height-0.01];
%             Euler_trunk_des = [0;-0.464+(t-20.4)/0.8*0.464;0];
%             V_trunk_des = [0.5;0;0.125];
%             omega_trunk_des = [0;0;0];
%             trajectory = Compute_Trajectory(P_trunk_des, Euler_trunk_des, V_trunk_des, omega_trunk_des);
%             Q = diag([100 400 4900 400 400 100 100 120 40 0.01 0.01 0.01 0]);
%         else
%             P_trunk_des = [0.5*t-0.43;0;0.9+default_trunk_height];
%             Euler_trunk_des = [0;0;0];
%             V_trunk_des = [0.5;0;0];
%             omega_trunk_des = [0;0;0];
%             trajectory = Compute_Trajectory(P_trunk_des, Euler_trunk_des, V_trunk_des, omega_trunk_des);
%             Q = diag([100 400 4900 400 400 100 100 120 40 0.01 0.01 0.01 0]);
%         end

        % Task 5
        if t<0.4
            P_trunk_des = [P_trunk(1);0;default_trunk_height];
            Euler_trunk_des = [0;0;0];
            V_trunk_des = [0.64+t*2.4;0;0];
            omega_trunk_des = [0;0;0];
            trajectory = Compute_Trajectory(P_trunk_des, Euler_trunk_des, V_trunk_des, omega_trunk_des);
            Q = diag([100 400 4900 400 400 100 4 1 1 0.01 0.01 0.01 0]);
        elseif t<0.8
            P_trunk_des = [P_trunk(1);0;default_trunk_height];
            Euler_trunk_des = [0;0;0];
            V_trunk_des = [1.6;0;0];
            omega_trunk_des = [0;0;0];
            trajectory = Compute_Trajectory(P_trunk_des, Euler_trunk_des, V_trunk_des, omega_trunk_des);
            Q = diag([100 400 4900 400 400 100 4 1 1 0.01 0.01 0.01 0]);
        elseif t<0.9
            P_trunk_des = [P_trunk(1);0;default_trunk_height];
            Euler_trunk_des = [0;-0.32;0];
            V_trunk_des = [1.6;0;0];
            omega_trunk_des = [0;0;0];
            trajectory = Compute_Trajectory(P_trunk_des, Euler_trunk_des, V_trunk_des, omega_trunk_des);
            Q = diag([100 400 4900 400 400 100 4 1 1 0.01 0.01 0.01 0]);
        elseif t<1.09
            P_trunk_des = [P_trunk(1);0;P_trunk(3)];
            Euler_trunk_des = [0;-0.32;0];
            V_trunk_des = [1.6;0;1.6];
            omega_trunk_des = [0;0;0];
            trajectory = Compute_Trajectory(P_trunk_des, Euler_trunk_des, V_trunk_des, omega_trunk_des);
            Q = diag([100 400 4900 400 400 100 4 1 4 0.01 0.01 0.01 0]);
        elseif t<1.48
            P_trunk_des = [P_trunk(1);0;default_trunk_height+0.2];
            Euler_trunk_des = [0;0;0];
            V_trunk_des = [1.6;0;0];
            omega_trunk_des = [0;0;0];
            trajectory = Compute_Trajectory(P_trunk_des, Euler_trunk_des, V_trunk_des, omega_trunk_des);
            Q = diag([100 400 4900 400 400 100 4 1 40 0.01 0.01 0.01 0]);
        elseif t<1.68
            P_trunk_des = [P_trunk(1);0;P_trunk(3)];
            Euler_trunk_des = [0;0.1;0];
            V_trunk_des = [1.6;0;0];
            omega_trunk_des = [0;0;0];
            trajectory = Compute_Trajectory(P_trunk_des, Euler_trunk_des, V_trunk_des, omega_trunk_des);
            Q = diag([100 400 4900 400 400 100 4 1 4 0.01 0.01 0.01 0]);
        elseif t<4.08
            P_trunk_des = [P_trunk(1);0;default_trunk_height-0.028];
            Euler_trunk_des = [0;0;0];
            V_trunk_des = [1.6;0;0];
            omega_trunk_des = [0;0;0];
            trajectory = Compute_Trajectory(P_trunk_des, Euler_trunk_des, V_trunk_des, omega_trunk_des);
            Q = diag([100 400 4900 400 400 100 4 1 1 0.01 0.01 0.01 0]);
        elseif t<4.4
            P_trunk_des = [P_trunk(1);0;default_trunk_height-0.02];
            Euler_trunk_des = [0;0;0];
            V_trunk_des = [1.7;0;0];
            omega_trunk_des = [0;0;0];
            trajectory = Compute_Trajectory(P_trunk_des, Euler_trunk_des, V_trunk_des, omega_trunk_des);
            Q = diag([100 400 4900 400 400 100 4 1 1 0.01 0.01 0.01 0]);
        elseif t<4.5
            P_trunk_des = [P_trunk(1);0;default_trunk_height-0.04];
            Euler_trunk_des = [0;-0.4;0];
            V_trunk_des = [1.7;0;0];
            omega_trunk_des = [0;0;0];
            trajectory = Compute_Trajectory(P_trunk_des, Euler_trunk_des, V_trunk_des, omega_trunk_des);
            Q = diag([100 400 4900 400 400 100 4 1 4 0.01 0.01 0.01 0]);
        elseif t<4.7
            P_trunk_des = [P_trunk(1);0;P_trunk(3)];
            Euler_trunk_des = [0;-0.4;0];
            V_trunk_des = [1.7;0;1.4];
            omega_trunk_des = [0;0;0];
            trajectory = Compute_Trajectory(P_trunk_des, Euler_trunk_des, V_trunk_des, omega_trunk_des);
            Q = diag([100 400 4900 400 400 100 4 1 4 0.01 0.01 0.01 0]);
        elseif t<4.83
            P_trunk_des = [P_trunk(1);0;P_trunk(3)];
            Euler_trunk_des = [0;0.1;0];
            V_trunk_des = [1.7;0;0];
            omega_trunk_des = [0;0;0];
            trajectory = Compute_Trajectory(P_trunk_des, Euler_trunk_des, V_trunk_des, omega_trunk_des);
            Q = diag([100 400 4900 400 400 100 4 1 4 0.01 0.01 0.01 0]);
        elseif t<5.03
            P_trunk_des = [P_trunk(1);0;P_trunk(3)];
            Euler_trunk_des = [0;0.3;0];
            V_trunk_des = [1.7;0;0];
            omega_trunk_des = [0;0;0];
            trajectory = Compute_Trajectory(P_trunk_des, Euler_trunk_des, V_trunk_des, omega_trunk_des);
            Q = diag([100 400 4900 400 400 100 4 1 4 0.01 0.01 0.01 0]);
        else
            P_trunk_des = [P_trunk(1);0;default_trunk_height];
            Euler_trunk_des = [0;0;0];
            V_trunk_des = [1.7;0;0];
            omega_trunk_des = [0;0;0];
            trajectory = Compute_Trajectory(P_trunk_des, Euler_trunk_des, V_trunk_des, omega_trunk_des);
            Q = diag([100 400 4900 400 400 100 4 1 1 0.01 0.01 0.01 0]);
        end
        
        F = MPC_Control(t, P_trunk, Q_trunk, V_trunk, omega_trunk, P_foot, trajectory, bounding_gait, Q);
    end
end

