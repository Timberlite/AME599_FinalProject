function F  = Cartesian_Control(t, Q_trunk, V_trunk, P_foot, V_foot, P_hip)
    global sim_params;
    setup_time = sim_params.setup_time;      % (s) time for initial settling
    gait_cycle = sim_params.gait_cycle;      % (s) time for one gait cycle
    ground_offset = 0.020; % (m) P_foot height at contact
    step_height = sim_params.default_step_height;
    
    R_trunk = quat2rotm(Q_trunk.');

    P_foot_FL = [P_foot(1);
                 P_foot(2);
                 P_foot(3)];
    P_foot_FR = [P_foot(4);
                 P_foot(5);
                 P_foot(6)];
    P_foot_RL = [P_foot(7);
                 P_foot(8);
                 P_foot(9)];
    P_foot_RR = [P_foot(10);
                 P_foot(11);
                 P_foot(12)];
    V_foot_FL = [V_foot(1);
                 V_foot(2);
                 V_foot(3)];
    V_foot_FR = [V_foot(4);
                 V_foot(5);
                 V_foot(6)];
    V_foot_RL = [V_foot(7);
                 V_foot(8);
                 V_foot(9)];
    V_foot_RR = [V_foot(10);
                 V_foot(11);
                 V_foot(12)];
    P_hip_FL = [P_hip(1);
                P_hip(2);
                P_hip(3)];
    P_hip_FR = [P_hip(4);
                P_hip(5);
                P_hip(6)];
    P_hip_RL = [P_hip(7);
                P_hip(8);
                P_hip(9)];
    P_hip_RR = [P_hip(10);
                P_hip(11);
                P_hip(12)];
             
    Kp = diag([300,300,300]);
    Kd = diag([10,10,10]);
 
    F_FL = [0;
            0;
            0];
    F_FR = [0;
            0;
            0];
    F_RL = [0;
            0;
            0];
    F_RR = [0;
            0;
            0];
        
    if t>=setup_time 
        t = t-setup_time;
        
        if t<4
            V_trunk_des = [0;0;0];
        elseif t<8
            V_trunk_des = [1;0;0];
        elseif t<12
            V_trunk_des = [0;0;0];
        elseif t<16
            V_trunk_des = [-1;0;0];
        elseif t<20
            V_trunk_des = [0;0;0];
        elseif t<24
            V_trunk_des = [0;0.5;0];
        elseif t<28
            V_trunk_des = [0;0;0];
        elseif t<32
            V_trunk_des = [0;-0.5;0];
        else
            V_trunk_des = [0;0;0];
        end
        
        t = rem(t, gait_cycle);
        T = gait_cycle/2;
        K_step = 0.03;
        t_onestep = rem(t, T);

        P_foot_FL_des = [P_hip_FL(1)+T/2*V_trunk(1)+K_step*(V_trunk(1)-V_trunk_des(1));
                         P_hip_FL(2)+T/2*V_trunk(2)+K_step*(V_trunk(2)-V_trunk_des(2));
                         step_height/(T/2)^2*t_onestep*(T-t_onestep)];
        P_foot_FR_des = [P_hip_FR(1)+T/2*V_trunk(1)+K_step*(V_trunk(1)-V_trunk_des(1));
                         P_hip_FR(2)+T/2*V_trunk(2)+K_step*(V_trunk(2)-V_trunk_des(2));
                         step_height/(T/2)^2*t_onestep*(T-t_onestep)];
        P_foot_RL_des = [P_hip_RL(1)+T/2*V_trunk(1)+K_step*(V_trunk(1)-V_trunk_des(1));
                         P_hip_RL(2)+T/2*V_trunk(2)+K_step*(V_trunk(2)-V_trunk_des(2));
                         step_height/(T/2)^2*t_onestep*(T-t_onestep)];
        P_foot_RR_des = [P_hip_RR(1)+T/2*V_trunk(1)+K_step*(V_trunk(1)-V_trunk_des(1));
                         P_hip_RR(2)+T/2*V_trunk(2)+K_step*(V_trunk(2)-V_trunk_des(2));
                         step_height/(T/2)^2*t_onestep*(T-t_onestep)];
        V_foot_des = [0;
                      0;
                      -2*step_height/(T/2)^2*t_onestep+step_height*T/(T/2)^2];
         
        % trotting gait
        if t<T % phase 1: FR,RL swing
            F_FR = Kp*(P_foot_FR_des+[0;0;ground_offset] - P_foot_FR)+ Kd*(V_foot_des - V_foot_FR);
            F_RL = Kp*(P_foot_RL_des+[0;0;ground_offset] - P_foot_RL)+ Kd*(V_foot_des - V_foot_RL);
        else   % phase 2: FL,RR swing
            F_FL = Kp*(P_foot_FL_des+[0;0;ground_offset] - P_foot_FL)+ Kd*(V_foot_des - V_foot_FL);
            F_RR = Kp*(P_foot_RR_des+[0;0;ground_offset] - P_foot_RR)+ Kd*(V_foot_des - V_foot_RR);
        end
    end

    F = [(R_trunk.')*F_FL;
         (R_trunk.')*F_FR;
         (R_trunk.')*F_RL;
         (R_trunk.')*F_RR];
end

