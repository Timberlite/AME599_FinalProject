function F  = Cartesian_Control(t, Q_trunk, V_trunk, P_foot, V_foot, P_hip)
    global sim_params;
    setup_time = sim_params.setup_time;      % (s) time for initial settling
    gait_cycle = sim_params.gait_cycle;
    ground_offset = 0.020; % (m) P_foot height at contact
   
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
        
%         % Task2
%         if t<4
%             V_trunk_des = [0;0;0];
%         elseif t<8
%             V_trunk_des = [1;0;0];
%         elseif t<12
%             V_trunk_des = [0;0;0];
%         elseif t<16
%             V_trunk_des = [-1;0;0];
%         elseif t<20
%             V_trunk_des = [0;0;0];
%         elseif t<24
%             V_trunk_des = [0;0.5;0];
%         elseif t<28
%             V_trunk_des = [0;0;0];
%         elseif t<32
%             V_trunk_des = [0;-0.5;0];
%         else
%             V_trunk_des = [0;0;0];
%         end

        % Task3
        if t<1.6
            V_trunk_des = [1+1.75*t;0;0];
        else
            V_trunk_des = [3.8;0;0];
        end
        
        
        [P_foot_des, V_foot_des] = Foot_Placement(t, V_trunk, V_trunk_des, P_hip);
        P_foot_FL_des = P_foot_des(1:3);
        P_foot_FR_des = P_foot_des(4:6);
        P_foot_RL_des = P_foot_des(7:9);
        P_foot_RR_des = P_foot_des(10:12);
        V_foot_FL_des = V_foot_des(1:3);
        V_foot_FR_des = V_foot_des(4:6);
        V_foot_RL_des = V_foot_des(7:9);
        V_foot_RR_des = V_foot_des(10:12);

%         % trotting gait
%         t = rem(t, gait_cycle);
%         if t<0.5*gait_cycle % phase 1: FR,RL swing
%             F_FR = Kp*(P_foot_FR_des+[0;0;ground_offset] - P_foot_FR)+ Kd*(V_foot_FR_des - V_foot_FR);
%             F_RL = Kp*(P_foot_RL_des+[0;0;ground_offset] - P_foot_RL)+ Kd*(V_foot_RL_des - V_foot_RL);
%         else   % phase 2: FL,RR swing
%             F_FL = Kp*(P_foot_FL_des+[0;0;ground_offset] - P_foot_FL)+ Kd*(V_foot_FL_des - V_foot_FL);
%             F_RR = Kp*(P_foot_RR_des+[0;0;ground_offset] - P_foot_RR)+ Kd*(V_foot_RR_des - V_foot_RR);
%         end
        
        % bounding gait
        t = rem(t, gait_cycle);
        if t<0.5*gait_cycle % phase 1: FL,FR swing
            F_FL = Kp*(P_foot_FL_des+[0;0;ground_offset] - P_foot_FL)+ Kd*(V_foot_FL_des - V_foot_FL);
            F_FR = Kp*(P_foot_FR_des+[0;0;ground_offset] - P_foot_FR)+ Kd*(V_foot_FR_des - V_foot_FR);
        end
        if t>=0.5*gait_cycle  % phase 2: RL,RR swing
            F_RL = Kp*(P_foot_RL_des+[0;0;ground_offset] - P_foot_RL)+ Kd*(V_foot_RL_des - V_foot_RL);
            F_RR = Kp*(P_foot_RR_des+[0;0;ground_offset] - P_foot_RR)+ Kd*(V_foot_RR_des - V_foot_RR);
        end
    end

    F = [(R_trunk.')*F_FL;
         (R_trunk.')*F_FR;
         (R_trunk.')*F_RL;
         (R_trunk.')*F_RR];
end

