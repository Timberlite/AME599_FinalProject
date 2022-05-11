function F  = Cartesian_Control(t, Q_trunk, V_trunk, P_foot, V_foot, P_hip)
    global sim_params;
    setup_time = sim_params.setup_time;      % (s) time for initial settling
    gait_cycle = sim_params.gait_cycle;
    ground_offset = 0.001; % (m) P_foot height at contact
   
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
    Kd = diag([11,11,11]);
 
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

%         % Task3
%         if t<1.6
%             V_trunk_des = [1+1.75*t;0;0];
%         else
%             V_trunk_des = [3.8;0;0];
%         end

%         % Task4
%         V_trunk_des = [0.5;0;0];

        % Task5
        if t<0.4
            V_trunk_des = [0.64+t*2.4;0;0];
        elseif t<0.9
            V_trunk_des = [1.6;0;0];
        elseif t<1.09
            V_trunk_des = [1.6;0;1.6];
        elseif t<4.08
            V_trunk_des = [1.6;0;0];
        elseif t<4.5
            V_trunk_des = [1.7;0;0];
        elseif t<4.7
            V_trunk_des = [1.7;0;1.4];
        else
            V_trunk_des = [1.7;0;0.0];
        end
        
        % rear leg placement adjust
        if t>0.8&&t<0.9
            x_offset_R = 0.03;
        elseif t>1.38&&t<1.5
            x_offset_R = 0.015;
        elseif t>4.38&&t<4.5
            x_offset_R = 0.01; 
        elseif t>4.69&&t<4.8
            x_offset_R = 0.034; 
        else
            x_offset_R = 0;
        end
        % front leg placement adjust
        if t>0.9&&t<1.09
            x_offset_F = 0.07;
        elseif t>4.55&&t<4.68
            x_offset_F = 0.04;
        else
            x_offset_F = 0;
        end
        
        % ground adjust
        if t>0.9&&t<1.55
            ground_offset_F = 0.201;
        elseif t>4.55&&t<4.88
            ground_offset_F = 0.301;
        else 
            ground_offset_F = 0;
        end
        
        if t>0.9&&t<1.75
            ground_offset_R = 0.201;
        elseif t>4.69&&t<5.06
            ground_offset_R = 0.301;
        else 
            ground_offset_R = 0;
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
%         if t<=0.5*gait_cycle % phase 1: FR,RL swing
%             F_FR = Kp*(P_foot_FR_des+[0;0;ground_offset] - P_foot_FR)+ Kd*(V_foot_FR_des - V_foot_FR);
%             F_RL = Kp*(P_foot_RL_des+[0;0;ground_offset] - P_foot_RL)+ Kd*(V_foot_RL_des - V_foot_RL);
%         end
%         if t>=0.5*gait_cycle  % phase 2: FL,RR swing
%             F_FL = Kp*(P_foot_FL_des+[0;0;ground_offset] - P_foot_FL)+ Kd*(V_foot_FL_des - V_foot_FL);
%             F_RR = Kp*(P_foot_RR_des+[0;0;ground_offset] - P_foot_RR)+ Kd*(V_foot_RR_des - V_foot_RR);
%         end
        
        % bounding gait
        t = rem(t, gait_cycle);
        if t<0.45*gait_cycle % phase 1: FL,FR swing
            F_FL = Kp*(P_foot_FL_des+[x_offset_F;0;ground_offset_F] - P_foot_FL)+ Kd*(V_foot_FL_des - V_foot_FL);
            F_FR = Kp*(P_foot_FR_des+[x_offset_F;0;ground_offset_F] - P_foot_FR)+ Kd*(V_foot_FR_des - V_foot_FR);
        end
        if t>=0.55*gait_cycle  % phase 2: RL,RR swing
            F_RL = Kp*(P_foot_RL_des+[x_offset_R;0;ground_offset_R] - P_foot_RL)+ Kd*(V_foot_RL_des - V_foot_RL);
            F_RR = Kp*(P_foot_RR_des+[x_offset_R;0;ground_offset_R] - P_foot_RR)+ Kd*(V_foot_RR_des - V_foot_RR);
        end
    end

    F = [(R_trunk.')*F_FL;
         (R_trunk.')*F_FR;
         (R_trunk.')*F_RL;
         (R_trunk.')*F_RR];
end

