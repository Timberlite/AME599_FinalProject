function F  = Cartesian_Control(t, P_foot, V_foot)
    g = 9.81;
    setup_time = 0.0;      % (s) time for initial settling
    gait_cycle = 0.4;      % (s) time for one gait cycle
    ground_offset = 0.020; % (m) P_foot height at contact
    step_height = 0.10;

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
             
    Kp = diag([150,150,150]);
    Kd = diag([5,5,10]);
 
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
        
%     if t>=setup_time 
%         t = rem(t-setup_time, gait_cycle);
%         
%         % trotting gait
%         T = gait_cycle/2;
%         t_onestep = rem(t, T);
%         P_foot_FL_des = [0.217;
%                          0.130;
%                          step_height/(T/2)^2*t_onestep*(T-t_onestep)];
%         P_foot_FR_des = [0.217;
%                          -0.130;
%                          step_height/(T/2)^2*t_onestep*(T-t_onestep)];
%         P_foot_RL_des = [-0.144;
%                          0.130;
%                          step_height/(T/2)^2*t_onestep*(T-t_onestep)];
%         P_foot_RR_des = [-0.144;
%                          -0.130;
%                          step_height/(T/2)^2*t_onestep*(T-t_onestep)];
%         V_foot_des = [0;
%                       0;
%                       -2*step_height/(T/2)^2*t_onestep+step_height*T/(T/2)^2];
%         if t<T % phase 1: FR,RL swing
%             F_FR = Kp*(P_foot_FR_des+[0;0;ground_offset] - P_foot_FR)+ Kd*(V_foot_des - V_foot_FR)+[0;0;g];
%             F_RL = Kp*(P_foot_RL_des+[0;0;ground_offset] - P_foot_RL)+ Kd*(V_foot_des - V_foot_RL)+[0;0;g];
%         else   % phase 2: FL,RR swing
%             F_FL = Kp*(P_foot_FL_des+[0;0;ground_offset] - P_foot_FL)+ Kd*(V_foot_des - V_foot_FL)+[0;0;g];
%             F_RR = Kp*(P_foot_RR_des+[0;0;ground_offset] - P_foot_RR)+ Kd*(V_foot_des - V_foot_RR)+[0;0;g];
%         end
%     end

    F = [F_FL;
         F_FR;
         F_RL;
         F_RR];
end

