function [P_foot_des, V_foot_des] = Foot_Placement(t, V_trunk, V_trunk_des, P_hip)
    global sim_params;
    gait_cycle = sim_params.gait_cycle;
    step_height = sim_params.default_step_height;
%     kth_cycle = floor(t/gait_cycle); % kth stair within 0~50
%     K_step = 0.005;
%     if t>17.2 && t<=20.8
%         climb_offset = 0.028;
%         climb_offset_R = -0.008;
%     else
%         climb_offset = 0;
%         climb_offset_R = 0;
%     end

    K_step = 0.03;

    t = rem(t, gait_cycle);
    
    
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
%     % Task2
%     if t<0.5*gait_cycle % phase 1: FR,RL swing
%         t_onestep = t;
%         T = 0.5*gait_cycle;
%         
%         P_foot_FR_des = [P_hip_FR(1)+(-T/2+t_onestep)*V_trunk(1)+K_step*(V_trunk(1)-V_trunk_des(1));
%                          P_hip_FR(2)+(-T/2+t_onestep)*V_trunk(2)+K_step*(V_trunk(2)-V_trunk_des(2));
%                          step_height/(T/2)^2*t_onestep*(T-t_onestep)];
%         P_foot_RL_des = [P_hip_RL(1)+(-T/2+t_onestep)*V_trunk(1)+K_step*(V_trunk(1)-V_trunk_des(1));
%                          P_hip_RL(2)+(-T/2+t_onestep)*V_trunk(2)+K_step*(V_trunk(2)-V_trunk_des(2));
%                          step_height/(T/2)^2*t_onestep*(T-t_onestep)];
%         V_foot_FR_des = [2*V_trunk(1);
%                          2*V_trunk(2);
%                         -2*step_height/(T/2)^2*t_onestep+step_height*T/(T/2)^2];
%         V_foot_RL_des = [2*V_trunk(1);
%                          2*V_trunk(2);
%                         -2*step_height/(T/2)^2*t_onestep+step_height*T/(T/2)^2];
%     else
%         t_onestep = t-0.5*gait_cycle;
%         T = 0.5*gait_cycle;
%         P_foot_RL_des = [P_hip_RL(1)-(-T/2+t_onestep)*V_trunk(1);
%                          P_hip_RL(2)-(-T/2+t_onestep)*V_trunk(1);
%                          0];
%         P_foot_FR_des = [P_hip_FR(1)-(-T/2+t_onestep)*V_trunk(1);
%                          P_hip_FR(2)-(-T/2+t_onestep)*V_trunk(1);
%                          0];
%         V_foot_RL_des = [0;
%                          0;
%                          0];
%         V_foot_FR_des = [0;
%                          0;
%                          0];
%     end
%     
%     if t>=0.5*gait_cycle % phase 2: FL,RR swing
%         t_onestep = t-0.5*gait_cycle;
%         T = 0.5*gait_cycle;
%         P_foot_FL_des = [P_hip_FL(1)+(-T/2+t_onestep)*V_trunk(1)+K_step*(V_trunk(1)-V_trunk_des(1));
%                          P_hip_FL(2)+(-T/2+t_onestep)*V_trunk(2)+K_step*(V_trunk(2)-V_trunk_des(2));
%                          step_height/(T/2)^2*t_onestep*(T-t_onestep)];
%         P_foot_RR_des = [P_hip_RR(1)+(-T/2+t_onestep)*V_trunk(1)+K_step*(V_trunk(1)-V_trunk_des(1));
%                          P_hip_RR(2)+(-T/2+t_onestep)*V_trunk(2)+K_step*(V_trunk(2)-V_trunk_des(2));
%                          step_height/(T/2)^2*t_onestep*(T-t_onestep)];
%         V_foot_FL_des = [2*V_trunk(1);
%                          2*V_trunk(2);
%                         -2*step_height/(T/2)^2*t_onestep+step_height*T/(T/2)^2];
%         V_foot_RR_des = [2*V_trunk(1);
%                          2*V_trunk(2);
%                         -2*step_height/(T/2)^2*t_onestep+step_height*T/(T/2)^2];
%     else
%         t_onestep = t;
%         T = 0.5*gait_cycle;
%         P_foot_FL_des = [P_hip_FL(1)-(-T/2+t_onestep)*V_trunk(1);
%                          P_hip_FL(2)-(-T/2+t_onestep)*V_trunk(2);
%                          0];
%         P_foot_RR_des = [P_hip_RR(1)-(-T/2+t_onestep)*V_trunk(1);
%                          P_hip_RR(2)-(-T/2+t_onestep)*V_trunk(2);
%                          0];
%         V_foot_FL_des = [0;
%                          0;
%                          0];
%         V_foot_RR_des = [0;
%                          0;
%                          0];
%     end
            
            
%     % Task4
%     if t<0.5*gait_cycle % phase 1: FR,RL swing
%         t_onestep = t;
%         T = 0.5*gait_cycle;
%         
%         t_b = t_onestep/T;
%         
%         P0x = (-T*0.5)*V_trunk(1);
%         P1x = (-T*1.5)*V_trunk(1);
%         P2x = (T*1.5)*V_trunk(1);
%         P3x = (T*0.5)*V_trunk(1);
%         Bx = (1-t_b)^3*P0x+3*(1-t_b)^2*t_b*P1x+3*(1-t_b)*t_b^2*P2x+t_b^3*P3x;
%         dBx = 3*(1-t_b)^2*(P1x-P0x)+6*(1-t_b)*t_b*(P2x-P1x)+3*t_b^2*(P3x-P2x);
%         
%         P0 = rough_terrain([-0.1+(kth_cycle-1)*0.2;0;0]);
%         P3 = rough_terrain([-0.1+(kth_cycle)*0.2;0;0]);
%         P1 = P0+sim_params.default_step_height;
%         P2 = P3+sim_params.default_step_height;
%         
%         P_foot_FR_des = [P_hip_FR(1)+Bx+0.01+climb_offset;
%                          P_hip_FR(2)+(-T/2+t_onestep)*V_trunk(2)+K_step*(V_trunk(2)-V_trunk_des(2));
%                          (1-t_b)^3*P0+3*(1-t_b)^2*t_b*P1+3*(1-t_b)*t_b^2*P2+t_b^3*P3];
%         V_foot_FR_des = [V_trunk(1)+dBx;
%                          2*V_trunk(2);
%                          3*(1-t_b)^2*(P1-P0)+6*(1-t_b)*t_b*(P2-P1)+3*t_b^2*(P3-P2)];
%         
%         P0 = rough_terrain([-0.1+(kth_cycle-3)*0.2;0;0]);
%         P3 = rough_terrain([-0.1+(kth_cycle-2)*0.2;0;0]);
%         P1 = P0+sim_params.default_step_height;
%         P2 = P3+sim_params.default_step_height;
%                      
%         P_foot_RL_des = [P_hip_RL(1)+Bx-0.01+climb_offset_R;
%                          P_hip_RL(2)+(-T/2+t_onestep)*V_trunk(2)+K_step*(V_trunk(2)-V_trunk_des(2));
%                          (1-t_b)^3*P0+3*(1-t_b)^2*t_b*P1+3*(1-t_b)*t_b^2*P2+t_b^3*P3];
%         
%         V_foot_RL_des = [V_trunk(1)+dBx;
%                          2*V_trunk(2);
%                          3*(1-t_b)^2*(P1-P0)+6*(1-t_b)*t_b*(P2-P1)+3*t_b^2*(P3-P2)];
%     else
%         t_onestep = t-0.5*gait_cycle;
%         T = 0.5*gait_cycle;
%         
%         P3 = rough_terrain([-0.1+(kth_cycle)*0.2;0;0]);
%         P_foot_FR_des = [P_hip_FR(1)-(-T/2+t_onestep)*V_trunk(1)+0.01+climb_offset;
%                          P_hip_FR(2)-(-T/2+t_onestep)*V_trunk(1);
%                          P3];
%         V_foot_FR_des = [0;
%                          0;
%                          0];
%                      
%         P3 = rough_terrain([-0.1+(kth_cycle-2)*0.2;0;0]);             
%         P_foot_RL_des = [P_hip_RL(1)-(-T/2+t_onestep)*V_trunk(1)-0.01+climb_offset_R;
%                          P_hip_RL(2)-(-T/2+t_onestep)*V_trunk(1);
%                          P3];
%         V_foot_RL_des = [0;
%                          0;
%                          0];
%         
%     end
%     
%     if t>=0.5*gait_cycle % phase 2: FL,RR swing
%         t_onestep = t-0.5*gait_cycle;
%         T = 0.5*gait_cycle;
%         
%         t_b = t_onestep/T;
%         
%         P0x = (-T*0.5)*V_trunk(1);
%         P1x = (-T*1.5)*V_trunk(1);
%         P2x = (T*1.5)*V_trunk(1);
%         P3x = (T*0.5)*V_trunk(1);
%         Bx = (1-t_b)^3*P0x+3*(1-t_b)^2*t_b*P1x+3*(1-t_b)*t_b^2*P2x+t_b^3*P3x;
%         dBx = 3*(1-t_b)^2*(P1x-P0x)+6*(1-t_b)*t_b*(P2x-P1x)+3*t_b^2*(P3x-P2x);
%         
%         P0 = rough_terrain([-0.1+(kth_cycle-1)*0.2;0;0]);
%         P3 = rough_terrain([-0.1+(kth_cycle)*0.2;0;0]);
%         P1 = P0+sim_params.default_step_height;
%         P2 = P3+sim_params.default_step_height;
%         
%         P_foot_FL_des = [P_hip_FL(1)+Bx+0.01+climb_offset;
%                          P_hip_FL(2)+(-T/2+t_onestep)*V_trunk(2)+K_step*(V_trunk(2)-V_trunk_des(2));
%                          (1-t_b)^3*P0+3*(1-t_b)^2*t_b*P1+3*(1-t_b)*t_b^2*P2+t_b^3*P3];
%         V_foot_FL_des = [V_trunk(1)+dBx;
%                          2*V_trunk(2);
%                          3*(1-t_b)^2*(P1-P0)+6*(1-t_b)*t_b*(P2-P1)+3*t_b^2*(P3-P2)];
%         
%         P0 = rough_terrain([-0.1+(kth_cycle-3)*0.2;0;0]);
%         P3 = rough_terrain([-0.1+(kth_cycle-2)*0.2;0;0]);
%         P1 = P0+sim_params.default_step_height;
%         P2 = P3+sim_params.default_step_height;
%                      
%         P_foot_RR_des = [P_hip_RR(1)+Bx-0.01+climb_offset_R;
%                          P_hip_RR(2)+(-T/2+t_onestep)*V_trunk(2)+K_step*(V_trunk(2)-V_trunk_des(2));
%                          (1-t_b)^3*P0+3*(1-t_b)^2*t_b*P1+3*(1-t_b)*t_b^2*P2+t_b^3*P3];
%         V_foot_RR_des = [V_trunk(1)+dBx;
%                          2*V_trunk(2);
%                          3*(1-t_b)^2*(P1-P0)+6*(1-t_b)*t_b*(P2-P1)+3*t_b^2*(P3-P2)];
%     else
%         t_onestep = t;
%         T = 0.5*gait_cycle;
%         
%         P3 = rough_terrain([-0.1+(kth_cycle)*0.2;0;0]);
%         P_foot_FL_des = [P_hip_FL(1)-(-T/2+t_onestep)*V_trunk(1)+0.01+climb_offset;
%                          P_hip_FL(2)-(-T/2+t_onestep)*V_trunk(2);
%                          P3];
%         V_foot_FL_des = [0;
%                          0;
%                          0];
%                      
%         P3 = rough_terrain([-0.1+(kth_cycle-2)*0.2;0;0]);             
%         P_foot_RR_des = [P_hip_RR(1)-(-T/2+t_onestep)*V_trunk(1)-0.01+climb_offset_R;
%                          P_hip_RR(2)-(-T/2+t_onestep)*V_trunk(2);
%                          P3];
%         V_foot_RR_des = [0;
%                          0;
%                          0];
%     end
            
%     % Task3
%     if t<0.55*gait_cycle % phase 1: FL,FR swing
%         t_onestep = t;
%         T = 0.55*gait_cycle;
%         P_foot_FL_des = [P_hip_FL(1)+(9/11)*(-T/2+t_onestep)*V_trunk(1)+K_step*(V_trunk(1)-V_trunk_des(1));
%                          P_hip_FL(2)+(9/11)*(-T/2+t_onestep)*V_trunk(2)+K_step*(V_trunk(2)-V_trunk_des(2));
%                          step_height/(T/2)^2*t_onestep*(T-t_onestep)];
%         P_foot_FR_des = [P_hip_FR(1)+(9/11)*(-T/2+t_onestep)*V_trunk(1)+K_step*(V_trunk(1)-V_trunk_des(1));
%                          P_hip_FR(2)+(9/11)*(-T/2+t_onestep)*V_trunk(2)+K_step*(V_trunk(2)-V_trunk_des(2));
%                          step_height/(T/2)^2*t_onestep*(T-t_onestep)];
%         V_foot_FL_des = [1.818*V_trunk(1);
%                          1.818*V_trunk(2);
%                         -2*step_height/(T/2)^2*t_onestep+step_height*T/(T/2)^2];
%         V_foot_FR_des = [1.818*V_trunk(1);
%                          1.818*V_trunk(2);
%                         -2*step_height/(T/2)^2*t_onestep+step_height*T/(T/2)^2];
%     else
%         t_onestep = t-0.55*gait_cycle;
%         T = 0.45*gait_cycle;
%         P_foot_FL_des = [P_hip_FL(1)-(-T/2+t_onestep)*V_trunk(1);
%                          P_hip_FL(2)-(-T/2+t_onestep)*V_trunk(1);
%                          0];
%         P_foot_FR_des = [P_hip_FR(1)-(-T/2+t_onestep)*V_trunk(1);
%                          P_hip_FR(2)-(-T/2+t_onestep)*V_trunk(1);
%                          0];
%         V_foot_FL_des = [0;
%                          0;
%                          0];
%         V_foot_FR_des = [0;
%                          0;
%                          0];
%     end
%     
%     if t>=0.45*gait_cycle % phase 2: RL,RR swing
%         t_onestep = t-0.45*gait_cycle;
%         T = 0.55*gait_cycle;
%         P_foot_RL_des = [P_hip_RL(1)+(9/11)*(-T/2+t_onestep)*V_trunk(1)+K_step*(V_trunk(1)-V_trunk_des(1));
%                          P_hip_RL(2)+(9/11)*(-T/2+t_onestep)*V_trunk(2)+K_step*(V_trunk(2)-V_trunk_des(2));
%                          step_height/(T/2)^2*t_onestep*(T-t_onestep)];
%         P_foot_RR_des = [P_hip_RR(1)+(9/11)*(-T/2+t_onestep)*V_trunk(1)+K_step*(V_trunk(1)-V_trunk_des(1));
%                          P_hip_RR(2)+(9/11)*(-T/2+t_onestep)*V_trunk(2)+K_step*(V_trunk(2)-V_trunk_des(2));
%                          step_height/(T/2)^2*t_onestep*(T-t_onestep)];
%         V_foot_RL_des = [1.818*V_trunk(1);
%                          1.818*V_trunk(2);
%                         -2*step_height/(T/2)^2*t_onestep+step_height*T/(T/2)^2];
%         V_foot_RR_des = [1.818*V_trunk(1);
%                          1.818*V_trunk(2);
%                         -2*step_height/(T/2)^2*t_onestep+step_height*T/(T/2)^2];
%     else
%         t_onestep = t;
%         T = 0.45*gait_cycle;
%         P_foot_RL_des = [P_hip_RL(1)-(-T/2+t_onestep)*V_trunk(1);
%                          P_hip_RL(2)-(-T/2+t_onestep)*V_trunk(2);
%                          0];
%         P_foot_RR_des = [P_hip_RR(1)-(-T/2+t_onestep)*V_trunk(1);
%                          P_hip_RR(2)-(-T/2+t_onestep)*V_trunk(2);
%                          0];
%         V_foot_RL_des = [0;
%                          0;
%                          0];
%         V_foot_RR_des = [0;
%                          0;
%                          0];
%     end


    % Task5
    if t<0.55*gait_cycle % phase 1: FL,FR swing
        t_onestep = t;
        T = 0.55*gait_cycle;
        P_foot_FL_des = [P_hip_FL(1)+(9/11)*(-T/2+t_onestep)*V_trunk(1)+K_step*(V_trunk(1)-V_trunk_des(1));
                         P_hip_FL(2)+(9/11)*(-T/2+t_onestep)*V_trunk(2)+K_step*(V_trunk(2)-V_trunk_des(2));
                         step_height/(T/2)^2*t_onestep*(T-t_onestep)];
        P_foot_FR_des = [P_hip_FR(1)+(9/11)*(-T/2+t_onestep)*V_trunk(1)+K_step*(V_trunk(1)-V_trunk_des(1));
                         P_hip_FR(2)+(9/11)*(-T/2+t_onestep)*V_trunk(2)+K_step*(V_trunk(2)-V_trunk_des(2));
                         step_height/(T/2)^2*t_onestep*(T-t_onestep)];
        V_foot_FL_des = [1.818*V_trunk(1);
                         1.818*V_trunk(2);
                        -2*step_height/(T/2)^2*t_onestep+step_height*T/(T/2)^2];
        V_foot_FR_des = [1.818*V_trunk(1);
                         1.818*V_trunk(2);
                        -2*step_height/(T/2)^2*t_onestep+step_height*T/(T/2)^2];
    else
        t_onestep = t-0.55*gait_cycle;
        T = 0.45*gait_cycle;
        P_foot_FL_des = [P_hip_FL(1)-(-T/2+t_onestep)*V_trunk(1);
                         P_hip_FL(2)-(-T/2+t_onestep)*V_trunk(1);
                         0];
        P_foot_FR_des = [P_hip_FR(1)-(-T/2+t_onestep)*V_trunk(1);
                         P_hip_FR(2)-(-T/2+t_onestep)*V_trunk(1);
                         0];
        V_foot_FL_des = [0;
                         0;
                         0];
        V_foot_FR_des = [0;
                         0;
                         0];
    end
    
    if t>=0.45*gait_cycle % phase 2: RL,RR swing
        t_onestep = t-0.45*gait_cycle;
        T = 0.55*gait_cycle;
        P_foot_RL_des = [P_hip_RL(1)+(9/11)*(-T/2+t_onestep)*V_trunk(1)+K_step*(V_trunk(1)-V_trunk_des(1));
                         P_hip_RL(2)+(9/11)*(-T/2+t_onestep)*V_trunk(2)+K_step*(V_trunk(2)-V_trunk_des(2));
                         step_height/(T/2)^2*t_onestep*(T-t_onestep)];
        P_foot_RR_des = [P_hip_RR(1)+(9/11)*(-T/2+t_onestep)*V_trunk(1)+K_step*(V_trunk(1)-V_trunk_des(1));
                         P_hip_RR(2)+(9/11)*(-T/2+t_onestep)*V_trunk(2)+K_step*(V_trunk(2)-V_trunk_des(2));
                         step_height/(T/2)^2*t_onestep*(T-t_onestep)];
        V_foot_RL_des = [1.818*V_trunk(1);
                         1.818*V_trunk(2);
                        -2*step_height/(T/2)^2*t_onestep+step_height*T/(T/2)^2];
        V_foot_RR_des = [1.818*V_trunk(1);
                         1.818*V_trunk(2);
                        -2*step_height/(T/2)^2*t_onestep+step_height*T/(T/2)^2];
    else
        t_onestep = t;
        T = 0.45*gait_cycle;
        P_foot_RL_des = [P_hip_RL(1)-(-T/2+t_onestep)*V_trunk(1);
                         P_hip_RL(2)-(-T/2+t_onestep)*V_trunk(2);
                         0];
        P_foot_RR_des = [P_hip_RR(1)-(-T/2+t_onestep)*V_trunk(1);
                         P_hip_RR(2)-(-T/2+t_onestep)*V_trunk(2);
                         0];
        V_foot_RL_des = [0;
                         0;
                         0];
        V_foot_RR_des = [0;
                         0;
                         0];
    end


    P_foot_des = [P_foot_FL_des;
                  P_foot_FR_des;
                  P_foot_RL_des;
                  P_foot_RR_des];
    V_foot_des = [V_foot_FL_des;
                  V_foot_FR_des;
                  V_foot_RL_des;
                  V_foot_RR_des];
end

