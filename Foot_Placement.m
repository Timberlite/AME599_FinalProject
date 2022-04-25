function [P_foot_des, V_foot_des] = Foot_Placement(t, V_trunk, V_trunk_des, P_hip)
    global sim_params;
    gait_cycle = sim_params.gait_cycle;
    step_height = sim_params.default_step_height;
    t = rem(t, gait_cycle);
    K_step = 0.0;
    
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

