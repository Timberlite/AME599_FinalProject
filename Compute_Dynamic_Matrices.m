function [A_bar, B_bar ] = Compute_Dynamic_Matrices(t ,trajectory, R_trunk)
    global sim_params robot_params;
    m_trunk = robot_params.m_trunk;
    m_hip = robot_params.m_hip;
    m_thigh = robot_params.m_thigh;
    m_calf = robot_params.m_calf;
    m = m_trunk+4*(m_hip+m_thigh+m_calf);
    I_b = robot_params.I_b;
    
    N = sim_params.Horizon;
    dt_MPC = sim_params.dt_MPC;

    A_bar = cell(1,N);
    B_bar = cell(1,N);
    
    r_hipFL_b = [0.18;0.13;0];
    r_hipFR_b = [0.18;-0.13;0];
    r_hipRL_b = [-0.18;0.13;0];
    r_hipRR_b = [-0.18;-0.13;0];
    
    for i=1:N
        rz_des = trajectory(6,i);
        R_z = [cos(rz_des) -sin(rz_des) 0;
               sin(rz_des) cos(rz_des) 0;
               0 0 1];
        A_dynamic = [zeros(3), zeros(3), eye(3), zeros(3), zeros(3,1);
                     zeros(3), zeros(3), zeros(3), R_z, zeros(3,1);
                     zeros(3), zeros(3), zeros(3), zeros(3), [0;0;-1];
                     zeros(3), zeros(3), zeros(3), zeros(3), zeros(3,1);
                     zeros(1,13)];
        A_bar{i} = A_dynamic*dt_MPC+eye(13);
        
        
        P_trunk_des = trajectory(1:3,i);
        Euler_trunk_des = trajectory(4:6,i);
        R_trunk_des = eul2rotm(Euler_trunk_des.','XYZ');
        V_trunk_des = trajectory(7:9,i);
        P_hip_FL_des = P_trunk_des+R_trunk_des*r_hipFL_b;
        P_hip_FR_des = P_trunk_des+R_trunk_des*r_hipFR_b;
        P_hip_RL_des = P_trunk_des+R_trunk_des*r_hipRL_b;
        P_hip_RR_des = P_trunk_des+R_trunk_des*r_hipRR_b;
        P_hip_des = [P_hip_FL_des;
                     P_hip_FR_des;
                     P_hip_RL_des;
                     P_hip_RR_des];

        [P_foot_des, V_foot_des] = Foot_Placement(t+i*dt_MPC, V_trunk_des, V_trunk_des, P_hip_des);
        P_foot_FL_des = P_foot_des(1:3);
        P_foot_FR_des = P_foot_des(4:6);
        P_foot_RL_des = P_foot_des(7:9);
        P_foot_RR_des = P_foot_des(10:12);
        
        r_footFL_b = (R_trunk_des.')*(P_foot_FL_des-P_trunk_des);
        r_footFR_b = (R_trunk_des.')*(P_foot_FR_des-P_trunk_des);
        r_footRL_b = (R_trunk_des.')*(P_foot_RL_des-P_trunk_des);
        r_footRR_b = (R_trunk_des.')*(P_foot_RR_des-P_trunk_des);
        
        B_dynamic = [zeros(3), zeros(3), zeros(3), zeros(3);
                     zeros(3), zeros(3), zeros(3), zeros(3);
                     (1/m)*R_trunk, (1/m)*R_trunk, (1/m)*R_trunk, (1/m)*R_trunk;
                     I_b\skew(r_footFL_b), I_b\skew(r_footFR_b), I_b\skew(r_footRL_b), I_b\skew(r_footRR_b);
                     zeros(1,12)];
        B_bar{i} = B_dynamic*dt_MPC;
    end
end

