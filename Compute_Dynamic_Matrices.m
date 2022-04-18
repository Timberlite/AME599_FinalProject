function [A_bar, B_bar ] = Compute_Dynamic_Matrices(k ,trajectory, R_trunk)
    global sim_params robot_params;
    m_trunk = robot_params.m_trunk;
    m_hip = robot_params.m_hip;
    m_thigh = robot_params.m_thigh;
    m_calf = robot_params.m_calf;
    m = m_trunk+4*(m_hip+m_thigh+m_calf);
    I_b = robot_params.I_b;
    default_trunk_height = sim_params.default_trunk_height;
    ground_offset = 0.020; % (m) P_foot height at contact
    
    N = sim_params.Horizon;
    dt_MPC = sim_params.dt_MPC;
    gait_cycle = sim_params.gait_cycle;

    A_bar = cell(1,N);
    B_bar = cell(1,N);
    
    r_hipFL_b = [0.1805;0.128;0];
    r_hipFR_b = [0.1805;-0.128;0];
    r_hipRL_b = [-0.1805;0.128;0];
    r_hipRR_b = [-0.1805;-0.128;0];
    
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
        
        
        T = gait_cycle/2;
        V_trunk_des = trajectory(7:9,i);
        
        foot_offset = i+k-1;
        if foot_offset>10
            foot_offset = foot_offset-10;
        end
            
        if foot_offset<6 
            r_footFL_b = r_hipFL_b+T/2*V_trunk_des-(foot_offset-1)*T/4*V_trunk_des+[0;0;ground_offset-default_trunk_height];
            r_footFR_b = r_hipFR_b+T/2*V_trunk_des-(5-foot_offset)*T/4*V_trunk_des+[0;0;ground_offset-default_trunk_height];
            r_footRL_b = r_hipRL_b+T/2*V_trunk_des-(foot_offset-1)*T/4*V_trunk_des+[0;0;ground_offset-default_trunk_height];
            r_footRR_b = r_hipRR_b+T/2*V_trunk_des-(5-foot_offset)*T/4*V_trunk_des+[0;0;ground_offset-default_trunk_height];
        else
            r_footFL_b = r_hipFL_b+T/2*V_trunk_des-(10-foot_offset)*T/4*V_trunk_des+[0;0;ground_offset-default_trunk_height];
            r_footFR_b = r_hipFR_b+T/2*V_trunk_des-(foot_offset-6)*T/4*V_trunk_des+[0;0;ground_offset-default_trunk_height];
            r_footRL_b = r_hipRL_b+T/2*V_trunk_des-(10-foot_offset)*T/4*V_trunk_des+[0;0;ground_offset-default_trunk_height];
            r_footRR_b = r_hipRR_b+T/2*V_trunk_des-(foot_offset-6)*T/4*V_trunk_des+[0;0;ground_offset-default_trunk_height];
        end
        
%         R_trunk = eul2rotm([trajectory(6,i), trajectory(5,i), trajectory(4,i)]);
        B_dynamic = [zeros(3), zeros(3), zeros(3), zeros(3);
                     zeros(3), zeros(3), zeros(3), zeros(3);
                     (1/m)*R_trunk, (1/m)*R_trunk, (1/m)*R_trunk, (1/m)*R_trunk;
                     I_b\skew(r_footFL_b), I_b\skew(r_footFR_b), I_b\skew(r_footRL_b), I_b\skew(r_footRR_b);
                     zeros(1,12)];
        B_bar{i} = B_dynamic*dt_MPC;
    end
end

