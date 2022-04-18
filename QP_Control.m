function F = QP_Control(P_trunk, Q_trunk, V_trunk, omega_trunk, P_foot)
    % system parameters
    global sim_params robot_params;
    m_trunk = robot_params.m_trunk;
    m_hip = robot_params.m_hip;
    m_thigh = robot_params.m_thigh;
    m_calf = robot_params.m_calf;
    m = m_trunk+4*(m_hip+m_thigh+m_calf);
    g = sim_params.g;
    I_b = robot_params.I_b;
    default_trunk_height = sim_params.default_trunk_height;
       
    % force constraints
    f_min = 10;
    f_max = 500;
    mu = 0.5;

    % desire state
    P_trunk_des = [(P_foot(1)+P_foot(4)+P_foot(7)+P_foot(10))/4;(P_foot(2)+P_foot(5)+P_foot(8)+P_foot(11))/4;default_trunk_height];
    Euler_trunk_des = [0;0;0];
    V_trunk_des = [0;0;0];
    omega_trunk_des = [0;0;0];

    % Calculate desire force b_des
    Euler_trunk = (quat2eul(Q_trunk.','XYZ')).';
    R_trunk = quat2rotm(Q_trunk.');
    I_w = R_trunk*I_b*R_trunk.';

    Kp_p = diag([80 80 80]); 
    Kd_p = diag([20 20 20]);
    Kp_w = diag([150 150 150]);
    Kd_w = diag([50 50 50]);

    dd_P_trunk_des = Kp_p*(P_trunk_des-P_trunk)+Kd_p*(V_trunk_des-V_trunk)+[0;0;g];
    dd_R_trunk_des = Kp_w*(Euler_trunk_des-Euler_trunk)+Kd_w*(omega_trunk_des-(R_trunk)*omega_trunk);

    b_des = [m*(dd_P_trunk_des);
             I_w*dd_R_trunk_des];

    % Calculate foot pos
    P_footFL_w = [P_foot(1);P_foot(2);P_foot(3)];
    P_footFR_w = [P_foot(4);P_foot(5);P_foot(6)];
    P_footRL_w = [P_foot(7);P_foot(8);P_foot(9)];
    P_footRR_w = [P_foot(10);P_foot(11);P_foot(12)];

    r_footFL_w = P_footFL_w-P_trunk;
    r_footFR_w = P_footFR_w-P_trunk;
    r_footRL_w = P_footRL_w-P_trunk;
    r_footRR_w = P_footRR_w-P_trunk;

    A_f = [eye(3), eye(3), eye(3), eye(3);
           skew(r_footFL_w), skew(r_footFR_w), skew(r_footRL_w), skew(r_footRR_w)];

    alpha = 0.001*eye(12);
    S = eye(6);
    H = A_f'*S*A_f+alpha; 
    f = -(A_f.')*S*b_des;

    C_f = [0,0,1;
           0,0,-1;  
           1,0,-mu;
           -1,0,-mu;
           0,1,-mu;
           0,-1,-mu];
    d_f = [f_max;
           -f_min;
           0;
           0;
           0;
           0];
    C = blkdiag(C_f,C_f,C_f,C_f);
    d = repmat(d_f,4,1);

    F_ground = quadprog(H,f,C,d);
    F = -1*kron(eye(4),R_trunk.')*F_ground;
end