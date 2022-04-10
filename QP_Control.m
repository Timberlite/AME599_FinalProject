function F = QP_Control(P_trunk, Q_trunk, V_trunk, omega_trunk, P_foot)

    % system parameters
    m_trunk = 6.0;
    m_hip = 0.696;
    m_thigh = 1.013;
    m_calf = 0.166;
    m = m_trunk+4*(m_hip+m_thigh+m_calf);
    g = 9.81;
    I_b = [0.0158533, 0, 0;
           0 0.0377999 0;
           0 0 0.0456542];
       
    % force constraints
    f_min = 10;
    f_max = 500;
    mu = 0.5;

    % desire state
    P_trunk_des = [0;0;0.3];
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
    dd_R_trunk_des = Kp_w*(Euler_trunk_des-Euler_trunk)+Kd_w*(omega_trunk_des-omega_trunk);

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

    F = quadprog(H,f,C,d);

end