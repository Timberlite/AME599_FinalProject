function F = MPC_Control(t, P_trunk, Q_trunk, V_trunk, omega_trunk, P_foot, trajectory, gait, Q, Alpha)
    % system parameters
    global sim_params robot_params;
    m_trunk = robot_params.m_trunk;
    m_hip = robot_params.m_hip;
    m_thigh = robot_params.m_thigh;
    m_calf = robot_params.m_calf;
    m = m_trunk+4*(m_hip+m_thigh+m_calf);
    g = sim_params.g;
    I_b = robot_params.I_b;
       
    N = sim_params.Horizon;
    dt_MPC = sim_params.dt_MPC;
    k = floor(t/dt_MPC)+1; % horizon k within 1~N
    
    % force constraints
    f_max = 500;
    mu = 1.0;

    % current state
    Euler_trunk = (quat2eul(Q_trunk.','XYZ')).';
    R_trunk = quat2rotm(Q_trunk.');
%     I_w = R_trunk*I_b*R_trunk.';
    
    X = [P_trunk;
         Euler_trunk;
         V_trunk;
         omega_trunk;
         g];

    R = Alpha*eye(12);
    
    % Calculate foot pos
    P_footFL_w = [P_foot(1);P_foot(2);P_foot(3)];
    P_footFR_w = [P_foot(4);P_foot(5);P_foot(6)];
    P_footRL_w = [P_foot(7);P_foot(8);P_foot(9)];
    P_footRR_w = [P_foot(10);P_foot(11);P_foot(12)];

    r_footFL_b = (R_trunk.')*(P_footFL_w-P_trunk);
    r_footFR_b = (R_trunk.')*(P_footFR_w-P_trunk);
    r_footRL_b = (R_trunk.')*(P_footRL_w-P_trunk);
    r_footRR_b = (R_trunk.')*(P_footRR_w-P_trunk);

    
    % dynamic matrix
    rz_des = trajectory(6,N/2);
    A_dynamic = [0 0 0 0 0 0 1 0 0 0 0 0 0;
                 0 0 0 0 0 0 0 1 0 0 0 0 0;
                 0 0 0 0 0 0 0 0 1 0 0 0 0;
                 0 0 0 0 0 0 0 0 0 cos(rz_des)  sin(rz_des) 0 0;
                 0 0 0 0 0 0 0 0 0 -sin(rz_des) cos(rz_des) 0 0;
                 0 0 0 0 0 0 0 0 0 0 0 1 0;
                 0 0 0 0 0 0 0 0 0 0 0 0 0;
                 0 0 0 0 0 0 0 0 0 0 0 0 0;
                 0 0 0 0 0 0 0 0 0 0 0 0 -1;
                 0 0 0 0 0 0 0 0 0 0 0 0 0;
                 0 0 0 0 0 0 0 0 0 0 0 0 0;
                 0 0 0 0 0 0 0 0 0 0 0 0 0;
                 0 0 0 0 0 0 0 0 0 0 0 0 0];
    B_dynamic = [0 0 0 0 0 0 0 0 0 0 0 0;
                 0 0 0 0 0 0 0 0 0 0 0 0;
                 0 0 0 0 0 0 0 0 0 0 0 0;
                 0 0 0 0 0 0 0 0 0 0 0 0;
                 0 0 0 0 0 0 0 0 0 0 0 0;
                 0 0 0 0 0 0 0 0 0 0 0 0;
                 (1/m)*R_trunk, (1/m)*R_trunk, (1/m)*R_trunk, (1/m)*R_trunk;
                 I_b\skew(r_footFL_b), I_b\skew(r_footFR_b), I_b\skew(r_footRL_b), I_b\skew(r_footRR_b)
                 0 0 0 0 0 0 0 0 0 0 0 0;];
    A_bar = A_dynamic*dt_MPC+eye(13);
    B_bar = B_dynamic*dt_MPC;   

    % contact force constrains
    C_f = [0,0,1; 
           1,0,-mu;
           -1,0,-mu;
           0,1,-mu;
           0,-1,-mu];
    d_f = [f_max;
           0;
           0;
           0;
           0];

    C_f = kron(eye(4*N),C_f);
    d_f = repmat(d_f,4*N,1);
    
    % dynamic constraints   
    A_bar_map = diag(ones(1,N-1),-1);
    Ceq_dynamic = [eye(13*N)+kron(A_bar_map,-A_bar), kron(eye(N),-B_bar)];
    deq_dynamic = [A_bar*X;
                   zeros(13*(N-1),1)];
               
    % gaits constraints
    Ceq_gaits = gait;
    Ceq_gaits = circshift(Ceq_gaits,[-(k-1)*12 -(k-1)*12]);
    Ceq_gaits = [zeros(length(Ceq_gaits(:,1)),13*N),Ceq_gaits];
    deq_gaits = zeros(length(Ceq_gaits(:,1)),1);
    
    % formulate QP form
    H = blkdiag(kron(eye(N),Q),kron(eye(N),R));
    f = [kron(eye(N),-Q)*reshape(trajectory,[],1);
         repmat(zeros(12,1),N,1)];
    C = [zeros(length(C_f(:,1)),13*N), C_f];
    d = d_f;
    
    Ceq = [Ceq_dynamic;
           Ceq_gaits];
    deq = [deq_dynamic;
           deq_gaits];

    X_bar_optimal = quadprog(H,f,C,d, Ceq, deq);
    Fb_optimal = X_bar_optimal(13*N+1:13*N+12);
    F = -Fb_optimal;
end