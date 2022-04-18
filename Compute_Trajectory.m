function trajectory = Compute_Trajectory(P_trunk, Euler_trunk, V_trunk_des, omega_trunk_des)
    global sim_params;
    g = sim_params.g;
    N = sim_params.Horizon;
    dt_MPC = sim_params.dt_MPC;
    
    rz_des = Euler_trunk(3);
    R_z = [cos(rz_des) -sin(rz_des) 0;
           sin(rz_des) cos(rz_des) 0;
           0 0 1];
    dEuler_trunk_des = R_z*omega_trunk_des; 
    
    trajectory = zeros(13,N);
    for k=1:N
        trajectory(1:3,k) = P_trunk+k*dt_MPC*V_trunk_des;
        Euler_trunk_k = Euler_trunk+k*dt_MPC*dEuler_trunk_des;
        if Euler_trunk_k>pi
            Euler_trunk_k = Euler_trunk_k - 2*pi;
        elseif Euler_trunk_k<-pi
            Euler_trunk_k = Euler_trunk_k - 2*pi;
        end
        trajectory(4:6,k) = Euler_trunk_k;
    end
    
    trajectory(7:9,:) = repmat(V_trunk_des,1,N);
    trajectory(10:12,:) = repmat(omega_trunk_des,1,N);
    trajectory(13,:) = g*ones(1,N);
end

