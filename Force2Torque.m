function Torque = Force2Torque(F, q)

    l_hip = 0.0838; % hip length
    l_thigh = 0.2; % thigh length
    l_calf = 0.2;  % calf length
    
    L1 = l_hip;
    L2 = l_thigh;
    L3 = l_calf;
    
    F_FL = F(1:3);
    F_FR = F(4:6);
    F_RL = F(7:9);
    F_RR = F(10:12);

    %FL
    q1 = q(1);
    q2 = q(2);
    q3 = q(3);
    J_FL = [                                                                    0,            L3*sin(q2)*sin(q3) - cos(q2)*(L2 + L3*cos(q3)),            L3*sin(q2)*sin(q3) - L3*cos(q2)*cos(q3);
            cos(q1)*(cos(q2)*(L2 + L3*cos(q3)) - L3*sin(q2)*sin(q3)) - L1*sin(q1), -sin(q1)*(sin(q2)*(L2 + L3*cos(q3)) + L3*cos(q2)*sin(q3)), -sin(q1)*(L3*cos(q2)*sin(q3) + L3*cos(q3)*sin(q2));
            sin(q1)*(cos(q2)*(L2 + L3*cos(q3)) - L3*sin(q2)*sin(q3)) + L1*cos(q1),  cos(q1)*(sin(q2)*(L2 + L3*cos(q3)) + L3*cos(q2)*sin(q3)),  cos(q1)*(L3*cos(q2)*sin(q3) + L3*cos(q3)*sin(q2))];
    Torque_FL = (J_FL.')*F_FL;
    
    %FR 
    q1 = q(4);
    q2 = q(5);
    q3 = q(6);
    J_FR = [                                                                    0,            L3*sin(q2)*sin(q3) - cos(q2)*(L2 + L3*cos(q3)),            L3*sin(q2)*sin(q3) - L3*cos(q2)*cos(q3);
            cos(q1)*(cos(q2)*(L2 + L3*cos(q3)) - L3*sin(q2)*sin(q3)) + L1*sin(q1), -sin(q1)*(sin(q2)*(L2 + L3*cos(q3)) + L3*cos(q2)*sin(q3)), -sin(q1)*(L3*cos(q2)*sin(q3) + L3*cos(q3)*sin(q2));
            sin(q1)*(cos(q2)*(L2 + L3*cos(q3)) - L3*sin(q2)*sin(q3)) - L1*cos(q1),  cos(q1)*(sin(q2)*(L2 + L3*cos(q3)) + L3*cos(q2)*sin(q3)),  cos(q1)*(L3*cos(q2)*sin(q3) + L3*cos(q3)*sin(q2))];
    Torque_FR = (J_FR.')*F_FR;
    
    %RL
    q1 = q(7);
    q2 = q(8);
    q3 = q(9);
    J_RL = [                                                                    0,            L3*sin(q2)*sin(q3) - cos(q2)*(L2 + L3*cos(q3)),            L3*sin(q2)*sin(q3) - L3*cos(q2)*cos(q3);
            cos(q1)*(cos(q2)*(L2 + L3*cos(q3)) - L3*sin(q2)*sin(q3)) - L1*sin(q1), -sin(q1)*(sin(q2)*(L2 + L3*cos(q3)) + L3*cos(q2)*sin(q3)), -sin(q1)*(L3*cos(q2)*sin(q3) + L3*cos(q3)*sin(q2));
            sin(q1)*(cos(q2)*(L2 + L3*cos(q3)) - L3*sin(q2)*sin(q3)) + L1*cos(q1),  cos(q1)*(sin(q2)*(L2 + L3*cos(q3)) + L3*cos(q2)*sin(q3)),  cos(q1)*(L3*cos(q2)*sin(q3) + L3*cos(q3)*sin(q2))];
    Torque_RL = (J_RL.')*F_RL;
    
    %RR
    q1 = q(10);
    q2 = q(11);
    q3 = q(12);
    J_RR = [                                                                    0,            L3*sin(q2)*sin(q3) - cos(q2)*(L2 + L3*cos(q3)),            L3*sin(q2)*sin(q3) - L3*cos(q2)*cos(q3);
            cos(q1)*(cos(q2)*(L2 + L3*cos(q3)) - L3*sin(q2)*sin(q3)) + L1*sin(q1), -sin(q1)*(sin(q2)*(L2 + L3*cos(q3)) + L3*cos(q2)*sin(q3)), -sin(q1)*(L3*cos(q2)*sin(q3) + L3*cos(q3)*sin(q2));
            sin(q1)*(cos(q2)*(L2 + L3*cos(q3)) - L3*sin(q2)*sin(q3)) - L1*cos(q1),  cos(q1)*(sin(q2)*(L2 + L3*cos(q3)) + L3*cos(q2)*sin(q3)),  cos(q1)*(L3*cos(q2)*sin(q3) + L3*cos(q3)*sin(q2))];
    Torque_RR = (J_RR.')*F_RR;

    Torque = [Torque_FL;
              Torque_FR;
              Torque_RL;
              Torque_RR];
end