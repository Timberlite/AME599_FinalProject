function flag = Joint_Monitor(q,dq)
global robot_params;
q_max_hip = robot_params.q_max_hip;
q_min_hip = robot_params.q_min_hip;
q_max_thigh = robot_params.q_max_thigh;
q_min_thigh = robot_params.q_min_thigh;
q_max_calf = robot_params.q_max_calf;
q_min_calf = robot_params.q_min_calf;
dq_max = robot_params.dq_max;
dq_min = robot_params.dq_min;

q_max = repmat([q_max_hip;q_max_thigh;q_max_calf],4,1);
q_min = repmat([q_min_hip;q_min_thigh;q_min_calf],4,1);
dqs_max = dq_max*ones(12,1);
dqs_min = dq_min*ones(12,1);

flag = 0;
if any(q_max<q)
    flag = 1;
elseif any(q_min>q)
    flag = 1;
end
if any(dqs_max<dq)
    flag = 1;
elseif any(dqs_min>dq)
    flag = 1;
end

if flag==1
    disp('Joint limit violated.');
    flag = 0;
end
end

