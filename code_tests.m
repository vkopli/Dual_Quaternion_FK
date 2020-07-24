q1 = [0,0,0,0,0,0];
q2 = [0,0,0,0,0,0];
q3 = [0,0,0,0,0,0];
q4 = [0,0,0,0,0,0];
q5 = [0,0,0,0,0,0];
q6 = [0,0,0,0,0,0];
qs = {q1, q2, q3, q4, q5, q6};

for i = 1:length(qs)
    [o_quat, T0e] = calculateFK_quaternion(qs{i});
    [o_sol, Q0e] = calculateFK_sol(qs{i});
    disp(['Moving Joint ', num2str(i)])
    o_quat
    o_sol
end

