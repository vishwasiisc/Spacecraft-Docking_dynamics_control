function delta_q = q_subtract(q_target,q)



q0t = q_target(1);
q1t = q_target(2);
q2t = q_target(3);
q3t = q_target(4);


q_now = [q(1),q(2),q(3),q(4)]';

delta_q = [q0t,  q1t,  q2t,  q3t;
           q1t, -q0t, -q3t,  q2t;
           q2t,  q3t, -q0t, -q1t;
           q3t, -q2t,  q1t, -q0t]*q_now;

if delta_q(1) < 0
   % delta_q = -delta_q;
end

if norm(delta_q(2:4)) < 1e-7
    delta_q(2) = 0;
    delta_q(3) = 0;
    delta_q(4) = 0;
end

disp(delta_q)


%% trajectory generation
%{
T0 = 100; %manuver time

phi0   = acos(delta_q(1));
e_hat = [delta_q(2),delta_q(3),delta_q(4)]'/sin(phi);


%}
         


end