function qdot = qdot_rhs(omega,q)

omega_x = omega(1);
omega_y = omega(2);
omega_z = omega(3);

q0 = q(1);
q1 = q(2);
q2 = q(3);
q3 = q(4);


omega_    = [omega_x,omega_y,omega_z]';

Qdot_mat = [ -q1, -q2, -q3;
              q0, -q3,  q2;
              q3,  q0, -q1;
             -q2,  q1,  q0];



qdot = Qdot_mat*omega_; %colomn vector;


end