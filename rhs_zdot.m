function zdot = rhs_zdot(t,z,p)
%% unpacking states
q_spacecraft     = z(1:4);
omega_spacecraft = z(5:7);

q_target = z(8:11);
omega_target = z(12:14);

r_spacecraft = z(15:17);

%% unpacking parameters
inv_I_spacecraft = p.inv_I;
inv_I_target     = p.inv_I1;

m_spacecraft = p.m_spacecraft;
I_spacecraft = p.I_spacecraft;
I_target = p.I_target;

%% quaternion dynamics

q_dot_spacecraft = qdot_rhs(omega_spacecraft,q_spacecraft);
q_dot_target     = qdot_rhs(omega_target,q_target);


%% spacecraft and target control torques

tau_F = controller_docking(t,z,p);
tau_target     = [0,0,0]';

tau_spacecraft = tau_F(1:3);
F_spacecraft = tau_F(4:6);
%% rigid body dynamics

omegadot_spacecraft = inv_I_spacecraft*(tau_spacecraft - cross(omega_spacecraft,I_spacecraft*omega_spacecraft));
omegadot_target = inv_I_target*(tau_target - cross(omega_target,I_target*omega_target));
r_spacecraftdot = F_spacecraft/m_spacecraft;





zdot = [q_dot_spacecraft;omegadot_spacecraft;q_dot_target;omegadot_target;r_spacecraftdot];
end