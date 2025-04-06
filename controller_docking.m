function trq_F = controller_docking(t,z,p)
%%
q_spacecraft     = z(1:4);
omega_spacecraft = z(5:7);

q_target = z(8:11);
omega_target = z(12:14);

r_spacecraft = z(15:17);
q_dockframe  = p.q_dockframe; 
%% relative attitude stuff

%desired_relative_quat = [cos(pi/4),0,0,sin(pi/4)]';
desired_relative_quat = q_dockframe;
current_relative_quat = subEP(q_target,q_spacecraft);
target_rel_craft_quat = subEP(q_spacecraft,q_target);
q_relative_error = q_subtract(desired_relative_quat,current_relative_quat);

%%
q_dockframenow = q_subtract(q_target,q_dockframe);
q_error = q_subtract(q_target,q_dockframenow);

%% relative omega stuff
%{
map current spacecraft frame to docking frame
relative omega in docking frame should be zero
omega_target - omega_in_dock_frame = 0
omega_in_dock_frame = current attitude to target current attitude


%}
%
%R1 = quat2dcm(target_rel_craft_quat);
R = quat2dcm(q_dockframe);
%desired_omega = R*omega_target;
%}

%% gains

p_gain = 100*[0.01,    0,    0;
                0, 0.01,    0;
                0,    0, 0.01];

d_gain = 3*[ 0.15,    0,    0;
                0, 0.15,    0;
                0,    0, 0.15];

%% controller

tau_spacecraft = p_gain*R'*q_error(2:4) - d_gain*(omega_spacecraft) + 0*d_gain*omega_target;
F_spacecraft = [0,0,0]';


trq_F = [tau_spacecraft;F_spacecraft];

end