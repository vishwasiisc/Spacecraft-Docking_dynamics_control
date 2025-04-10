function trq_F = controller_docking(t,z,p)
%%
q_spacecraft     = z(1:4);
omega_spacecraft = z(5:7);

q_target = z(8:11);
omega_target = z(12:14);

r_spacecraft = z(15:17);
v_spacecraft = z(18:20);
q_dockframe  = p.q_dockframe; 
%% relative attitude stuff

target_rel_quat = q_dockframe;
current_rel_quat = subEP(q_spacecraft,q_target);

q_error = subEP(current_rel_quat,target_rel_quat);



%% relative omega stuff

%}
%
%R1 = quat2dcm(target_rel_craft_quat);
R = quat2dcm(q_dockframe);
R_target = quat2dcm(q_target);
R_spacecraft = quat2dcm(q_spacecraft);
%desired_omega = R*omega_target;
%}

%% gains

p_gain = 50*[0.01,    0,    0;
                0, 0.01,    0;
                0,    0, 0.01];

d_gain = 20*[ 0.15,    0,    0;
                0, 0.15,    0;
                0,    0, 0.15];

%% attitude controller

tau_spacecraft = -p_gain*q_error(2:4) - d_gain*(omega_spacecraft) + d_gain*omega_target;


%% position controller

r_dockport_target = R_target'*p.r_port_target;
r_dock_norm = r_dockport_target/norm(r_dockport_target);
r_dockport_spacecraft = r_spacecraft - R_spacecraft*p.r_port_spacecraft;

t0 = 100;

if t < t0
    r = r_spacecraft-5*r_dock_norm; 
    target_r = r*(1 - (t/t0)) + 5*r_dock_norm;
    error_r = target_r-r_spacecraft;
    v_target = [0,0,0]';
    %F_spacecraft = 50*error_r - 30*v_spacecraft;
else
    t1 = t-t0;
    r1 = 5 - (4/200)*t1;

    if r1<1
        r1 = 1;
    end

    target_r = r1*r_dock_norm;
    error_r = target_r-r_spacecraft;
    %F_spacecraft = 50*error_r - 30*v_spacecraft;
    v_target = cross(omega_target,target_r);

end


F_spacecraft = 400*error_r + 200*(v_target - v_spacecraft);
%F_spacecraft = [0,0,0]';


trq_F = [tau_spacecraft;F_spacecraft];

end