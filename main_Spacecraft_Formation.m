%%
clc;
clear;
close all

addpath('/Users/vishwas/Documents/MATLAB/Pixxel/Support_funcs_RBK')

%% Target object stuff

I_target = [  1, 0.1, 0.1;
            0.1,   1, 0.2;
            0.1, 0.2,  1];

q0_target     = [1,0,0,0]; % target object initial attitude
omega0_target = 1*deg2rad([1,0,0]);
r_port_target = [0.5,0,0]'; %%in target body frame

inv_I1 = inv(I_target);

%% Chasing Spacecraft stuff
%
I_spacecraft = [1, 0, 0;
                0, 1, 0;
                0, 0, 1];
%}

m_spacecraft = 10;
q0_spacecraft     = [1,0,0,0];
%q_dockframe       = [cos(pi/4),0,0,sin(pi/4)];
q_dockframe = [1,0,0,0];

omega0_spacecraft = 0*[0.3,0.1,0.2];
r_spacecraft      = [1,1,1];
r_port_spacecraft = [0,0.5,0]'; % in chasing spacraft body frame
docking_frame = quat2dcm([0,0,0,1]);

inv_I = inv(I_spacecraft);


%% packing parameters
p.I_spacecraft = I_spacecraft;
p.I_target     = I_target;
p.m_spacecraft = m_spacecraft;
p.q_dockframe = q_dockframe;
p.inv_I  = inv_I;
p.inv_I1 = inv_I1;

p.r_port_target     = r_port_target;
p.r_port_spacecraft = r_port_spacecraft;
p.docking_frame = docking_frame;


%%



z0 = [q0_spacecraft, omega0_spacecraft, q0_target, omega0_target, r_spacecraft]';


start = 0; stop = 100; tspan = linspace(start,stop,10000);

zdot = @(t,z)rhs_zdot(t,z,p);

small = 1e-9;
options = odeset('AbsTol',small,'RelTol',small);

soln = ode45(zdot,tspan,z0,options);

plot_soln(soln,start,stop)
animate_docking(soln,start,stop,5,p)