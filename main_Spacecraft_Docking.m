%%
clc;
clear;
close all

addpath('/Users/vishwas/Documents/MATLAB/Pixxel/Support_funcs_RBK')
addpath('Controller/','Dynamics/','Graphics/','Support_functions/')

%% Target object stuff

I_target = [  1, 0.1, 0.1;
            0.1,   1.5, 0.2;
            0.1, 0.2,  1.7];

q0_target     = [1,0,0,0]; % target object initial attitude
omega0_target = 2*deg2rad([1,1,1]);
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
%q0_spacecraft = q_dockframe;
q_dockframe = [1,0,0,0];

omega0_spacecraft = 0*[0.3,0.1,0.2];
r_spacecraft      = [6,6,6];
v_spacecraft      = [0,0,0];
r_port_spacecraft = [-0.5,0,0]'; % in chasing spacraft body frame
docking_frame = quat2dcm(q_dockframe);

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

cube = stlread('Body1.stl');

chaser_points = cube.Points;
chaser_points = chaser_points*quat2dcm([0,0,0,1]);


target = struct('Points',cube.Points,'ConnectivityList',cube.ConnectivityList);
chaser = struct('Points',chaser_points,'ConnectivityList',cube.ConnectivityList);
p.target = target;
p.chaser = chaser;



%%



z0 = [q0_spacecraft, omega0_spacecraft, q0_target, omega0_target, r_spacecraft,v_spacecraft]';


start = 0; stop = 500; tspan = linspace(start,stop,10000);

zdot = @(t,z)rhs_zdot(t,z,p);

small = 1e-12;
options = odeset('AbsTol',small,'RelTol',small);

soln = ode45(zdot,tspan,z0,options);

plot_soln(soln,start,stop)
animate_docking(soln,start,stop,10,p)