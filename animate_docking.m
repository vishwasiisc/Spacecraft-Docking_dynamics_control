function animate_docking(soln,start,stop,speed,p)

r_port_target = p.r_port_target;
r_port_spacecraft = p.r_port_spacecraft;
R_dock = p.docking_frame;



tic
figure(10)
movegui('northeast')
hold on
t = 0;
while t < stop
    figure(10)
    clf
    hold on
    grid on
    z = deval(soln,t);

    q_spacecraft     = z(1:4);
    omega_spacecraft = z(5:7);

    q_target = z(8:11);
    omega_target = z(12:14);

    r_spacecraft = z(15:17);

%%
R_spacecraft = quat2dcm(q_spacecraft);
R_target     = quat2dcm(q_target);

r_port_spacecraftp = r_spacecraft + R_spacecraft'*r_port_spacecraft;
r_port_targetp = R_target'*r_port_target;

docking_frame = R_dock*R_spacecraft;




%%

plot_frame(0,0,0,R_target);
plot_frame(r_spacecraft(1),r_spacecraft(2),r_spacecraft(3),R_spacecraft);


%{
quiver3(r_spacecraft(1),r_spacecraft(2),r_spacecraft(3),docking_frame(1,1),docking_frame(2,1),docking_frame(3,1),'r-','LineWidth',1.1)
quiver3(r_spacecraft(1),r_spacecraft(2),r_spacecraft(3),docking_frame(1,2),docking_frame(2,2),docking_frame(3,2),'g-','LineWidth',1.1)
quiver3(r_spacecraft(1),r_spacecraft(2),r_spacecraft(3),docking_frame(1,3),docking_frame(2,3),docking_frame(3,3),'b-','LineWidth',1.1)
%}


plot3(r_port_targetp(1),r_port_targetp(2),r_port_targetp(3),'k*')
plot3(r_port_spacecraftp(1),r_port_spacecraftp(2),r_port_spacecraftp(3),'k*')

axis("equal")
xlim([-2,2]);
ylim([-2,2]);
zlim([-2,2]);
xlabel("x")
ylabel("y")
zlabel("z")
view(40,30)

drawnow



    t = toc*speed;
end



end