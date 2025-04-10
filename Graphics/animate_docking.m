function animate_docking(soln,start,stop,speed,p)

r_port_target = p.r_port_target;
r_port_spacecraft = p.r_port_spacecraft;
R_dock = p.docking_frame;

target = p.target;
chaser = p.chaser;


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

    if t > 50
        t1 = t-50:0.2:t;
    else
        t1 = 0:0.8:t;
    end
    z1 = deval(soln,t1);

    q_spacecraft     = z(1:4);
    omega_spacecraft = z(5:7);

    q_target = z(8:11);
    omega_target = z(12:14);

    r_spacecraft = z(15:17);
    r1_spacecraft = z1(15:17,:);

%%
R_spacecraft = quat2dcm(q_spacecraft);
R_target     = quat2dcm(q_target);

target_p = target.Points*R_target;
chaser_p = chaser.Points*R_spacecraft;
chaser_p = chaser_p + r_spacecraft';

r_port_spacecraftp = r_spacecraft + R_spacecraft'*r_port_spacecraft;
r_port_targetp = R_target'*r_port_target;

docking_frame = R_dock*R_spacecraft;




%%

plot_frame(0,0,0,R_target);
%plot3(target_p(1,:),target_p(2,:),target_p(3,:),'r-');
trisurf(target.ConnectivityList,target_p(:,1),target_p(:,2),target_p(:,3),'FaceColor','r');%'edgecolor','none')
plot_frame(r_spacecraft(1),r_spacecraft(2),r_spacecraft(3),R_spacecraft);
%plot3(r1_spacecraft(1,:),r1_spacecraft(2,:),r1_spacecraft(3,:),'k')


%{
quiver3(r_spacecraft(1),r_spacecraft(2),r_spacecraft(3),docking_frame(1,1),docking_frame(2,1),docking_frame(3,1),'r-','LineWidth',1.1)
quiver3(r_spacecraft(1),r_spacecraft(2),r_spacecraft(3),docking_frame(1,2),docking_frame(2,2),docking_frame(3,2),'g-','LineWidth',1.1)
quiver3(r_spacecraft(1),r_spacecraft(2),r_spacecraft(3),docking_frame(1,3),docking_frame(2,3),docking_frame(3,3),'b-','LineWidth',1.1)
%}


plot3(r_port_targetp(1),r_port_targetp(2),r_port_targetp(3),'k*')
plot3(r_port_spacecraftp(1),r_port_spacecraftp(2),r_port_spacecraftp(3),'k*')
%plot3(chaser_p(1,:),chaser_p(2,:),chaser_p(3,:),'b-');
trisurf(chaser.ConnectivityList,chaser_p(:,1),chaser_p(:,2),chaser_p(:,3),'FaceColor','y');


axis("equal")
xlim([-5,5]);
ylim([-5,5]);
zlim([-5,5]);
xlabel("x")
ylabel("y")
zlabel("z")
view(40,40)

drawnow



    t = toc*speed;
end



end