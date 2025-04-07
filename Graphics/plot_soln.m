function plot_soln(soln,start,stop)

close all


t = soln.x;
z = soln.y;

q_spacecraft = z(1:4,:);
q_target     = z(8:11,:);
r_spacecraft = z(15:17,:);

figure(1)
subplot(3,1,1)
hold on
title("target quat")
plot(t,q_target);

subplot(3,1,2)
hold on
title("spacecraft quat")
plot(t,q_spacecraft);

subplot(3,1,3)
hold on
title("tracking error quat")
plot(t,q_target - q_spacecraft)

movegui('northwest')


figure(3)
plot(t,r_spacecraft);
movegui('southwest')






end