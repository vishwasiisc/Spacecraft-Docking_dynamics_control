function plot_frame(x,y,z,R_BN)



quiver3(x,y,z,R_BN(1,1),R_BN(1,2),R_BN(1,3),'r','LineWidth',1.3)
quiver3(x,y,z,R_BN(2,1),R_BN(2,2),R_BN(2,3),'g','LineWidth',1.3)
quiver3(x,y,z,R_BN(3,1),R_BN(3,2),R_BN(3,3),'b','LineWidth',1.3)



end