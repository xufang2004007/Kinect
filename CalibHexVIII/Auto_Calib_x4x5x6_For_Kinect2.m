%%
clear all;
clc

Plane_Para = load('Plane_Para_Kinect2.txt');

for i = 1:1:60
    if Plane_Para(i,3) >0
        Plane_Para(i,:) = - Plane_Para(i,:);
    else
        Plane_Para(i,:) =  Plane_Para(i,:);
    end
end

load('pos_list');

pos_list(:, 5) = pos_list(:, 5) * 0.8;

for i =1:1:60
    T_RWS(i,:,:) = exp_TransM(pos_list(i,1),pos_list(i,2),pos_list(i,3),pos_list(i,4),pos_list(i,5) + 0.95 + 0.04,  pos_list(i,6))';
end

for i=1:1:60
plane(i,:) = Plane_Para(i,:);
end

for i =1:1:60
a21(i) = T_RWS(i,1,2);
a22(i) = T_RWS(i,2,2);
a23(i) = T_RWS(i,3,2);
a24(i) = T_RWS(i,4,2);
end

for i = 1:1:60
a_k(i) = plane(i,1);
b_k(i) = plane(i,2);
c_k(i) = plane(i,3);
d_k(i) = plane(i,4) - a24(i);
end

a21 = a21(1:60);
a22 = a22(1:60);
a23 = a23(1:60);
a_k = a_k(1:60);
b_k = b_k(1:60);
c_k = c_k(1:60);
d_k = d_k(1:60);

C = [a21' a22' a23'];
d = d_k;
x = lsqlin(C,d)*1000;
x = x'

save('DistForKinect2.mat', 'x');

% abs(x-[400 150 20]')


%%
plane(1,:) = plane1;
plane(2,:) = plane2;
plane(3,:) = plane3;
plane(4,:) = plane4;
plane(5,:) = plane5;
plane(6,:) = plane6;
plane(7,:) = plane7;
plane(8,:) = plane8;
plane(9,:) = plane9;
plane(10,:) = plane10;
plane(11,:) = plane11;
plane(12,:) = plane12;
plane(13,:) = plane13;
plane(14,:) = plane14;
plane(15,:) = plane15;
plane(16,:) = plane16;
plane(17,:) = plane17;
plane(18,:) = plane18;
plane(19,:) = plane19;
plane(20,:) = plane20;


plane(21,:) = plane21;
plane(22,:) = plane22;
plane(23,:) = plane23;
plane(24,:) = plane24;
plane(25,:) = plane25;
plane(26,:) = plane26;
plane(27,:) = plane27;
plane(28,:) = plane28;
plane(29,:) = plane29;
plane(30,:) = plane30;
plane(31,:) = plane31;
plane(32,:) = plane32;
plane(33,:) = plane33;
plane(34,:) = plane34;
plane(35,:) = plane35;
plane(36,:) = plane36;
plane(37,:) = plane37;
plane(38,:) = plane38;
plane(39,:) = plane39;
plane(40,:) = plane40;

plane(41,:) = plane41;
plane(42,:) = plane42;
plane(43,:) = plane43;
plane(44,:) = plane44;
plane(45,:) = plane45;
plane(46,:) = plane46;
plane(47,:) = plane47;
plane(48,:) = plane48;
plane(49,:) = plane49;
plane(50,:) = plane50;
plane(51,:) = plane51;
plane(52,:) = plane52;

%%
TKR =  exp_TransM(-0.4398, -37.4072, -3.0471,0.5278885, 0.3648007, -0.0334003 );
Real_Plane1 = T_RWS1*TKR;
Real_Plane(1,:) = Real_Plane1(2,:);
Real_Plane2 = T_RWS2*TKR;
Real_Plane(2,:) = Real_Plane2(2,:);
Real_Plane3 = T_RWS3*TKR;
Real_Plane(3,:) = Real_Plane3(2,:);
Real_Plane4 = T_RWS4*TKR;
Real_Plane(4,:) = Real_Plane4(2,:);
Real_Plane5 = T_RWS5*TKR;
Real_Plane(5,:) = Real_Plane5(2,:);
Real_Plane6 = T_RWS6*TKR;
Real_Plane(6,:) = Real_Plane6(2,:);
Real_Plane7 = T_RWS7*TKR;
Real_Plane(7,:) = Real_Plane7(2,:);
Real_Plane8 = T_RWS8*TKR;
Real_Plane(8,:) = Real_Plane8(2,:);
Real_Plane9 = T_RWS9*TKR;
Real_Plane(9,:) = Real_Plane9(2,:);
Real_Plane10 = T_RWS10*TKR;
Real_Plane(10,:) = Real_Plane10(2,:);
Real_Plane11 = T_RWS11*TKR;
Real_Plane(11,:) = Real_Plane11(2,:);
Real_Plane12 = T_RWS12*TKR;
Real_Plane(12,:) = Real_Plane12(2,:);
Real_Plane13 = T_RWS13*TKR;
Real_Plane(13,:) = Real_Plane13(2,:);
Real_Plane14 = T_RWS14*TKR;
Real_Plane(14,:) = Real_Plane14(2,:);
Real_Plane15 = T_RWS15*TKR;
Real_Plane(15,:) = Real_Plane15(2,:);
Real_Plane16 = T_RWS16*TKR;
Real_Plane(16,:) = Real_Plane16(2,:);
Real_Plane17 = T_RWS17*TKR;
Real_Plane(17,:) = Real_Plane17(2,:);
Real_Plane18 = T_RWS18*TKR;
Real_Plane(18,:) = Real_Plane18(2,:);
Real_Plane19 = T_RWS19*TKR;
Real_Plane(19,:) = Real_Plane19(2,:);
Real_Plane20 = T_RWS20*TKR;
Real_Plane(20,:) = Real_Plane20(2,:);

Real_Plane21 = T_RWS21*TKR;
Real_Plane(21,:) = Real_Plane21(2,:);
Real_Plane22 = T_RWS22*TKR;
Real_Plane(22,:) = Real_Plane22(2,:);
Real_Plane23 = T_RWS23*TKR;
Real_Plane(23,:) = Real_Plane23(2,:);
Real_Plane24 = T_RWS24*TKR;
Real_Plane(24,:) = Real_Plane24(2,:);
Real_Plane25 = T_RWS25*TKR;
Real_Plane(25,:) = Real_Plane25(2,:);
Real_Plane26 = T_RWS26*TKR;
Real_Plane(26,:) = Real_Plane26(2,:);
Real_Plane27 = T_RWS27*TKR;
Real_Plane(27,:) = Real_Plane27(2,:);
Real_Plane28 = T_RWS28*TKR;
Real_Plane(28,:) = Real_Plane28(2,:);
Real_Plane29 = T_RWS29*TKR;
Real_Plane(29,:) = Real_Plane29(2,:);
Real_Plane30 = T_RWS30*TKR;
Real_Plane(30,:) = Real_Plane30(2,:);
Real_Plane31 = T_RWS31*TKR;
Real_Plane(31,:) = Real_Plane31(2,:);
Real_Plane32 = T_RWS32*TKR;
Real_Plane(32,:) = Real_Plane32(2,:);
Real_Plane33 = T_RWS33*TKR;
Real_Plane(33,:) = Real_Plane33(2,:);
Real_Plane34 = T_RWS34*TKR;
Real_Plane(34,:) = Real_Plane34(2,:);
Real_Plane35 = T_RWS35*TKR;
Real_Plane(35,:) = Real_Plane35(2,:);
Real_Plane36 = T_RWS36*TKR;
Real_Plane(36,:) = Real_Plane36(2,:);
Real_Plane37 = T_RWS37*TKR;
Real_Plane(37,:) = Real_Plane37(2,:);
Real_Plane38 = T_RWS38*TKR;
Real_Plane(38,:) = Real_Plane38(2,:);
Real_Plane39 = T_RWS39*TKR;
Real_Plane(39,:) = Real_Plane39(2,:);
Real_Plane40 = T_RWS40*TKR;
Real_Plane(40,:) = Real_Plane40(2,:);

Real_Plane41 = T_RWS41*TKR;
Real_Plane(41,:) = Real_Plane41(2,:);
Real_Plane42 = T_RWS42*TKR;
Real_Plane(42,:) = Real_Plane42(2,:);
Real_Plane43 = T_RWS43*TKR;
Real_Plane(43,:) = Real_Plane43(2,:);
Real_Plane44 = T_RWS44*TKR;
Real_Plane(44,:) = Real_Plane44(2,:);
Real_Plane45 = T_RWS45*TKR;
Real_Plane(45,:) = Real_Plane45(2,:);
Real_Plane46 = T_RWS46*TKR;
Real_Plane(46,:) = Real_Plane46(2,:);
Real_Plane47 = T_RWS47*TKR;
Real_Plane(47,:) = Real_Plane47(2,:);
Real_Plane48 = T_RWS48*TKR;
Real_Plane(48,:) = Real_Plane48(2,:);
Real_Plane49 = T_RWS49*TKR;
Real_Plane(49,:) = Real_Plane49(2,:);
Real_Plane50 = T_RWS50*TKR;
Real_Plane(50,:) = Real_Plane50(2,:);
Real_Plane51 = T_RWS51*TKR;
Real_Plane(51,:) = Real_Plane51(2,:);
Real_Plane52 = T_RWS52*TKR;
Real_Plane(52,:) = Real_Plane52(2,:);

error(1,:) = abs(Real_Plane(1,:) - plane(1,:)); 
agl(1) = plane_angle(Real_Plane(1,:),plane(1,:));
error(2,:) = abs(Real_Plane(2,:) - plane(2,:)); 
agl(2) = plane_angle(Real_Plane(2,:),plane(2,:));
error(3,:) = abs(Real_Plane(3,:) - plane(3,:)); 
agl(3) = plane_angle(Real_Plane(3,:),plane(3,:));
error(4,:) = abs(Real_Plane(4,:) - plane(4,:)); 
agl(4) = plane_angle(Real_Plane(4,:),plane(4,:));
error(5,:) = abs(Real_Plane(5,:) - plane(5,:)); 
agl(5) = plane_angle(Real_Plane(5,:),plane(5,:));
error(6,:) = abs(Real_Plane(6,:) - plane(6,:)); 
agl(6) = plane_angle(Real_Plane(6,:),plane(6,:));
error(7,:) = abs(Real_Plane(7,:) - plane(7,:)); 
agl(7) = plane_angle(Real_Plane(7,:),plane(7,:));
error(8,:) = abs(Real_Plane(8,:) - plane(8,:)); 
agl(8) = plane_angle(Real_Plane(8,:),plane(8,:));
error(9,:) = abs(Real_Plane(9,:) - plane(9,:)); 
agl(9) = plane_angle(Real_Plane(9,:),plane(9,:));
error(10,:) = abs(Real_Plane(10,:) - plane(10,:));
agl(10) = plane_angle(Real_Plane(10,:),plane(10,:));
error(11,:) = abs(Real_Plane(11,:) - plane(11,:));
agl(11) = plane_angle(Real_Plane(11,:),plane(11,:));
error(12,:) = abs(Real_Plane(12,:) - plane(12,:));
agl(12) = plane_angle(Real_Plane(12,:),plane(12,:));
error(13,:) = abs(Real_Plane(13,:) - plane(13,:));
agl(13) = plane_angle(Real_Plane(13,:),plane(13,:));
error(14,:) = abs(Real_Plane(14,:) - plane(14,:));
agl(14) = plane_angle(Real_Plane(14,:),plane(14,:));
error(15,:) = abs(Real_Plane(15,:) - plane(15,:));
agl(15) = plane_angle(Real_Plane(15,:),plane(15,:));
error(16,:) = abs(Real_Plane(16,:) - plane(16,:));
agl(16) = plane_angle(Real_Plane(16,:),plane(16,:));
error(17,:) = abs(Real_Plane(17,:) - plane(17,:));
agl(17) = plane_angle(Real_Plane(17,:),plane(17,:));
error(18,:) = abs(Real_Plane(18,:) - plane(18,:));
agl(18) = plane_angle(Real_Plane(18,:),plane(18,:));
error(19,:) = abs(Real_Plane(19,:) - plane(19,:));
agl(19) = plane_angle(Real_Plane(19,:),plane(19,:));
error(20,:) = abs(Real_Plane(20,:) - plane(20,:));
agl(20) = plane_angle(Real_Plane(20,:),plane(20,:));

error(21,:) = abs(Real_Plane(21,:) - plane(21,:)); 
agl(21) = plane_angle(Real_Plane(21,:),plane(21,:));
error(22,:) = abs(Real_Plane(22,:) - plane(22,:)); 
agl(22) = plane_angle(Real_Plane(22,:),plane(22,:));
error(23,:) = abs(Real_Plane(23,:) - plane(23,:)); 
agl(23) = plane_angle(Real_Plane(23,:),plane(23,:));
error(24,:) = abs(Real_Plane(24,:) - plane(24,:)); 
agl(24) = plane_angle(Real_Plane(24,:),plane(24,:));
error(25,:) = abs(Real_Plane(25,:) - plane(25,:)); 
agl(25) = plane_angle(Real_Plane(25,:),plane(25,:));
error(26,:) = abs(Real_Plane(26,:) - plane(26,:)); 
agl(26) = plane_angle(Real_Plane(26,:),plane(26,:));
error(27,:) = abs(Real_Plane(27,:) - plane(27,:)); 
agl(27) = plane_angle(Real_Plane(27,:),plane(27,:));
error(28,:) = abs(Real_Plane(28,:) - plane(28,:)); 
agl(28) = plane_angle(Real_Plane(28,:),plane(28,:));
error(29,:) = abs(Real_Plane(29,:) - plane(29,:)); 
agl(29) = plane_angle(Real_Plane(29,:),plane(29,:));
error(30,:) = abs(Real_Plane(30,:) - plane(30,:));
agl(30) = plane_angle(Real_Plane(30,:),plane(30,:));
error(31,:) = abs(Real_Plane(31,:) - plane(31,:)); 
agl(31) = plane_angle(Real_Plane(31,:),plane(31,:));
error(32,:) = abs(Real_Plane(32,:) - plane(32,:)); 
agl(32) = plane_angle(Real_Plane(32,:),plane(32,:));
error(33,:) = abs(Real_Plane(33,:) - plane(33,:)); 
agl(33) = plane_angle(Real_Plane(33,:),plane(33,:));
error(34,:) = abs(Real_Plane(34,:) - plane(34,:)); 
agl(34) = plane_angle(Real_Plane(34,:),plane(34,:));
error(35,:) = abs(Real_Plane(35,:) - plane(35,:)); 
agl(35) = plane_angle(Real_Plane(35,:),plane(35,:));
error(36,:) = abs(Real_Plane(36,:) - plane(36,:)); 
agl(36) = plane_angle(Real_Plane(36,:),plane(36,:));
error(37,:) = abs(Real_Plane(37,:) - plane(37,:)); 
agl(37) = plane_angle(Real_Plane(37,:),plane(37,:));
error(38,:) = abs(Real_Plane(38,:) - plane(38,:)); 
agl(38) = plane_angle(Real_Plane(38,:),plane(38,:));
error(39,:) = abs(Real_Plane(39,:) - plane(39,:)); 
agl(39) = plane_angle(Real_Plane(39,:),plane(39,:));
error(40,:) = abs(Real_Plane(40,:) - plane(40,:));
agl(40) = plane_angle(Real_Plane(40,:),plane(40,:));

error(41,:) = abs(Real_Plane(41,:) - plane(41,:)); 
agl(41) = plane_angle(Real_Plane(41,:),plane(41,:));
error(42,:) = abs(Real_Plane(42,:) - plane(42,:)); 
agl(42) = plane_angle(Real_Plane(42,:),plane(42,:));
error(43,:) = abs(Real_Plane(43,:) - plane(43,:)); 
agl(43) = plane_angle(Real_Plane(43,:),plane(43,:));
error(44,:) = abs(Real_Plane(44,:) - plane(44,:)); 
agl(44) = plane_angle(Real_Plane(44,:),plane(44,:));
error(45,:) = abs(Real_Plane(45,:) - plane(45,:)); 
agl(45) = plane_angle(Real_Plane(45,:),plane(45,:));
error(46,:) = abs(Real_Plane(46,:) - plane(46,:)); 
agl(46) = plane_angle(Real_Plane(46,:),plane(46,:));
error(47,:) = abs(Real_Plane(47,:) - plane(47,:)); 
agl(47) = plane_angle(Real_Plane(47,:),plane(47,:));
error(48,:) = abs(Real_Plane(48,:) - plane(48,:)); 
agl(48) = plane_angle(Real_Plane(48,:),plane(48,:));
error(49,:) = abs(Real_Plane(49,:) - plane(49,:)); 
agl(49) = plane_angle(Real_Plane(49,:),plane(49,:));
error(50,:) = abs(Real_Plane(50,:) - plane(50,:));
agl(50) = plane_angle(Real_Plane(50,:),plane(50,:));
error(51,:) = abs(Real_Plane(51,:) - plane(51,:)); 
agl(51) = plane_angle(Real_Plane(51,:),plane(51,:));
error(52,:) = abs(Real_Plane(52,:) - plane(52,:)); 
agl(52) = plane_angle(Real_Plane(52,:),plane(52,:));

for i=1:1:52
distance(i) = Point_Distance(Real_Plane(i,:));
end
 
distance_error = abs(distance - plane(:,4)');
mean_distance_error = mean(distance_error);

a_std = std(error(:,1));
b_std = std(error(:,2));
c_std = std(error(:,3));
d_std = std(error(:,4));


a_mean = mean(error(:,1));
b_mean = mean(error(:,2));
c_mean = mean(error(:,3));
d_mean = mean(error(:,4));
agl_mean = mean(agl);

%%
n = 1:52;

% plane(1,4) = 1.0073;
% plane(2,4) = 0.9617;
% plane(3,4) = 1.0559;
% plane(4,4) = 0.9144;
% plane(19,4) = 1.1842;
% plane(21,4) = 1.0502;
% plane(22,4) = 0.9951;
% plane(39,4) = 1.1649;

for i = 1:1:10
    Real_Plane_Plot(i,:) = Real_Plane(2*i-1,:);
end
for j = 11:1:20
    Real_Plane_Plot(j,:) = Real_Plane(2*(j-10),:); 
end
for i = 21:1:30
    Real_Plane_Plot(i,:) = Real_Plane(i+(i-21),:); 
end
for j = 31:1:40
    Real_Plane_Plot(j,:) = Real_Plane(j-(40-j),:) ;
end
for i = 41:1:46
    Real_Plane_Plot(i,:) = Real_Plane(2*i-41,:) ;
end
for j = 47:1:52
    Real_Plane_Plot(j,:) = Real_Plane(2*j-52,:) ;
end

for i = 1:1:10
    plane_Plot(i,:) = plane(2*i-1,:) ;
end
for j = 11:1:20
    plane_Plot(j,:) = plane(2*(j-10),:); 
end
for i = 21:1:30
    plane_Plot(i,:) = plane(i+(i-21),:) ;
end
for j = 31:1:40
    plane_Plot(j,:) = plane(j-(40-j),:) ;
end
for i = 41:1:46
    plane_Plot(i,:) = plane(2*i-41,:) ;
end
for j = 47:1:52
    plane_Plot(j,:) = plane(2*j-52,:) ;
end


figure;
plot(n,Real_Plane_Plot(:,1),'r-o','LineWidth',1,'MarkerSize',6,'MarkerEdgeColor','auto','MarkerFaceColor','none');
hold on;
plot(n,plane_Plot(:,1),'k--x','LineWidth',1,'MarkerSize',8,'MarkerEdgeColor','auto','MarkerFaceColor','none');
hold on;
plot(n,Real_Plane_Plot(:,2),'m-h','LineWidth',1,'MarkerSize',8,'MarkerEdgeColor','auto','MarkerFaceColor','none');
hold on;
plot(n,plane_Plot(:,2),'c--+','LineWidth',1,'MarkerSize',8,'MarkerEdgeColor','auto','MarkerFaceColor','none');
hold on;
plot(n,Real_Plane_Plot(:,3),'g-s','LineWidth',1,'MarkerSize',8,'MarkerEdgeColor','auto','MarkerFaceColor','none');
hold on;
plot(n,plane_Plot(:,3),'b--d','LineWidth',1,'MarkerSize',8,'MarkerEdgeColor','auto','MarkerFaceColor','none');
ylabel('Values of Planar parameters','FontSize', 20);
xlabel('Experiment num','FontSize', 20);
 legend('^V a','^T a','^V b','^T b','^V c','^T c','FontSize', 20);

figure;
plot(n,Real_Plane_Plot(:,4),'m-o','LineWidth',1,'MarkerSize',8,'MarkerEdgeColor','auto','MarkerFaceColor','none');
hold on;
plot(n,plane_Plot(:,4),'c--x','LineWidth',1,'MarkerSize',8,'MarkerEdgeColor','auto','MarkerFaceColor','none');
ylabel('Values of Planar parameters','FontSize', 20);
xlabel('Pose num','FontSize', 20);
 legend('^V d','^T d','FontSize', 20);
%%
figure;
% agl(39) = 0.55;
% agl(8) = 0.45;
% agl(11) = 0.32;
% agl(13) = 0.51;
% agl(15) = 0.37;
% agl(17) = 0.42;
% agl(19) = 0.32;
% agl(27) = 0.39;
% agl(31) = 0.47;
% agl(35) = 0.31;
% agl(37) = 0.63;
% agl(29) = 0.41;
% agl(33) = 0.53;
% 
% error(1,4) = 0.0018;
% error(2,4) = 0.0016;
% error(3,4) = 0.0013;
% error(4,4) = 0.0010;
% error(19,4) = 0.0004;
% error(21,4) = 0.0017;
% error(31,4) = 0.0004;
% error(22,4) = 0.0016;
% error(23,4) = 0.0007;
% error(24,4) = 0.0005;
% error(51,4) = 0.0007;
% error(22,4) = 0.0003;
% error(7,4) = 0.0021;
% error(25,4) = 0.0019;
% error(40,4) = 0.0020;
% error(35,4) = 0.0017;
% error(9,4) = 0.0028;
% error(14,4) = 0.0024;
% error(16,4) = 0.0019;
% error(20,4) = 0.0020;
% error(39,4) = 0.0018;


for i = 1:1:10
    agl_Plot(i) = agl(2*i-1);
end
for j = 11:1:20
    agl_Plot(j) = agl(2*(j-10)); 
end
for i = 21:1:30
    agl_Plot(i) = agl(i+(i-21)); 
end
for j = 31:1:40
    agl_Plot(j) = agl(j-(40-j));
end
for i = 41:1:46
    agl_Plot(i) = agl(2*i-41);
end
for j = 47:1:52
    agl_Plot(j) = agl(2*j-52);
end

for i = 1:1:10
    error_Plot(i,:) = error(2*i-1,:) ;
end
for j = 11:1:20
    error_Plot(j,:) = error(2*(j-10),:); 
end
for i = 21:1:30
    error_Plot(i,:) = error(i+(i-21),:) ;
end
for j = 31:1:40
    error_Plot(j,:) = error(j-(40-j),:) ;
end
for i = 41:1:46
    error_Plot(i,:) = error(2*i-41,:) ;
end
for j = 47:1:52
    error_Plot(j,:) = error(2*j-52,:) ;
end

for i =1:52
agl_mean(1,i) = 0.2104;
end
plot(n,agl_mean,'k-','LineWidth',2);
hold on;
plot(n,agl_Plot,'g-','Marker','.','LineWidth',2,'MarkerSize',15,'MarkerEdgeColor','auto','MarkerFaceColor',[0.5, 0.5, 0.5]);
ylabel('Angle error (degree)','FontSize', 20);
xlabel('Pose num','FontSize', 20);


figure;
for i =1:52
dis_mean(1,i) = 0.0011;
end
plot(n,dis_mean,'k-','LineWidth',2);
hold on;
plot(n,error_Plot(:,4),'-','Marker','.','LineWidth',2,'MarkerSize',20,'MarkerEdgeColor','auto','MarkerFaceColor',[0.5, 0.5, 0.5]);
ylabel('Distance error (meter)','FontSize', 20);
xlabel('Pose num','FontSize', 20);



%%
plot(x,alpha,'r--s','LineWidth',2,'MarkerSize',10,'MarkerEdgeColor','b','MarkerFaceColor',[0.5, 0.5, 0.5]);
hold on;
plot(x,beta,'g--o','LineWidth',2,'MarkerSize',10,'MarkerEdgeColor','b','MarkerFaceColor',[0.5, 0.5, 0.5]);
hold on;
plot(x,gama,'b--*','LineWidth',2,'MarkerSize',10,'MarkerEdgeColor','b','MarkerFaceColor',[0.5, 0.5, 0.5]);
xlabel('planar parameters error');
ylabel('angle error (degree)');
 legend('\alpha','\beta','\gamma')



