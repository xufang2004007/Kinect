%%
%%计算旋转矩阵
syms alpha beta gama Px Py Pz;

%2014.12.17
% alpha = deg2rad(0.9124);
% beta = deg2rad(-32.8027);
% gama = deg2rad(-2.1281);

%2015.1.6
alpha = deg2rad(-0.3930);
beta = deg2rad(-37.8491);
gama = deg2rad(-1.8223);

R_X=[1 0 0;0 cos(gama) -sin(gama);0 sin(gama) cos(gama)];
R_Z=[cos(beta) -sin(beta) 0;sin(beta) cos(beta) 0;0 0 1];
R_Y=[cos(alpha) 0 sin(alpha);0 1 0;-sin(alpha) 0 cos(alpha)];


R = R_Y*R_Z*R_X
% Z=[0 0 0];
%
% R=[R_Y*R_Z*R_X;Z];
%
% P=[Px Py Pz 1]';
%
% T = [R,P];


%%
clear all;
pi=3.1415926;
filename = 'plane1.txt';
Point_Sensor=load(filename)';
T=[0,0,1;0,1,0;-1,0,0];
Point_World=zeros(size(Point_Sensor));
Point_World = T*Point_Sensor;
Point_World=Point_World';
figure;
scatter3(Point_World(:,1),Point_World(:,2),Point_World(:,3));
%%

%%选择数据,剔除掉不是平面的
j1=1;
for i1=1:1:size(Point_World,1)
    if (Point_World(i1,1)<=2.6)
        Point_Sel(j1,1)=Point_World(i1,1);
        Point_Sel(j1,2)=Point_World(i1,2);
        Point_Sel(j1,3)=Point_World(i1,3);
        j1=j1+1;
    end
end

%%选择数据，剔除掉深度等于0的无效点
j2=1;
for i2=1:1:size(Point_Sel,1)
    if (Point_Sel(i2,1)~=0)
        Point_NewSel(j2,1)=Point_Sel(i2,1);
        Point_NewSel(j2,2)=Point_Sel(i2,2);
        Point_NewSel(j2,3)=Point_Sel(i2,3);
        j2=j2+1;
    end
end

%%X Y Z
X=Point_NewSel(:,1);
Y=Point_NewSel(:,2);
Z=Point_NewSel(:,3);

[Plain,d]=Plain_Fit(X,Y,Z);

while(any(abs(d)>2*Plain(5,1)))
    clear x_new y_new z_new;
    j3=1;
    for i3=1:1:size(d)
        if (abs(d(i3))<=2*Plain(5,1))
            x_new(j3)=X(i3);
            y_new(j3)=Y(i3);
            z_new(j3)=Z(i3);
            j3=j3+1;
        end
    end
    clear X Y Z;
    X= x_new';
    Y= y_new';
    Z= z_new';
    clear d Plain;
    [Plain,d]=Plain_Fit(X,Y,Z);
end

if Plain(1) > 0
    Plane_Para = -Plain(1:4);
else
    Plane_Para = Plain(1:4);
end

%%
%RANSAC
clear all;
pi=3.1415926;
filename = 'plane1.txt';
Point_Sensor=load(filename)';
T=[0,0,1;0,1,0;-1,0,0];
Point_World=zeros(size(Point_Sensor));
Point_World = T*Point_Sensor;

%%
clear all;
clc
ofs  = fopen('Plane_Para.txt','wt');
i = 0;

clearvars -except i ofs;
%%原始数据
pi=3.1415926;
filename = strcat(num2str(i), '.pcd');
filename = strcat('./cloud/cloud',filename);
Point_Sensor=loadpcd(filename);
T=[-1,0,0;0,-1,0;0,0,1];
Point_World=zeros(size(Point_Sensor));
Point_World = T*Point_Sensor;
Point_World=Point_World';

%%
T2 = exp_TransM(-0.5761,-1.0462, 62.7083,0.0208598,0.1986032,0.5633780);
save('T2.mat','T2')
%%
T8 = exp_TransM(0.8892,-0.1879, 44.3982, 0.0107876,0.2614642,0.5367827);
% T8 = exp_TransM(44.3982, 0, 0,  0, 0, 0);
save('T8.mat','T8')
%%
clear all;
load('T8.mat');
load('T2.mat');
i = 0;
filename = strcat(num2str(i), '.pcd');
filename = strcat('./cloud1/cloud',filename);
Point_Sensor=loadpcd(filename);
Point_Sensor(4, :) = 1;

T=[-1, 0, 0, 0; 0, 0, 1, 0; 0, 1, 0, 0; 0, 0, 0, 1];
Point_World = T8*T*Point_Sensor;
% Point_World = T*Point_Sensor;
Point_World= Point_World';
figure;
scatter3(-Point_World(:,1),Point_World(:,3),Point_World(:,2));


%% For Kinect2
T8K = exp_TransM(1.8581, -0.2437, 53.4911, -0.0437089, 0.3875362, 0.5138082);
% T8 = exp_TransM(44.3982, 0, 0,  0, 0, 0);
save('T8K.mat','T8K')

%% For Kinect2
T8K = exp_TransM(1.4087, -0.0755, 53.6747, -0.0458843, 0.3380042, 0.5216166);
% T8 = exp_TransM(44.3982, 0, 0,  0, 0, 0);
save('T8K.mat','T8K')

%%
 % 4.6745   -0.4048   56.8403      -21.0399  321.5179  377.5623
 
 T9K = exp_TransM(4.6745, -0.4048, 56.8403, -0.0210399, 0.3215179, 0.3775623);
% T8 = exp_TransM(44.3982, 0, 0,  0, 0, 0);
save('T9K.mat','T9K')
%%
clear all;
load('T9K.mat');
i = 61;
filename = strcat(num2str(i), '.pcd');
filename = strcat('./cloud1/cloud',filename);
Point_Sensor=loadpcd(filename);
Point_Sensor(4, :) = 1;
T9K(3, 4) = T9K(3, 4) +  0.28;

T=[1, 0, 0, 0; 0, -1, 0, 0; 0, 0, 1, 0; 0, 0, 0, 1];
Point_World = T9K*T*Point_Sensor;
% Point_World = T*Point_Sensor;
Point_World=Point_World';
figure;

scatter3(-Point_World(:,1),Point_World(:,3),Point_World(:,2));


%%
T9 = [0.9990, 0.0392, 0.0231, -0.0447;
    0.0006, 0.4977, -0.8673, 0.3243;
    -0.0455, 0.8665, 0.4972, 0.4963 + 1;
    0, 0, 0, 1];
save('T9.mat', 'T9');


