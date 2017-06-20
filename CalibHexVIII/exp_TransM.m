function T = exp_TransM( alpha, beta, gama, Px, Py, Pz )
%UNTITLED Summary of this function goes here
%   绕X-Z-Y固定轴旋转表示位姿
alpha = alpha/180*pi;
beta = beta/180*pi;
gama = gama/180*pi;

R_X=[1 0 0;0 cos(gama) -sin(gama);0 sin(gama) cos(gama)];
R_Z=[cos(beta) -sin(beta) 0;sin(beta) cos(beta) 0;0 0 1];
R_Y=[cos(alpha) 0 sin(alpha);0 1 0;-sin(alpha) 0 cos(alpha)];
Z=[0 0 0];

R=[R_Y*R_Z*R_X;Z];

P=[Px Py Pz 1]';

T = [R,P];

end

