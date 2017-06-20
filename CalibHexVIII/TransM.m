function T = TransM( alpha, beta, gama, Px, Py, Pz )
%UNTITLED Summary of this function goes here
%   绕X-Y-Z固定轴旋转表示位姿
alpha = alpha/180*pi;
beta = beta/180*pi;
gama = gama/180*pi;

R_Z=[cos(alpha) -sin(alpha) 0;sin(alpha) cos(alpha) 0;0 0 1];
R_Y=[cos(beta) 0 sin(beta);0 1 0;-sin(beta) 0 cos(beta)];
R_X=[1 0 0;0 cos(gama) -sin(gama);0 sin(gama) cos(gama)];
Z=[0 0 0];

R=[R_Z*R_Y*R_X;Z];

P=[Px Py Pz 1]';

T = [R,P];

end

