function F = exp_myfun( x )

%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

Plane_Para = load('Plane_Para_Kinect2.txt');

load('pos_list');

pos_list(:, 5) = pos_list(:, 5) * 0.8;

for i = 1:1:60
    if Plane_Para(i,3) >0
        Plane_Para(i,:) = - Plane_Para(i,:);
    else
        Plane_Para(i,:) =  Plane_Para(i,:);
    end
end

for i =1:1:60
    T_RWS(i,:,:) = exp_TransM(pos_list(i,1),pos_list(i,2),pos_list(i,3),pos_list(i,4),pos_list(i,5) + 0.95+0.04, pos_list(i,6))';
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


 F1 = norm(a21*(cosd(x(1))*cosd(x(2)))+a22*(sind(x(2)))+a23*(-sind(x(1))*cosd(x(2)))-a_k);
 F2 = norm(a21*(-cosd(x(1))*sind(x(2))*cosd(x(3))+sind(x(1))*sind(x(3)))+a23*(sind(x(1))*sind(x(2))*cosd(x(3))+cosd(x(1))*sind(x(3)))+a22*(cosd(x(2))*cosd(x(3)))-b_k);
 F3 = norm(a21*(cosd(x(1))*sind(x(2))*sind(x(3))+sind(x(1))*cosd(x(3)))+a23*(-sind(x(1))*sind(x(2))*sind(x(3))+cosd(x(1))*cosd(x(3)))+a22*(-cosd(x(2))*sind(x(3)))-c_k);
%  F4 = norm(a21*x(1)+a22*x(2)+a23*x(3)-d_k);
%  F4 = a21*x(1)+a22*x(2)+a23*x(3)-d_k;


F=F1+F2+F3;
% F=F4;

end