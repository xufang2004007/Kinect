function [Plain, d] = Plain_Fit( X,Y,Z )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
x=X;
y=Y;
z=Z;

x_bar=mean(x);
y_bar=mean(y);
z_bar=mean(z);

A=zeros(3,3);
for i3=1:1:size(x)
    A(1,1)=(x(i3)-x_bar)*(x(i3)-x_bar)+A(1,1);
    A(1,2)=(x(i3)-x_bar)*(y(i3)-y_bar)+A(1,2);
    A(1,3)=(x(i3)-x_bar)*(z(i3)-z_bar)+A(1,3);
    A(2,1)=(y(i3)-y_bar)*(x(i3)-x_bar)+A(2,1);
    A(2,2)=(y(i3)-y_bar)*(y(i3)-y_bar)+A(2,2);
    A(2,3)=(y(i3)-y_bar)*(z(i3)-z_bar)+A(2,3);
    A(3,1)=(z(i3)-z_bar)*(x(i3)-x_bar)+A(3,1);
    A(3,2)=(z(i3)-z_bar)*(y(i3)-y_bar)+A(3,2);
    A(3,3)=(z(i3)-z_bar)*(z(i3)-z_bar)+A(3,3);
end
[V,D]=eig(A);
a=V(1,1);
b=V(2,1);
c=V(3,1);
d_Plain=-a*x_bar-b*y_bar-c*z_bar;
Plain(1,1)=a;
Plain(2,1)=b;
Plain(3,1)=c;
Plain(4,1)=d_Plain;
%%
for i4=1:1:size(x)
    d(i4)=a*(x(i4)-x_bar)+b*(y(i4)-y_bar)+c*(z(i4)-z_bar);
end
d=d';
Plain(5,1)=std(d);

end

