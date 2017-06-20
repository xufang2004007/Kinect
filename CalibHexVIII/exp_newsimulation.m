%5 -27 1
clear all;
options = optimoptions(@lsqnonlin, 'Algorithm', 'levenberg-marquardt');
% options = optimoptions(@lsqnonlin, 'Algorithm', 'trust-region-reflective');
options.TolFun = 1e-20;
options.TolX = 1e-100;
options.MaxFunEvals = 1e18;
options.MaxIter = 1e20;
x0 = [5 -27 1];
% x0 = [66 66 66];
% x0 = [66 66 3];
% x0 = [ 0.400, 0.150, 0.02];
% [x, fval] = fminunc(@myfun, x0)
%[x resnom] = lsqnonlin(@New_expi,x0,[ ],[ ],options)
[x resnom] = lsqnonlin(@Auto_Calib_Plane_Para,x0,[ ],[ ],options)
save('AngForVelodyne.mat', 'x');
%%
abs(x-x0)