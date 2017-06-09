% Intrinsic and Extrinsic Camera Parameters
%
% This script file can be directly executed under Matlab to recover the camera intrinsic and extrinsic parameters.
% IMPORTANT: This file contains neither the structure of the calibration objects nor the image coordinates of the calibration points.
%            All those complementary variables are saved in the complete matlab data file Calib_Results.mat.
% For more information regarding the calibration model visit http://www.vision.caltech.edu/bouguetj/calib_doc/


%-- Focal length:
fc = [ 520.503235582373920 ; 518.952775970883290 ];

%-- Principal point:
cc = [ 322.763693274491230 ; 236.644790702156710 ];

%-- Skew coefficient:
alpha_c = 0.000000000000000;

%-- Distortion coefficients:
kc = [ 0.022290459306606 ; -0.130268071444498 ; 0.000619145616595 ; -0.001256207202551 ; 0.000000000000000 ];

%-- Focal length uncertainty:
fc_error = [ 1.727352012516446 ; 1.591459989446109 ];

%-- Principal point uncertainty:
cc_error = [ 2.652234999082214 ; 2.189845016039147 ];

%-- Skew coefficient uncertainty:
alpha_c_error = 0.000000000000000;

%-- Distortion coefficients uncertainty:
kc_error = [ 0.011348751883930 ; 0.032324479104574 ; 0.001437604738150 ; 0.001601519388452 ; 0.000000000000000 ];

%-- Image size:
nx = 640;
ny = 480;


%-- Various other variables (may be ignored if you do not use the Matlab Calibration Toolbox):
%-- Those variables are used to control which intrinsic parameters should be optimized

n_ima = 20;						% Number of calibration images
est_fc = [ 1 ; 1 ];					% Estimation indicator of the two focal variables
est_aspect_ratio = 1;				% Estimation indicator of the aspect ratio fc(2)/fc(1)
center_optim = 1;					% Estimation indicator of the principal point
est_alpha = 0;						% Estimation indicator of the skew coefficient
est_dist = [ 1 ; 1 ; 1 ; 1 ; 0 ];	% Estimation indicator of the distortion coefficients


%-- Extrinsic parameters:
%-- The rotation (omc_kk) and the translation (Tc_kk) vectors for every calibration image and their uncertainties

%-- Image #1:
omc_1 = [ -2.159691e+00 ; -2.240327e+00 ; -1.383655e-01 ];
Tc_1  = [ -1.085705e+02 ; -1.502415e+00 ; 3.520541e+02 ];
omc_error_1 = [ 6.124347e-03 ; 7.230980e-03 ; 1.363503e-02 ];
Tc_error_1  = [ 1.805990e+00 ; 1.525816e+00 ; 1.602937e+00 ];

%-- Image #2:
omc_2 = [ -2.204071e+00 ; -2.172735e+00 ; -3.938698e-02 ];
Tc_2  = [ -1.244042e+02 ; -5.516684e+01 ; 5.047512e+02 ];
omc_error_2 = [ 1.306018e-02 ; 1.246709e-02 ; 2.759638e-02 ];
Tc_error_2  = [ 2.586631e+00 ; 2.166660e+00 ; 2.497432e+00 ];

%-- Image #3:
omc_3 = [ 1.981524e+00 ; 1.844593e+00 ; 7.013883e-01 ];
Tc_3  = [ -1.169380e+02 ; -5.520620e+01 ; 2.655055e+02 ];
omc_error_3 = [ 4.992704e-03 ; 4.455385e-03 ; 8.775572e-03 ];
Tc_error_3  = [ 1.448063e+00 ; 1.185139e+00 ; 1.241978e+00 ];

%-- Image #4:
omc_4 = [ -2.159098e+00 ; -2.191647e+00 ; -2.808750e-01 ];
Tc_4  = [ -1.177917e+02 ; -6.841643e+01 ; 2.813970e+02 ];
omc_error_4 = [ 5.705824e-03 ; 5.384627e-03 ; 1.176662e-02 ];
Tc_error_4  = [ 1.475283e+00 ; 1.254065e+00 ; 1.299684e+00 ];

%-- Image #5:
omc_5 = [ -1.940570e+00 ; -1.890383e+00 ; 2.858252e-01 ];
Tc_5  = [ -9.872859e+01 ; -6.625676e+01 ; 3.230489e+02 ];
omc_error_5 = [ 4.471560e-03 ; 4.236668e-03 ; 7.909650e-03 ];
Tc_error_5  = [ 1.641450e+00 ; 1.380706e+00 ; 1.072854e+00 ];

%-- Image #6:
omc_6 = [ -2.281497e+00 ; -2.003693e+00 ; -8.750573e-02 ];
Tc_6  = [ -2.028832e+02 ; -6.516442e+01 ; 6.786099e+02 ];
omc_error_6 = [ 1.673086e-02 ; 1.307508e-02 ; 3.231071e-02 ];
Tc_error_6  = [ 3.492726e+00 ; 2.936800e+00 ; 3.204093e+00 ];

%-- Image #7:
omc_7 = [ -1.745329e+00 ; -1.828494e+00 ; -5.664863e-01 ];
Tc_7  = [ -1.762445e+02 ; -1.746012e+02 ; 6.640834e+02 ];
omc_error_7 = [ 7.533597e-03 ; 7.812629e-03 ; 1.340850e-02 ];
Tc_error_7  = [ 3.462466e+00 ; 2.965488e+00 ; 3.232652e+00 ];

%-- Image #8:
omc_8 = [ -2.145775e+00 ; -2.064718e+00 ; 6.796238e-02 ];
Tc_8  = [ -2.509674e+02 ; -1.510781e+02 ; 1.010213e+03 ];
omc_error_8 = [ 1.883455e-02 ; 1.536503e-02 ; 3.561345e-02 ];
Tc_error_8  = [ 5.328282e+00 ; 4.371663e+00 ; 5.509799e+00 ];

%-- Image #9:
omc_9 = [ 1.946904e+00 ; 1.729456e+00 ; 7.742590e-01 ];
Tc_9  = [ -2.302256e+02 ; -1.736350e+02 ; 9.249209e+02 ];
omc_error_9 = [ 9.793615e-03 ; 1.135622e-02 ; 1.874625e-02 ];
Tc_error_9  = [ 5.085541e+00 ; 4.078909e+00 ; 7.070836e+00 ];

%-- Image #10:
omc_10 = [ -2.045235e+00 ; -1.778310e+00 ; 3.432452e-01 ];
Tc_10  = [ -1.728215e+02 ; -1.954258e+02 ; 1.100488e+03 ];
omc_error_10 = [ 1.259959e-02 ; 1.027468e-02 ; 2.116844e-02 ];
Tc_error_10  = [ 5.762810e+00 ; 4.760849e+00 ; 6.675107e+00 ];

%-- Image #11:
omc_11 = [ -2.177351e+00 ; -2.231929e+00 ; -8.412559e-02 ];
Tc_11  = [ -8.068514e+01 ; -6.933880e+01 ; 2.210393e+02 ];
omc_error_11 = [ 3.946609e-03 ; 4.198050e-03 ; 8.763431e-03 ];
Tc_error_11  = [ 1.155971e+00 ; 9.644626e-01 ; 8.892417e-01 ];

%-- Image #12:
omc_12 = [ 2.012523e+00 ; 1.935004e+00 ; 6.500828e-01 ];
Tc_12  = [ -5.939805e+01 ; -6.462422e+01 ; 1.659532e+02 ];
omc_error_12 = [ 4.435816e-03 ; 3.539028e-03 ; 7.090827e-03 ];
Tc_error_12  = [ 8.938439e-01 ; 7.246716e-01 ; 7.023365e-01 ];

%-- Image #13:
omc_13 = [ -1.822099e+00 ; -1.814453e+00 ; 3.656667e-01 ];
Tc_13  = [ -7.866160e+01 ; -7.856989e+01 ; 2.565212e+02 ];
omc_error_13 = [ 3.926807e-03 ; 3.556568e-03 ; 6.294460e-03 ];
Tc_error_13  = [ 1.312728e+00 ; 1.087355e+00 ; 8.054608e-01 ];

%-- Image #14:
omc_14 = [ -1.855258e+00 ; -1.986657e+00 ; -2.620315e-01 ];
Tc_14  = [ -9.440897e+01 ; -6.726663e+01 ; 2.203939e+02 ];
omc_error_14 = [ 3.356530e-03 ; 4.220273e-03 ; 7.121694e-03 ];
Tc_error_14  = [ 1.144457e+00 ; 9.774108e-01 ; 9.252495e-01 ];

%-- Image #15:
omc_15 = [ 2.000551e+00 ; 2.013451e+00 ; -2.912203e-01 ];
Tc_15  = [ -8.737750e+01 ; -8.378413e+01 ; 2.349016e+02 ];
omc_error_15 = [ 3.233310e-03 ; 4.434844e-03 ; 7.408715e-03 ];
Tc_error_15  = [ 1.218309e+00 ; 1.006918e+00 ; 8.720090e-01 ];

%-- Image #16:
omc_16 = [ -1.810460e+00 ; -1.960542e+00 ; 5.117337e-01 ];
Tc_16  = [ -4.973283e+01 ; -9.864969e+01 ; 3.472056e+02 ];
omc_error_16 = [ 4.701706e-03 ; 4.341405e-03 ; 7.956574e-03 ];
Tc_error_16  = [ 1.794030e+00 ; 1.458545e+00 ; 1.023397e+00 ];

%-- Image #17:
omc_17 = [ 1.795326e+00 ; 1.672670e+00 ; 1.029715e+00 ];
Tc_17  = [ -6.253987e-01 ; -7.976579e+01 ; 1.753301e+02 ];
omc_error_17 = [ 4.998126e-03 ; 4.076376e-03 ; 6.012362e-03 ];
Tc_error_17  = [ 9.448731e-01 ; 7.465530e-01 ; 7.934121e-01 ];

%-- Image #18:
omc_18 = [ -1.422137e+00 ; -1.472171e+00 ; -1.046352e+00 ];
Tc_18  = [ -9.841745e+01 ; -8.468607e+01 ; 2.333435e+02 ];
omc_error_18 = [ 3.610015e-03 ; 4.658167e-03 ; 5.593955e-03 ];
Tc_error_18  = [ 1.241845e+00 ; 1.075573e+00 ; 1.066882e+00 ];

%-- Image #19:
omc_19 = [ -1.067398e+00 ; -1.609840e+00 ; 7.615780e-01 ];
Tc_19  = [ 4.148126e+01 ; -1.251772e+02 ; 3.292052e+02 ];
omc_error_19 = [ 4.079445e-03 ; 4.374527e-03 ; 4.835707e-03 ];
Tc_error_19  = [ 1.749076e+00 ; 1.388515e+00 ; 9.458093e-01 ];

%-- Image #20:
omc_20 = [ 1.743267e+00 ; 1.484761e+00 ; 1.093824e+00 ];
Tc_20  = [ -2.472837e+01 ; -5.619526e+01 ; 1.375539e+02 ];
omc_error_20 = [ 4.925821e-03 ; 3.746430e-03 ; 5.639917e-03 ];
Tc_error_20  = [ 7.368529e-01 ; 5.881965e-01 ; 6.231015e-01 ];

