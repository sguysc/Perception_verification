%%
close all
clear all
clc

m = 1;
dt = 1;

A = [0 1; 0 0];
B = [0; 1/m];
C = eye(2);
D = [0;0];

sys = ss(A,B,C,D);
sysd= c2d(sys, dt);

% estimator
p = [-2*pi*1*cosd(45)+1i*2*pi*1*sind(45), -2*pi*1*cosd(45)-1i*2*pi*1*sind(45)];
L = place(sysd.A',sysd.C',p).';

% poles for the closed loop system
p = [-2*pi*1*cosd(45)+1i*2*pi*1*sind(45), -2*pi*1*cosd(45)-1i*2*pi*1*sind(45)];

[Kd, S, e] = lqrd(sysd.A, sysd.B, eye(2), 1, 1);

%%
noise_y = [ 2.76585773e-04  5.16444670e-03 -4.49877762e-03 -2.77545348e-02 ...
   4.18161432e-02  1.39356380e-01 -3.22666968e-01 -6.36898290e-01 ...
   2.24556766e+00  2.48142612e+00];
noise_ydot = [ 6.39051462e-07  1.19324547e-05 -1.03944262e-05 -6.41268560e-05 ...
   9.66162037e-05  3.21982933e-04 -7.45522067e-04 -1.47155357e-03 ...
   5.18838435e-03  5.73333535e-03];
