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
noise_y = [-2.12132033  2.12132034  2.12132034];
noise_ydot = [-0.01826362  0.0644068   0.07118097];
