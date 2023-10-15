clear; close all; clc;
A=readmatrix("2023-08-22_2053navigation_exp");


geobasemap("satellite"); hold on;
R2D=180/pi;
geoplot(A(:,1),A(:,2),"b*","LineWidth",4.0);         hold on;
geoplot(A(:,3)*R2D,A(:,4)*R2D,'r-',"LineWidth",2.5);
legend("GPS","EKF")