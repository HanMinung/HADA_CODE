clear all; close all; clc;

data = readmatrix("갈상가이던스테스트.csv");
data2 = readmatrix("갈태2.csv");
LAT2METER = 110950.59672489;
LON2METER =   5159243.427952315 * pi / 180; 
geobasemap("satellite");hold on;
% geoplot(data(:,2)/LAT2METER + 37.2 ,data(:,1)/LON2METER + 126.7,"b-")%,data(:,4)/LAT2METER + 37.2,data(:,3)/LON2METER + 126.7,"k*")
geoplot(data(:,1), data(:,2) , "k*");hold on; geoplot(data2(:,1), data2(:,2) , "b*")