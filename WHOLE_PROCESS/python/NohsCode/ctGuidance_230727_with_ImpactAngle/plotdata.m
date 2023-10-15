clear all;close all; clc;

%delta, los , yaw, eta,rt.simtime 
data = readmatrix("2023-07-27_1726guidance_exp.csv");
delcmd = data(:,1);
los = data(:,2);
yaw = data(:,3);
eta = data(:,4);
t = data(:,5);
target = data(:,6:7);
mypos = data(:,8:9);

figure()
hold on;
plot(t , delcmd );
plot(t , los);
% plot(t , yaw );
plot(t , eta );
grid on;
result = 0;                                 

       % plot(target(:,1),target(:,2) ,mypos(:,1),mypos(:,2))

% for i = 1:1000:length(t)
% 
%     % plot(t(i),eta(i,1), 'k*' );
%     hold on;
%     plot(target(i,1),target(i,2) , 'k*--')
%     plot( mypos(i,1),mypos(i,2), 'b*--')
%     % drawnow
% %      grid on;
%     % pause(0.05)
%     grid on;
% 
% end
    legend('delta', 'los' , 'yaw', 'eta');



   