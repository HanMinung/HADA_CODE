clear all;close all; clc;

% %delta, los , yaw, eta,rt.simtime 
% data = readmatrix("2023-07-26_0143guidance_exp.csv");
% delcmd = data(:,1);
% los = data(:,2);
% yaw = data(:,3);
% eta = data(:,4);
% t = data(:,5);
% target = data(:,6:7);
% mypos = data(:,8:9);
% 
% figure()
% hold on;
% plot(t , delcmd );
% plot(t , los);
% plot(t , yaw );
% plot(t , eta );
% % result = 0;                                 
% 
%        % plot(target(:,1),target(:,2) ,mypos(:,1),mypos(:,2))
% 
% for i = 1:100:length(t)
% 
%     % plot(t(i),eta(i,1), 'k*' );
%     hold on;
%     plot(target(i,1),target(i,2) , 'k*--')
%     plot( mypos(i,1),mypos(i,2), 'b*--')
%     % drawnow
% %      grid on;
%     pause(0.05)
%     grid on;
% 
% end
%     legend('delta', 'los' , 'yaw', 'eta');
A=readmatrix("2023-09-08_1728navigation_exp.csv");
num=length(A);
lat=0;
lon=0;
for i=1:num
    lat=lat+A(i,1);
    lon=lon+A(i,2);
end

lat=lat/num;
lon=lon/num;
