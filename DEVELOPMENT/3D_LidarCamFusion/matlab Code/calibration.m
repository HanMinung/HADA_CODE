clear; close all; clc;

lidarData  = readmatrix("visualization.csv");

xCoor = lidarData(:,1);
yCoor = lidarData(:,2);
zCoor = lidarData(:,3);

Alpha = pi/2;  Beta = 0;   Gamma = 0;

camera_height = 0.2;
camera_recede = 0.0;

imgWidth  = 620;
imgHeight = 480;

camImage = zeros(imgHeight,imgWidth);
focalLen = 0.00367;

Rx = [1       0            0        ; 
      0  cos(Alpha)   -sin(Alpha)   ;
      0  sin(Alpha)   cos(Alpha)]   ;

Ry = [cos(Beta)     0       sin(Beta) ;
          0         1           0     ;
      -sin(Beta)    0       cos(Beta)];

Rz = [cos(Gamma)   -sin(Gamma)      0 ; 
      sin(Gamma)    cos(Gamma)      0 ;
            0           0           1];

rotMat = Rz * Ry * Rx;
% rotMat = Rx * Ry * Rz;

transMat = [0 ; camera_height ; camera_recede];

extMat = [rotMat transMat];

FOV_x = 62.92;
FOV_y = 51.3;

ox = imgWidth/2;
oy = imgHeight/2;
sx = focalLen * tand(0.5 * FOV_x)/(0.5 * imgWidth) ; 
sy = focalLen * tand(0.5 * FOV_y)/(0.5 * imgHeight) ; 

intMat = [focalLen/sx     0                 ox;
         0              focalLen/sy         oy;
         0                   0                   1 ];

pixelX = zeros(length(xCoor),1);
pixelY = zeros(length(xCoor),1);

for i = 1:length(xCoor)

    Cz = yCoor(i) + camera_recede;

    MAT =1/Cz * intMat * extMat * [xCoor(i);yCoor(i);zCoor(i); 1];

    pixelX(i) = MAT(1);
    pixelY(i) = MAT(2);

end

pixelX = pixelX(1:10028,:);
pixelY = pixelY(1:10028,:);

figure(1)
plot(640-pixelX, 480-pixelY, 'b.')
xlim([0 640]);
ylim([0 480]);

rgb = insertShape(camImage,'circle',[(imgWidth - pixelX) (pixelY) ones(length(pixelX),1)],'LineWidth',5,'Color', 'white');

figure(2)
imshow(rgb)
title("Projected PCD data",FontSize = 14);
xlabel("pixel X",FontSize = 14); ylabel("pixel Y",FontSize = 14);







