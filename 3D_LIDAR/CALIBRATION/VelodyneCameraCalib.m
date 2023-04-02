clear; close all; clc;

% YOLO image sjze : 620 x 480

lidarData  = pcread("../calibration/image,PCD/pointcloud.pcd");

Alpha = pi/2;  Beta = 0;   Gamma = 0;

camera_height = -0.2;
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

figure(1);
pcshow(lidarData);

yCoords = lidarData .Location(:,2);
negativeYIndices = find(yCoords < 0);
lidarData = select(lidarData, setdiff(1:size(lidarData.Location,1), negativeYIndices));
XYZ = lidarData.Location;

figure(2);
pcshow(lidarData);  title("Preprocess");

FOV_x = 48;
FOV_y = 42;

ox = imgWidth/2;
oy = imgHeight/2;
sx = focalLen * tand(0.5 * FOV_x)/(0.5 * imgWidth) ; 
sy = focalLen * tand(0.5 * FOV_y)/(0.5 * imgHeight) ; 

intMat = [focalLen/sx     0                   ox;
         0              focalLen/sy         oy;
         0                   0                   1 ];

x_im = zeros(length(XYZ),1);
y_im = zeros(length(XYZ),1);

for i = 1:length(XYZ)

    Cz = XYZ(i,2) + camera_recede;

    MAT =1/Cz * intMat * extMat * [XYZ(i,1);XYZ(i,2);XYZ(i,3); 1];

    x_im(i) = MAT(1);
    y_im(i) = MAT(2);

end

Projected = horzcat(x_im, y_im);

% rgb = insertShape(cam_image,'circle',[(img_width-x_im) (img_height-y_im) ones(length(XYZ),1)],'LineWidth',5,'Color', 'white');
rgb = insertShape(camImage,'circle',[(imgWidth - x_im) (y_im) ones(length(XYZ),1)],'LineWidth',5,'Color', 'white');
figure(3)
imshow(rgb)
title("Projected PCD data",FontSize = 14);
xlabel("pixel X",FontSize = 14); ylabel("pixel Y",FontSize = 14);







