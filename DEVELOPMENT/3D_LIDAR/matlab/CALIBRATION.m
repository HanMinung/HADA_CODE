clear all; clc; close all;

addpath('../calibration/image,PCD/');

Position = struct('height', 0.78, ...
                  'recede', 0.56 );

Camera = struct('imgWidth', 1280   , ...
                'imgHeight', 720   , ...
                'fclen',    0.00367, ...
                'FOVx',     70.42  , ...
                'FOVy',     43.30  , ...
                'cx',       640    ,...
                'cy',       360    );

sx = Camera.fclen * tand(0.5 * Camera.FOVx)/(0.5 * Camera.imgWidth) ; 
sy = Camera.fclen * tand(0.5 * Camera.FOVy)/(0.5 * Camera.imgHeight) ;

Mint = [Camera.fclen/sx           0                   Camera.cx;
          0                   Camera.fclen/sy         Camera.cy;
          0                       0                     1];

M_ext = [-1     0    0         0         ;
         0      0    1   Position.height ;
         0      1    0   Position.recede];

PCD = pcread("pointcloud.pcd");
PCD = PCD.Location;

Len = length(PCD);