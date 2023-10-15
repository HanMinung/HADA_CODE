clear all; clc; close all;

ptCloud = pcread('../code/Velodyne-VLP-16-master/test/binfile_19_59_30/PCD_2023-02-27_1959/pcdindex_10.pcd');

% Set the threshold to a suitable value
xThresh_1 = -1.6;
xThresh_2 = 0;
yThresh_1 = 1;
yThresh_2 = 3;
zThresh = -0.2;

indices = ptCloud.Location(:, 1) > xThresh_1 & ptCloud.Location(:, 1) < xThresh_2 & ptCloud.Location(:, 3) > zThresh ... 
            & ptCloud.Location(:, 2) > yThresh_1 & ptCloud.Location(:, 2) < yThresh_2;

ptCloudCut = select(ptCloud, indices);

[model, inlierIndices, outlierIndices] = pcfitplane(ptCloudCut, 0.01);

inlierColors = repmat([0,1,0], length(inlierIndices), 1);
outlierColors = repmat([1,0,0], length(outlierIndices), 1);
subplot(1,2,1);
pcshow(ptCloud);
subplot(1,2,2);
pcshow(ptCloudCut.Location([inlierIndices; outlierIndices], :), [inlierColors; outlierColors]);
title('Inliers (green) and outliers (red)')
