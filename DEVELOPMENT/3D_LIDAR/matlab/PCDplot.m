clear all; clc; close all;

ptCloud = pcread('../code/Velodyne-VLP-16-master/test/binfile_15_58_26/PCD_2023-03-02_1558/pcdindex_7.pcd');

THRESH = struct('xThresh_1', -1.2, ...
                'xThresh_2', 1.0, ...
                'yThresh_1', 1.0, ...
                'yThresh_2', 3.0, ...
                'zThresh', -0.3);

pcshow(ptCloud);

% SET ROI
indices = ptCloud.Location(:, 1) > THRESH.xThresh_1 & ptCloud.Location(:, 1) < THRESH.xThresh_2 & ptCloud.Location(:, 3) > THRESH.zThresh ... 
            & ptCloud.Location(:, 2) > THRESH.yThresh_1 & ptCloud.Location(:, 2) < THRESH.yThresh_2;

ptCloudCut = select(ptCloud, indices);

[model, inlierIndices, outlierIndices] = pcfitplane(ptCloudCut, 0.01);

inlierColors = repmat([0,1,0], length(inlierIndices), 1);
outlierColors = repmat([1,0,0], length(outlierIndices), 1);
subplot(1,2,1);
pcshow(ptCloud);
subplot(1,2,2);
pcshow(ptCloudCut.Location([inlierIndices; outlierIndices], :), [inlierColors; outlierColors]);
title('Inliers (green) and outliers (red)')


