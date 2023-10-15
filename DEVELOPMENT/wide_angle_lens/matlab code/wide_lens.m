
I = imread('../raw_images/WIN_20230707_21_16_09_Pro.jpg');

fisheyeIntrinsics = cameraParams.Intrinsics;

undistortedImage = undistortFisheyeImage(I, fisheyeIntrinsics);

figure(1)
imshow(I)

figure(2)
imshow(undistortedImage)
