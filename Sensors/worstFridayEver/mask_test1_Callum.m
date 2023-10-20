function [zRed, zBlue, zGreen, xRed,xBlue, xGreen] = mask_test1_Callum

I = rossubscriber("/camera/color/image_raw");%imread('IMG_4934.jpg');
RGB_Image = readImage(I.receive());
pause(5)

redBrick = createMaskRedBrick(RGB_Image);
blueBrick = createMaskBlueBrick(RGB_Image);
greenBrick = createMaskGreenBrick(RGB_Image);
figure(1)
imshow(redBrick);


figure(2)
imshow(greenBrick);


figure(3)
imshow(blueBrick);

stats = regionprops(redBrick,"Centroid", "Area");
bats = regionprops(greenBrick,"Centroid", "Area");
mats = regionprops(blueBrick,"Centroid", "Area");

redArea = [stats.Area];
greenArea = [bats.Area];
blueArea = [mats.Area];

[redMaxArea, redIndex] = max(redArea);
[greenMaxArea, greenIndex] = max(greenArea);
[blueMaxArea, blueIndex] = max(blueArea);

redCentroid = stats(redIndex).Centroid;
greenCentroid = bats(greenIndex).Centroid;
blueCentroid = mats(blueIndex).Centroid;




%% run

depthSub = rossubscriber('/camera/aligned_depth_to_color/image_raw');
depthMsg = readImage(depthSub.receive());
pause(5);

px = 500;  % distance from robot outwards
% Principlan x (From the dobot, how highj the cam is)
py = 0; % distance left or right

zRed = 0;
zBlue = 0;
zGreen = 0;
xRed = 0;
xBlue = 0;
xGreen = 0;



fx = 606.8311157226562
fy = 606.0000610351562

ixRed = double(redCentroid(1,1)) 
iyRed = double(redCentroid(1,2))


zRed = double(depthMsg(redIndex)/ 2.64583)
xRed = (ixRed - px) * zRed / fx
yRed = (iyRed - py) * zRed / fy

ixGreen = double(greenCentroid(1,1))
iyGreen = double(greenCentroid(1,2))


zGreen = double(depthMsg(greenIndex)/2.64583)
xGreen = (ixGreen - px) * zGreen / fx
yGreen = (iyGreen - py) * zGreen / fy

ixBlue = double(blueCentroid(1,1)) 
iyBlue = double(blueCentroid(1,2))


zBlue = double(depthMsg(blueIndex)/2.64583)
xBlue = (ixBlue - px) * zBlue / fx
yBlue = (iyBlue - py) * zBlue / fy

zRed = zRed/10000
zBlue = zBlue/10000
zGreen = zGreen/10000

zRed = zRed + 0.08
zBlue = zBlue +0.08
zGreen = zGreen + 0.08


xRed = xRed/10000 + 0.035
xBlue = xBlue/10000 + 0.035
xGreen = xGreen/10000 + 0.035

xRed = xRed
xBlue = xBlue
xGreen = xGreen

% J = imcrop(I,[600 200 700 750]);


% 
% J = imcrop(I,[450 450 700 1000]);
% imshow(J)
% I = rgb2gray(J);
% 
% cornerPoints = detectHarrisFeatures(I, "MinQuality", 0.04)
% imshow(I)
% hold on
% plot(cornerPoints)
% cornerPoints.Location

end