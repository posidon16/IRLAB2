clear all
clc
clf

I = imread('IMG_4934.jpg');
redBrick = createMaskRedBrick(I);
blueBrick = createMaskBlueBrick(I);
greenBrick = createMaskGreenBrick(I);
imshow(redBrick);
imshow(greenBrick);
imshow(blueBrick);

stats = regionprops(redBrick,"Centroid", "Area");
bats = regionprops(greenBrick,"Centroid", "Area");
mats = regionprops(blueBrick,"Centroid", "Area");

redArea = [stats.Area];
greenArea = [bats.Area];
blueArea = [mats.Area];

[redMaxArea, redIndex] = max(redArea)
[greenMaxArea, greenIndex] = max(greenArea)
[blueMaxArea, blueIndex] = max(blueArea)

redCentroid = stats(redIndex).Centroid;
greenCentroid = bats(greenIndex).Centroid;
blueCentroid = mats(blueIndex).Centroid;
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