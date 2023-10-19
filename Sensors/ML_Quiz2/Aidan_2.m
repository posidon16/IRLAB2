clear all;
clc;

% Intrinsic parameters
%image_resolution = [640, 480];
principal_point = [315, 242]; % Assuming the principal point is in the center of the image
focal_length = 900; % Focal length for both cameras

% Relative pose
translation_T = [0.15; 0; 0]; % Translation from right to left camera

left_image_point = [180, 242]; % (x, y) in the left image
right_image_point = [80, 242]; % (x, y) in the right image

% Calculate the baseline (distance between the camera centers)
baseline = norm(translation_T);

% Calculate the depth (Z) of the feature in the left camera coordinate frame
Z = (focal_length * baseline) / (left_image_point(1) - right_image_point(1));

% Calculate X and Y using the image coordinates and Z
X = (left_image_point(1) - principal_point(1)) * (Z / focal_length);
Y = (left_image_point(2) - principal_point(2)) * (Z / focal_length);

% Display the results
disp(['X = ' num2str(X)]);
disp(['Y = ' num2str(Y)]);
disp(['Z = ' num2str(Z)]);