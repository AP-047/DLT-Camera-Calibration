clc;
clear all;
clear;
##Start
##|
##---> Read image and display it
##|   ---> Read image file
##|   ---> Create figure
##|   ---> Display image
##|
##---> Get six pixel coordinates from the image
##|   ---> Use ginput function to collect six pairs of pixel coordinates
##|
##---> Convert pixel coordinates to 2D homogeneous coordinates
##|   ---> Reshape x-coordinates into a row vector
##|   ---> Reshape y-coordinates into a row vector
##|   ---> Combine coordinates into a homogeneous matrix
##|
##---> Define 3D coordinates of control points
##|   ---> Define 3D coordinates of six control points
##|
##---> Calculate homography matrix using 3D and 2D coordinates
##|   ---> Use Homography_cond function to calculate homography matrix
##|
##---> Perform normalization for decomposition
##|   ---> Extract rotational and translational components of the homography matrix
##|   ---> Calculate lambda for normalization
##|   ---> Normalize rotational and translational components
##|
##---> Perform RQD using QR decomposition
##|   ---> Perform QR decomposition
##|   ---> Invert the rotation matrix
##|   ---> Invert the intrinsic camera matrix
##|
##---> Ensure positive diagonals in K for a unique solution
##|   ---> Check if any diagonal element is negative
##|   ---> Negate the corresponding column and row of K and R to maintain consistency
##|
##---> Extract camera parameters from K and R
##|   ---> Extract principal point
##|   ---> Extract skew factor
##|   ---> Extract aspect ratio
##|   ---> Extract principal distance
##|
##---> Extract projection center using singular value decomposition
##|   ---> Perform singular value decomposition of the homography matrix
##|   ---> Extract projection center from the rightmost singular vector
##|   ---> Reshape the projection center into a row vector
##|
##---> Calculate extrinsic parameters
##|   ---> Calculate rotation angle around the optical axis
##|   ---> Calculate rotation angle around the x-axis
##|   ---> Calculate rotation angle around the y-axis
##|   ---> Calculate skew factor
##|
##---> Stop

% Read image and display it
image = imread('3dobject.jpg');  % Reading image file
figure(1);             % Creating a figure
imshow(image);          % Displaying the image

% Get six pixel coordinates from the image
[xCoordinates, yCoordinates] = ginput(6);  % Collecting six pairs of pixel coordinates

% Convert pixel coordinates to 2D homogeneous coordinates
xCoordinatesVector = reshape(xCoordinates, 1, []);  % Reshaping x-coordinates into a row vector
yCoordinatesVector = reshape(yCoordinates, 1, []);  % Reshaping y-coordinates into a row vector
twoDPoints = [xCoordinatesVector; yCoordinatesVector; ones(1, 6)];  % Combining coordinates into a homogeneous matrix

% Define the 3D coordinates of control points
controlPoints3D = defineControlPoints();


% Calculate homography matrix using 3D and 2D coordinates
homographyMatrix = calculateHomographyMatrix(controlPoints3D(1,:), controlPoints3D(2,:), controlPoints3D(3,:), twoDPoints(1,:), twoDPoints(2,:));  % Calculating homography matrix

% Perform normalization for decomposition
rotationalAndTranslationalComponents = homographyMatrix(:, 1:3);  % Extracting rotational and translational components of the homography matrix
normalizationFactor = sign(det(rotationalAndTranslationalComponents)) / norm(rotationalAndTranslationalComponents(3,:));  % Calculating normalization factor

normalizedComponents = rotationalAndTranslationalComponents * normalizationFactor;  % Normalizing rotational and translational components



% Perform RQD using QR decomposition
[rotationMatrix, intrinsicCameraMatrix] = performRQD(normalizedComponents)

% Ensure positive diagonals in K for a unique solution
for i = 1:3
    if intrinsicCameraMatrix(i, i) < 0  % Checking if any diagonal element is negative
        intrinsicCameraMatrix(:, i) = -intrinsicCameraMatrix(:, i);   % Negating the corresponding column and row of K and R to maintain consistency
        rotationMatrix(i,:) = -rotationMatrix(i,:);
    end
end

display(rotationMatrix);  % Displaying the rotation matrix
display(intrinsicCameraMatrix);  % Displaying the intrinsic camera matrix

% Extract camera parameters from K and R
principalPoint = [intrinsicCameraMatrix(1, 3), intrinsicCameraMatrix(2, 3)];  % Extracting principal point
skewFactor = intrinsicCameraMatrix(1, 2);  % Extracting skew factor
aspectRatio = intrinsicCameraMatrix(2, 2) / intrinsicCameraMatrix(1, 1);  % Extracting aspect ratio
principalDistance = intrinsicCameraMatrix(1, 1);  % Extracting principal distance

% Extract projection center using singular value decomposition
[matrix_U, singularValues, matrix_V] = svd(homographyMatrix);  % Performing singular value decomposition of the homography matrix
projectionCenter = matrix_V(:, end);
projectionCenter = projectionCenter / projectionCenter(end, end);  % Extracting projection center from the rightmost singular vector
projectionCenter = reshape(projectionCenter, 1, 4);  % Reshaping the projection center into a row vector

% Calculate extrinsic parameters
omega = atan2(rotationMatrix(3, 2), rotationMatrix(3, 3)) * 180 / pi;  % Calculating rotation angle around the optical axis
phi = -asin(rotationMatrix(3, 1)) * 180 / pi;  % Calculating rotation angle around the x-axis
kappa = atan2(rotationMatrix(2, 1), rotationMatrix(1, 1)) * 180 / pi;  % Calculating rotation angle around the y-axis

skewFactor = acot(-intrinsicCameraMatrix(1, 2) / intrinsicCameraMatrix(1, 1)) * 180 / pi;  % Calculating skew factor

