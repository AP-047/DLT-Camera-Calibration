% Function to perform RQD using QR decomposition
function [rotationMatrix, intrinsicCameraMatrix] = performRQD(normalizedMatrix)
    [rotationMatrix, intrinsicCameraMatrix] = qr(inv(normalizedMatrix));
    rotationMatrix = inv(rotationMatrix);
    intrinsicCameraMatrix = inv(intrinsicCameraMatrix);
end
