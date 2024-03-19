# Camera Calibration with DLT

## Introduction
This project focuses on calibrating a camera using Direct Linear Transformation (DLT) in MATLAB. It involves capturing an image of a calibration object, determining control points, and computing the camera's projection matrix to reconstruct the geometry of image formation.

## Methodology
- **Image Acquisition**: Captured an image of a calibration object with clearly defined control points.
- **Control Point Analysis**: Measured 3D object coordinates and their corresponding 2D image coordinates.
- **Projection Matrix Computation**: Implemented a MATLAB function for spatial resection using DLT and singular value decomposition (SVD).
- **Matrix Interpretation**: Derived interior and exterior orientation parameters through RQ-decomposition of the projection matrix.

## Results
- Successfully computed the camera's projection matrix, facilitating accurate 3D reconstruction from 2D image data.
- Interpreted the projection matrix to understand the camera's orientation and intrinsic properties, such as focal length and principal point.

## Technologies
- **MATLAB**: For computational tasks, including matrix operations and image processing.
