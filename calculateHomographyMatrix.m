##  x_3d = [1, 2, 3, 4, 5, 6];
##  y_3d = [7, 8, 9, 10, 11, 12];
##  z_3d = [13, 14, 15, 16, 17, 18];
##
##  x_2d = [0.5, 1.5, 2.5, 3.5, 4.5, 5.5];
##  y_2d = [6.5, 7.5, 8.5, 9.5, 10.5, 11.5];


  function H = calculateHomographyMatrix(x_3d, y_3d, z_3d, x_2d, y_2d)
      % Calculates the Homography matrix using given 3D and 2D control points

      % Calculate mean of 3D control points
      mean_x_3d = mean(x_3d);
      mean_y_3d = mean(y_3d);
      mean_z_3d = mean(z_3d);

      mean_x_2d = mean(x_2d);
      mean_y_2d = mean(y_2d);

      % Initialize variables for scaling factors and means
      s_3d = zeros(6, 3);
      s_2d = zeros(6, 2);
      sum_s_3d = [0, 0, 0];
      sum_s_2d = [0, 0];

      % Calculate scaling factors and sums
      for i = 1:6
          s_3d(i, :) = [x_3d(i) - mean_x_3d, y_3d(i) - mean_y_3d, z_3d(i) - mean_z_3d];
          s_2d(i, :) = [x_2d(i) - mean_x_2d, y_2d(i) - mean_y_2d];

          sum_s_3d = sum_s_3d + abs(s_3d(i, :));
          sum_s_2d = sum_s_2d + abs(s_2d(i, :));
      end

      % Calculate scaling factors
      scaling_factor_3d = sum_s_3d / 6;
      scaling_factor_2d = sum_s_2d / 6;

      % Transformation matrices for scaling and translation
      Transform_3d = [1/scaling_factor_3d(1), 0, 0, 0;
                      0, 1/scaling_factor_3d(2), 0, 0;
                      0, 0, 1/scaling_factor_3d(3), 0;
                      0, 0, 0, 1] * [1, 0, 0, -mean_x_3d;
                                      0, 1, 0, -mean_y_3d;
                                      0, 0, 1, -mean_z_3d;
                                      0, 0, 0, 1];

      Transform_2d = [1/scaling_factor_2d(1), 0, 0;
                      0, 1/scaling_factor_2d(2), 0;
                      0, 0, 1] * [1, 0, -mean_x_2d;
                                  0, 1, -mean_y_2d;
                                  0, 0, 1];

      % Initialize matrices for calculations
      wxt_3d = [];
      wxt_2d = [];
##      ux = zeros(1, 6);
##      vx = zeros(1, 6);

      % Calculate transformed points and other necessary values
      for j = 1:6
          wxt_3d(:, j) = Transform_3d * [x_3d(j) y_3d(j)  z_3d(j) 1]';
          wxt_2d(:, j) = Transform_2d * [x_2d(j) y_2d(j) 1]';
          ux(:,j) = wxt_3d(:, j)' * wxt_2d(1, j);
          vx(:,j) = wxt_3d(:, j)' * wxt_2d(2, j);
      end

      % Construct the design matrix
      Ai = [];
      for i = 1:6
          Ati = [-wxt_3d(:, i)', zeros(1, 4), ux(:,i)';
                 zeros(1, 4), -wxt_3d(:, i)', vx(:,i)'];
          Ai = vertcat(Ai, Ati);
      end

      % Perform Singular Value Decomposition (SVD)
      [U, ~, V] = svd(Ai);

      % Extract homography matrix from the rightmost singular vector
      homography_matrix = reshape(V(:, end), 4, 3)';

      % Final transformation matrix
      H = inv(Transform_2d) * homography_matrix * Transform_3d;
      H = H / H(3, 4); % Normalize homography matrix
  end

