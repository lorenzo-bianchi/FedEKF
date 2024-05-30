function [R, t] = icp2D(sourcePoints, targetPoints)
    % Compute the centroids of both point sets
    centroidSource = mean(sourcePoints, 1);
    centroidTarget = mean(targetPoints, 1);

    % Center the points by subtracting the centroids
    centeredSourcePoints = sourcePoints - centroidSource;
    centeredTargetPoints = targetPoints - centroidTarget;

    % Compute the covariance matrix
    H = centeredSourcePoints' * centeredTargetPoints;
    
    % Compute the SVD of the covariance matrix
    [U, ~, V] = svd(H);
    
    % Compute the rotation matrix
    R = V * U';
    
    % Ensure a proper rotation (det(R) should be 1)
    if det(R) < 0
        V(:, end) = -V(:, end);
        R = V * U';
    end

    % Compute the translation vector
    t = centroidTarget' - R * centroidSource';
end