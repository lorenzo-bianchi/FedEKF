function [bestR, bestT, inliers] = ransacRototranslation(A, B, numIterations, distanceThreshold, minInliers)
    numPoints = size(A, 1);
    bestInlierCount = 0;
    bestR = eye(2);
    bestT = [0; 0];

    for i = 1:numIterations
        % Randomly select 3 points
        indices = randperm(numPoints, 3);
        A_sample = A(indices, :);
        B_sample = B(indices, :);

        % Compute the transformation (R, T) using the 3 points
        [R, T] = icp2D(A_sample, B_sample);

        % Apply the transformation to all points in A
        A_transformed = (R * A' + T)';
        
        % Compute the distances between transformed A and B
        distances = sqrt(sum((A_transformed - B).^2, 2));
        
        % Count inliers
        inliers = find(distances < distanceThreshold);
        inlierCount = length(inliers);
        
        % Update the best transformation if current one is better
        if inlierCount > bestInlierCount && inlierCount >= minInliers
            bestInlierCount = inlierCount;
            bestR = R;
            bestT = T;
        end
    end

    % Final inliers based on best transformation
    A_transformed = (bestR * A' + bestT)';
    distances = sqrt(sum((A_transformed - B).^2, 2));
    inliers = find(distances < distanceThreshold);
end