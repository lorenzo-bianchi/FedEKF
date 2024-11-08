function [bestR, bestT, inliers] = ransacRototranslation(A, B, numIterations, distanceThreshold, minInliers)
    bestInlierCount = 0;
    inliers = [];
    bestR = eye(2);
    bestT = [0; 0];

    combinations = nchoosek(1:size(A, 1), 3);

    for i = 1:size(combinations, 1)
        if i > numIterations
            break
        end
        indices = combinations(i, :);
        A_sample = A(indices, :);
        B_sample = B(indices, :);

        % Compute the transformation (R, T) using the 3 points
        [R, T] = icp2D(A_sample, B_sample);

        % Apply the transformation to all points in A
        A_transformed = (R * A' + T)';
        
        % Compute the distances between transformed A and B
        distances = sqrt(sum((A_transformed - B).^2, 2));
        
        % Count inliers
        inlierCount = length(find(distances < distanceThreshold));
        
        if inlierCount >= minInliers
            bestR = R;
            bestT = T;
            inliers = find(distances < distanceThreshold);
            return
        end
    end

    return
end