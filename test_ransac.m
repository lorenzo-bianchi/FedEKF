% robot1 = 1;
% robot2 = 2;
% 
% tag1 = [ekfs(robot1).xHatTagStoria(:, end) ekfs(robot1).yHatTagStoria(:, end)];
% tag2 = [ekfs(robot2).xHatTagStoria(:, end) ekfs(robot2).yHatTagStoria(:, end)];
% % tag1(1, :) = tag1(1, :)*-0.7 -3;
% % tag1(2, :) = tag1(2, :)*1.8;
% 
% [R, t] = icp2D(tag1, tag2);
% 
% tag1Transformed = (R * tag1' + t)';
% figure;
% plot(tag1Transformed(:,1), tag1Transformed(:,2), 'r+', 'DisplayName', 'tag1Transformed', 'MarkerSize', 7, 'LineWidth', 2);
% hold on;
% plot(tag2(:,1), tag2(:,2), 'kx', 'DisplayName', 'tag2', 'MarkerSize', 7, 'LineWidth', 2);
% legend;
% % title(sprintf('Tags alignment robot %d', robot));
% xlabel('x [m]');
% ylabel('y [m]');
% axis equal;
% grid on;

%%
robot1 = 1;
robot2 = 2;

tag1 = [ekfs(robot1).xHatTagStoria(:, end) ekfs(robot1).yHatTagStoria(:, end)];
tag2 = [ekfs(robot2).xHatTagStoria(:, end) ekfs(robot2).yHatTagStoria(:, end)];
A = tag1;
B = tag2;
A(1, :) = A(1, :)*-0.7 -3;
A(2, :) = A(2, :)*1.8;

numIterations = 20;
distanceThreshold = 0.2;
minInliers = round(0.6 * size(A, 1));

[bestR, bestT, inliers] = ransacRototranslation(A, B, numIterations, distanceThreshold, minInliers);
if isempty(inliers)
    disp("No inliers")
    return
end

% Transform points A using the best rototranslation
A_transformed = (bestR * A' + bestT)';

figure;
plot(A_transformed(:,1), A_transformed(:,2), 'r+', 'DisplayName', 'Atransformed', 'MarkerSize', 7, 'LineWidth', 2);
hold on;
plot(B(:,1), B(:,2), 'kx', 'DisplayName', 'B', 'MarkerSize', 7, 'LineWidth', 2);
legend;
% title(sprintf('Tags alignment robot %d', robot));
xlabel('x [m]');
ylabel('y [m]');
axis equal;
grid on;