%%
figure
axis equal
hold on
grid on

plot(thisRobotTags(:, 1), thisRobotTags(:, 2), 'bo', 'LineWidth', 2)

plot(posTagRobot(1, :, 1), posTagRobot(2, :, 1), 'ro', 'LineWidth', 2)

%%
[RR, tt, inl] = ransacRototranslation(otherRobotTags', thisRobotTags, ...
                                      obj.data.numIterations, ...
                                      0.25, ...
                                      5)
TT = [RR, tt; zeros(1, 2), 1];
temp = TT*[otherRobotTags; ones(1, nTag)];

figure
axis equal
hold on
grid on

plot(thisRobotTags(:, 1), thisRobotTags(:, 2), 'bo', 'LineWidth', 2)
plot(temp(1, :), temp(2, :), 'ro', 'LineWidth', 2)

%%
robot = 4;
robot_tags = [ekfs(robot).xHatTagStoria(:, end)'; ekfs(robot).yHatTagStoria(:, end)']';
[RR, tt, inl] = ransacRototranslation(robot_tags, cTag, ...
                                      data.numIterations, ...
                                      0.8, ...
                                      3)
TT = [RR, tt; zeros(1, 2), 1];
temp = TT*[robot_tags'; ones(1, nTag)];
temp = temp(1:2, :);

mean_rotated = mean(vecnorm(cTag' - temp))

figure
axis equal
hold on
grid on

plot(cTag(:, 1), cTag(:, 2), 'bo', 'LineWidth', 2)
plot(temp(1, :), temp(2, :), 'ro', 'LineWidth', 2)