figure
axis equal
hold on
grid on

plot(thisRobotTags(:, 1), thisRobotTags(:, 2), 'bo', 'LineWidth', 2)
plot(temp(1, :), temp(2, :), 'ro', 'LineWidth', 2)