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

%% plot variance
robot = 1;

t0 = 1;
tf = nPassi;
t = t0:tf;

tags = 1:nTag;
for tag = tags
    var_x = squeeze(ekfs(robot).varsStoria(1, tag, t))';
    var_y = squeeze(ekfs(robot).varsStoria(2, tag, t))';
    
    figure
    hold on
    grid on
    legend
    plot(t, var_x, 'b',  'LineWidth', 1.5, 'DisplayName', 'var_x')
    plot(t, var_y, 'r', 'LineWidth', 1.5, 'DisplayName', 'var_y')
    plot(t, sqrt(var_x.^2 + var_y.^2), 'k', 'LineWidth', 1.5, 'DisplayName', 'var_tot')
end