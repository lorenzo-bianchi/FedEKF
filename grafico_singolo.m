if length(dbstack) == 1
    robot = 3;
end

figure

x = ekfs(robot).xHatTagStoria(:, time);
y = ekfs(robot).yHatTagStoria(:, time);

hAx1 = axes('Position', [0.05, 0.1, 0.44, 0.8]);
for tag = 1:nTag
    posLoc = [x(tag, :); y(tag, :); ones(1, length(time))];
    if ~isempty(tResets{robot})
        posGlob = zeros(3, nPassi);
        
        t0 = 0;
        tReset = tResets{robot};
        T = TsGL{robot}(:, :, 1);
        posGlob(:, t0+1:tReset) = T*posLoc(:, t0+1:tReset);

        t0 = tResets{robot};
        tReset = nPassi;
        T = TsGL{robot}(:, :, 2);
        posGlob(:, t0+1:tReset) = T*posLoc(:, t0+1:tReset);
        
    else
        posGlob = TsGL{robot}(:, :, 1)*posLoc;
    end
    plot(time, posGlob(1, :), 'LineWidth', 1.5, 'Color', colors{tag}, 'DisplayName', names{tag})
    hold on
    plot(time, cTag(tag, 1)*ones(1, length(time)), '--', 'LineWidth', 1, 'Color', colors{tag}, 'DisplayName', '')
end
if pruning
    xline(stepStartPruning, '--k', 'LineWidth', 1, 'DisplayName', 'Pruning');
end
if sharing
    first = 1;
    for t = startSharing
        if t ~= -1
            if first
                first = 0;
                xline(t, '-.k', 'LineWidth', 1, 'DisplayName', 'Sharing');
            else
                xline(t, '-.k', 'LineWidth', 1, 'HandleVisibility', 'off');
            end
        end
    end
end
if ~isempty(tResets{robot})
    xline(tResets{robot}, '--r', 'LineWidth', 2, 'DisplayName', 'Reset');
end

grid on
xlabel('simulation step');
ylabel('x [m]');
legend('location', 'eastoutside');

hAx2 = axes('Position', [0.53, 0.1, 0.44, 0.8]);
for tag = 1:nTag
    posLoc = [x(tag, :); y(tag, :); ones(1, length(time))];
    if ~isempty(tResets{robot})
        posGlob = zeros(3, nPassi);
        
        t0 = 0;
        tReset = tResets{robot};
        T = TsGL{robot}(:, :, 1);
        posGlob(:, t0+1:tReset) = T*posLoc(:, t0+1:tReset);

        t0 = tResets{robot};
        tReset = nPassi;
        T = TsGL{robot}(:, :, 2);
        posGlob(:, t0+1:tReset) = T*posLoc(:, t0+1:tReset);
        
    else
        posGlob = TsGL{robot}(:, :, 1)*posLoc;
    end
    plot(time, posGlob(2, :), 'LineWidth', 1.5, 'Color', colors{tag}, 'DisplayName', names{tag})
    hold on
    plot(time, cTag(tag, 2)*ones(1, length(time)), '--', 'LineWidth', 1, 'Color', colors{tag}, 'DisplayName', '')
end
if pruning
    xline(stepStartPruning, '--k', 'LineWidth', 1, 'DisplayName', 'Pruning');
end
if sharing
    first = 1;
    for t = startSharing
        if t ~= -1
            if first
                first = 0;
                xline(t, '-.k', 'LineWidth', 1, 'DisplayName', 'Sharing');
            else
                xline(t, '-.k', 'LineWidth', 1, 'HandleVisibility', 'off');
            end
        end
    end
end
if ~isempty(tResets{robot})
    xline(tResets{robot}, '--r', 'LineWidth', 2, 'DisplayName', 'Reset');
end

grid on
xlabel('simulation step');
ylabel('y [m]');
legend('location', 'eastoutside');
sgtitle(sprintf('Absolute errors robot %d', robot))

set(gcf, 'position', [100, 100, 1500, 600]);

if length(dbstack) == 1
    saveas(gcf, sprintf('./Article/robot%d.eps', robot), 'epsc');
end