if length(dbstack) == 1
    robot = 4;
end

figure

font_size = 15;

x = ekfs(robot).xHatTagStoria(:, time);
y = ekfs(robot).yHatTagStoria(:, time);

positions = [0.05, 0.53];
tiledlayout(1, 2, 'TileSpacing', 'compact')
for idx = 1:2
    nexttile
    for tag = 1:nTag
        posLoc = [x(tag, :); y(tag, :); ones(1, length(time))];
        if ~isempty(tResets{robot})
            posGlob = zeros(3, nPassi);
            
            tResetVect = [0, tResets{robot}, nPassi];
            for t_idx = 1:length(tResetVect)-1
                t0 = tResetVect(t_idx);
                t1 = tResetVect(t_idx+1);
    
                T = TsGL{robot}(:, :, t_idx);
                posGlob(:, t0+1:t1) = T*posLoc(:, t0+1:t1);
            end        
        else
            posGlob = TsGL{robot}(:, :, 1)*posLoc;
        end
        if idx == 1
            plot(time, posGlob(1, :), 'LineWidth', 1, 'Color', colors{tag}, 'DisplayName', names{tag})
            hold on
            plot(time, cTag(tag, 1)*ones(1, length(time)), '--', 'LineWidth', 0.8, 'Color', colors{tag}, 'DisplayName', '')
        else
            plot(time, posGlob(2, :), 'LineWidth', 1, 'Color', colors{tag}, 'DisplayName', names{tag})
            hold on
            plot(time, cTag(tag, 2)*ones(1, length(time)), '--', 'LineWidth', 0.8, 'Color', colors{tag}, 'DisplayName', '')
        end
    end
    if pruning
        for i = 1:length(stepStartPruning{robot})
            if i == 1
                xline(stepStartPruning{robot}(i), '--k', 'LineWidth', 1, 'DisplayName', 'Pruning');
            else
                xline(stepStartPruning{robot}(i), '--k', 'LineWidth', 1, 'HandleVisibility', 'off');
            end
        end
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
        for i = 1:length(tResets{robot})
            if i == 1
                xline(tResets{robot}(i), '--r', 'LineWidth', 1.5, 'DisplayName', 'Reset');
            else
                xline(tResets{robot}(i), '--r', 'LineWidth', 1.5, 'HandleVisibility', 'off');
            end
        end
    end
    
    grid on
    xlabel('simulation step');
    if idx == 1
        ylabel('x [m]');
    else
        ylabel('y [m]');
        legend('location', 'eastoutside');
    end
    xticks(t_min-1:250:t_max);

    ax = gca;
    ax.FontSize = font_size;

end

sgtitle(sprintf('Absolute errors robot %d', robot), 'FontSize', font_size+4)

if length(dbstack) == 1
    saveas(gcf, sprintf('./Article/robot%d.eps', robot), 'epsc');
end