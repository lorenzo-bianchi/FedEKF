%% Grafici
if DISEGNA_ANIMAZIONE
    for robot = 1:nRobot
        figure(robot)
    end

    if nRobot > 1
        disp("Premi invio...")
        pause
    end
    
    for k = 1:nPassi
        if mod(k,5) == 1
            disegna
        end
    end
end

if DISEGNA_ULTIMO
    disegna
end

if DISEGNA_PLOT
    %%
    colors = {'#0072BD', '#D95319', '#EDB120', '#7E2F8E', '#77AC30', '#4DBEEE', '#A2142F', '#0072BD', '#D95319', '#EDB120', '#7E2F8E', '#77AC30', '#4DBEEE', '#A2142F'};
    names = {'Tag1', 'Tag2', 'Tag3', 'Tag4', 'Tag5', 'Tag6', 'Tag7', 'Tag8', 'Tag9', 'Tag10', 'Tag11', 'Tag12', 'Tag13', 'Tag14'};
    t_min = 1;
    t_max = nPassi;
    time = t_min:t_max;

    for robot = 1:nRobot
        % Errori assoluti
        figure

        x = ekfs(robot).xHatTagStoria(:, time);
        y = ekfs(robot).yHatTagStoria(:, time);

        hAx1 = axes('Position', [0.05, 0.1, 0.4, 0.8]);
        for tag = 1:nTag
            posLoc = [x(tag, :); y(tag, :); ones(1, length(time))];
            posGlob = TsGL(:, :, robot)*posLoc;
            plot(time, posGlob(1, :), 'LineWidth', 1.5, 'Color', colors{tag}, 'DisplayName', names{tag})
            hold on
            plot(time, cTag(tag, 1)*ones(1, length(time)), '--', 'LineWidth', 1, 'Color', colors{tag}, 'DisplayName', '')
        end
        if pruning
            xline(stepStartPruning, '--k', 'LineWidth', 1, 'DisplayName', 'Pruning');
        end
        if sharing
            for t = startSharing
                if t ~= -1
                    xline(t, '-.k', 'LineWidth', 1, 'DisplayName', 'Sharing');
                end
            end
        end

        grid on
        xlabel('simulation step');
        ylabel('x [m]');
        legend('location', 'southeast');

        hAx2 = axes('Position', [0.55, 0.1, 0.4, 0.8]);
        for tag = 1:nTag
            posLoc = [x(tag, :); y(tag, :); ones(1, length(time))];
            posGlob = TsGL(:, :, robot)*posLoc;
            plot(time, posGlob(2, :), 'LineWidth', 1.5, 'Color', colors{tag}, 'DisplayName', names{tag})
            hold on
            plot(time, cTag(tag, 2)*ones(1, length(time)), '--', 'LineWidth', 1, 'Color', colors{tag}, 'DisplayName', '')
        end
        if pruning
            xline(stepStartPruning, '--k', 'LineWidth', 1, 'DisplayName', 'Pruning');
        end
        if sharing
            for t = startSharing
                if t ~= -1
                    xline(t, '-.k', 'LineWidth', 1, 'DisplayName', 'Sharing');
                end
            end
        end

        grid on
        xlabel('simulation step');
        ylabel('y [m]');
        legend('location', 'southeast');
        sgtitle(sprintf('Absolute errors robot %d', robot))
        
        set(gcf, 'position', [100, 100, 1500, 600]);
        %%
    end
end

if DISEGNA_ICP
    for robot = 1:nRobot
        cTagHat = [ekfs(robot).xHatTagStoria(:, end) ekfs(robot).yHatTagStoria(:, end)];
        [R1, t1] = icp2D(cTag, cTagHat);
        cTagTransformed = (R1 * cTag' + t1)';
        [R2, t2] = icp2D(cTagTransformed, cTagHat);
        cTagHatTransformed = (R2 * cTagHat' + t2)';
        figure;
        hold on;
        plot(cTagHat(:,1), cTagHat(:,2), 'r+', 'DisplayName', 'cTagHat (local)', 'MarkerSize', 7, 'LineWidth', 2);
        plot(cTagTransformed(:,1), cTagTransformed(:,2), 'kx', 'DisplayName', 'cTagTransformed', 'MarkerSize', 7, 'LineWidth', 2);
        plot(cTagHatTransformed(:,1), cTagHatTransformed(:,2), 'bo', 'DisplayName', 'cTagHatTransformed', 'MarkerSize', 7, 'LineWidth', 2);
        legend;
        title(sprintf('Tags alignment robot %d', robot));
        xlabel('x [m]');
        ylabel('y [m]');
        axis equal;
        grid on;
    end
end