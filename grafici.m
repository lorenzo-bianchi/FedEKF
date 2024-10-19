%% Grafici
if DISEGNA_ANIMAZIONE
    for robot = 1:nRobot
        figure(robot)
    end

    pause(5)

    if nRobot > 1
        disp('Premi invio...')
        pause
    end
    
    for k = 1:nPassi
        if mod(k,10) == 1
            disegna
        end
    end
end

if DISEGNA_ULTIMO
    disegna
end

if DISEGNA_PLOT
    %%
    colors = {'#0072BD', '#D95319', '#EDB120', '#7E2F8E', '#77AC30', '#4DBEEE', '#A2142F', '#1010FF', '#121A00', '#FFCAC1', '#10FF10', '#FF10FF', '#00FFCA', '#FF129E'};
    names = {'Tag1', 'Tag2', 'Tag3', 'Tag4', 'Tag5', 'Tag6', 'Tag7', 'Tag8', 'Tag9', 'Tag10', 'Tag11', 'Tag12', 'Tag13', 'Tag14'};
    t_min = 1;
    t_max = nPassi;
    time = t_min:t_max;

    for robot = 1:nRobot
        % Errori assoluti
        grafico_singolo;

        if 1
            print(['plot', num2str(robot), '.eps'], '-depsc2');
        end
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

if DISEGNA_VAR
    %%
    robot = 3;

    t0 = 1;
    tf = nPassi;
    t = t0:tf;
    
    tags = 1:1; %nTag;
    for tag = tags
        var_x = squeeze(ekfs(robot).varsStoria(1, tag, t))';
        var_y = squeeze(ekfs(robot).varsStoria(2, tag, t))';
        
        figure
        hold on
        grid on
        legend('Interpreter', 'latex', 'FontSize', 12)
        plot(t, sqrt(var_x), 'b', 'LineWidth', 1.5, 'DisplayName', '$\sigma_x$')
        plot(t, sqrt(var_y), 'r', 'LineWidth', 1.5, 'DisplayName', '$\sigma_y$')
        % plot(t, sqrt(var_x.^2 + var_y.^2), 'k', 'LineWidth', 1.5, 'DisplayName', 'var_tot')
        xlabel('simulation step');
        ylabel('[m]');
    end
end