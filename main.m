clc; clear; close all;

seeds = 7;
for seed = seeds
    rng(seed);
    fprintf("Seed %d:\n", seed);

    DISEGNA_ANIMAZIONE = 0;
    DISEGNA_ULTIMO = 0;
    DISEGNA_PLOT = 1;
    GENERA = 1;
    displayErrori = 1;
    
    nRobot = 4;
    
    dati % definisce alcune costanti del problema
    
    percorsi = zeros(nPassi, 5, nRobot);
    ekfs = FedEkf.empty(nRobot, 0);
    TsGL = zeros(3, 3, nRobot);
    TsLG = zeros(3, 3, nRobot);
    for robot = 1:nRobot
        percorsi(:, :, robot) = percorsoRandom(data, GENERA);     % xVett, yVett, thetaVett, uRe, uLe
    
        x0 = percorsi(1, 1, robot);
        y0 = percorsi(1, 2, robot);
        theta0 = percorsi(1, 3, robot);
        TsGL(:, :, robot) = [[cos(theta0) -sin(theta0) x0]; [sin(theta0) cos(theta0) y0]; [0 0 1]];
        TsLG(:, :, robot) = TsGL(:, :, robot)^-1;
    
        misureRange = sqrt((x0-cTag(:,1)).^2+(y0-cTag(:,2)).^2) + sigmaDistanza*randn;
    
        stato0 = [0, 0, 0];
        ekfs(robot) = FedEkf(data, robot, stato0, misureRange);
    end
    
    sharedInfoProto.indici = [0 0]';
    sharedInfoProto.tags = zeros(2, nTag);
    sharedInfoArray(nRobot) = sharedInfoProto;

    tic;
    for k = 2:nPassi
        for robot = 1:nRobot
            x = percorsi(k, 1, robot);
            y = percorsi(k, 2, robot);
            uRe = percorsi(k, 4 ,robot);
            uLe = percorsi(k, 5 ,robot);
            % PREDIZIONE
            ekfs(robot).prediction(uRe, uLe);
            
            % CORREZIONE (ogni Nstep passi)
            if mod(k, Nstep) == 0
                misureRange = sqrt((x-cTag(:,1)).^2+(y-cTag(:,2)).^2) + sigmaDistanza*randn;
                ekfs(robot).correction(misureRange);
            end
            
            % Salva le coordinate cartesiane dei tag
            ekfs(robot).save_tags();

            % Calcola informazioni per gli altri robot
            if sharing && k > stepStartSharing
                sharedInfoArray(robot) = ekfs(robot).data_to_share();
            end
        end

        % CORREZIONE con altre misure
        if sharing && k > stepStartSharing
            for robot = 1:nRobot
                ekfs(robot).correction_shared(sharedInfoArray);
            end
        end
    end
    fprintf("Tempo impiegato: %f s:\n", toc);

    %% Calcolo distanze vere e stimate tag-tag e tag-robot (errori SLAM relativi)
    erroreAssolutoRobot = zeros(1, nRobot);
    distanzeRobotVere = zeros(nRobot, nTag);
    distanzeRobotStimate = zeros(nRobot, nTag);
    distanzeInterTagVere = zeros(nRobot, nTag*(nTag-1)/2);
    distanzeInterTagStimate = zeros(nRobot, nTag*(nTag-1)/2);
    erroriAssolutiTag = zeros(nRobot, nTag);

    for robot = 1:nRobot
        xVett = percorsi(:, 1, robot);
        yVett = percorsi(:, 2, robot);
    
        distanzeRobotVere(robot, :) = sqrt((xVett(end)-cTag(:,1)).^2+(yVett(end)-cTag(:,2)).^2)';

        x_r = ekfs(robot).xHatSLAM(1, end);
        y_r = ekfs(robot).xHatSLAM(2, end);
        xHatTag = ekfs(robot).xHatTagStoria(:, end);
        yHatTag = ekfs(robot).yHatTagStoria(:, end);
        distanzeRobotStimate(robot, :) = sqrt((x_r-xHatTag).^2+(y_r-yHatTag).^2);
        
        indice = 0;
        for indTag = 1:nTag-1
            for jndTag = indTag+1:nTag
                indice = indice + 1;
                distanzeInterTagVere(robot, indice) = sqrt((cTag(indTag,1)-cTag(jndTag,1)).^2+(cTag(indTag,2)-cTag(jndTag,2)).^2);
                distanzeInterTagStimate(robot, indice) = sqrt((xHatTag(indTag)-xHatTag(jndTag)).^2+(yHatTag(indTag)-yHatTag(jndTag)).^2);
            end
        end
    
        posHatTagLoc = [xHatTag'; yHatTag'; ones(1, nTag)];
        posHatTagGlob = (TsGL(:, :, robot)*posHatTagLoc)';
        
        erroriAssolutiTag(robot, :) = sqrt((posHatTagGlob(:, 1)-cTag(:, 1)).^2+(posHatTagGlob(:, 2)-cTag(:, 2)).^2);
    
        posRobLoc = [x_r; y_r; 1];
        posRobGlob = TsGL(:, :, robot)*posRobLoc;
        erroreAssolutoRobot(robot) = sqrt((posRobGlob(1)-xVett(end))^2+(posRobGlob(2)-yVett(end))^2);
        
        if displayErrori
            fprintf("Robot %d:\n", robot);
            % fprintf("\tDistanze robot-tag vere: ")
            % fprintf("%.3f ", distanzeRobotVere(robot, :));
            % fprintf("\n");
            % fprintf("\tDistanze robot-tag stimate: ")
            % fprintf("%.3f ", distanzeRobotStimate(robot, :));
            % fprintf("\n");
            fprintf("\tDifferenza distanze robot-tag: ")
            fprintf("%.3f ", abs(distanzeRobotVere(robot, :) - distanzeRobotStimate(robot, :)));
            fprintf("\n");
            fprintf("\tMedia differenza distanze robot-tag: ")
            fprintf("%.3f ", mean(abs(distanzeRobotVere(robot, :) - distanzeRobotStimate(robot, :))));
            fprintf("\n");
            % fprintf("\tDistanze tag-tag vere: ")
            % fprintf("%.3f ", distanzeInterTagVere(robot, :));
            % fprintf("\n");
            % fprintf("\tDistanze tag-tag stimate: ")
            % fprintf("%.3f ", distanzeInterTagStimate(robot, :));
            % fprintf("\n");
            % fprintf("\tDifferenza distanze tag-tag: ")
            % fprintf("%.3f ", abs(distanzeInterTagVere(robot, :) - distanzeInterTagStimate(robot, :)));
            % fprintf("\n");
            fprintf("\tMedia differenza distanze tag-tag: ")
            fprintf("%.3f ", mean(abs(distanzeInterTagVere(robot, :) - distanzeInterTagStimate(robot, :))));
            fprintf("\n");
            % fprintf("\tErrori assoluti tag: ")
            % fprintf("%.3f ", erroriAssolutiTag(robot, :));
            % fprintf("\n");
            fprintf("\tMedia errori assoluti tag: ")
            fprintf("%.3f ", mean(erroriAssolutiTag(robot, :)));
            fprintf("\n");
            fprintf("\tErrore assoluto robot: ")
            fprintf("%.3f ", erroreAssolutoRobot(robot));
            fprintf("\n\n");
        end
    end

    %% Animazione
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
    elseif DISEGNA_ULTIMO
        disegna
        if length(seeds) > 1
            pause
        end
    end
end

%% Grafici
if DISEGNA_PLOT
    colors = {'#0072BD', '#D95319', '#EDB120', '#7E2F8E', '#77AC30', '#4DBEEE', '#A2142F'};
    names = {'Tag1', 'Tag2', 'Tag3', 'Tag4', 'Tag5', 'Tag6', 'Tag7'};
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
            xline(stepStartSharing, '-.k', 'LineWidth', 1, 'DisplayName', 'Sharing');
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
            xline(stepStartSharing, '-.k', 'LineWidth', 1, 'DisplayName', 'Sharing');
        end

        grid on
        xlabel('simulation step');
        ylabel('y [m]');
        legend('location', 'southeast');
        sgtitle(sprintf('Absolute errors robot %d', robot))
        
        set(gcf, 'position', [100, 100, 1500, 600]);

        % Grafici tag
        cTagHat = [ekfs(robot).xHatTagStoria(:, end) ekfs(robot).yHatTagStoria(:, end)];
        [R, t] = icp2D(cTag, cTagHat);
        cTagHatTransformed = (R * cTag' + t)';
        figure;
        plot(cTag(:,1), cTag(:,2), 'bo', 'DisplayName', 'cTag (global)', 'MarkerSize', 7, 'LineWidth', 2);
        hold on;
        plot(cTagHat(:,1), cTagHat(:,2), 'r+', 'DisplayName', 'cTagHat (local)', 'MarkerSize', 7, 'LineWidth', 2);
        plot(cTagHatTransformed(:,1), cTagHatTransformed(:,2), 'kx', 'DisplayName', 'cTagHatTransformed', 'MarkerSize', 7, 'LineWidth', 2);
        legend;
        title(sprintf('Tags alignment robot %d', robot));
        xlabel('x [m]');
        ylabel('y [m]');
        axis equal;
        grid on;
    end
end