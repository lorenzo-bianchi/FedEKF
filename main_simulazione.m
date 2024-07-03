rng(seed);
for robot = 1:nRobot
    x0 = percorsi(1, 1, robot);
    y0 = percorsi(1, 2, robot);
    theta0 = percorsi(1, 3, robot);
    TsGL(:, :, 1, robot) = [[cos(theta0) -sin(theta0) x0]; [sin(theta0) cos(theta0) y0]; [0 0 1]];
    TsLG(:, :, 1, robot) = TsGL(:, :, 1, robot)^-1;

    misureRange = sqrt((x0-cTag(:,1)).^2+(y0-cTag(:,2)).^2) + sigmaDistanza*randn;

    stato0 = [0, 0, 0];
    ekfs(robot) = FedEkf(data, robot, stato0, misureRange);
end

startSharing = -1 * ones(1, nRobot);
tResets = cell(1, nRobot);
for robot = 1:nRobot
    tResets{robot} = [];
end

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
    end

    % CORREZIONE con altre misure
    if sharing && k > stepStartSharing
        sharedInfoArray = struct('id', {}, 'tags', {}, 'vars', {});
        for robot = 1:nRobot
            if sum(ekfs(robot).nPhiVett) == nTag
                if startSharing(robot) == -1
                    startSharing(robot) = k;
                end
                sharedInfoArray(end+1) = ekfs(robot).data_to_share();
            end
        end

        if ~isempty(sharedInfoArray)
            for robot = 1:nRobot
                ekfs(robot).correction_shared(sharedInfoArray);
                if ekfs(robot).do_reset
                    fprintf('Robot %d resetting at t=%d\n', robot, k)
                    ekfs(robot).reset();
                    tResets{robot}(end+1) = k;

                    x0 = percorsi(k, 1, robot);
                    y0 = percorsi(k, 2, robot);
                    theta0 = percorsi(k, 3, robot);
                    TsGL(:, :, 1, robot) = [[cos(theta0) -sin(theta0) x0]; [sin(theta0) cos(theta0) y0]; [0 0 1]];
                    TsLG(:, :, 1, robot) = TsGL(:, :, 1, robot)^-1;
                end
            end
        end
    end
end
fprintf("Tempo impiegato: %f s:\n", toc);