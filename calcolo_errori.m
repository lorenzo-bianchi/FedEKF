%% Calcolo distanze vere e stimate tag-tag e tag-robot (errori SLAM relativi)
erroreAssolutoRobot = zeros(1, nRobot);
distanzeRobotVere = zeros(nRobot, nTag);
distanzeRobotStimate = zeros(nRobot, nTag);
distanzeInterTagVere = zeros(nRobot, nTag*(nTag-1)/2);
distanzeInterTagStimate = zeros(nRobot, nTag*(nTag-1)/2);
erroriAssolutiTag = zeros(nRobot, nTag);
erroriMediPostICP = zeros(1, nRobot);

for robot = 1:nRobot
    xVett = percorsi(:, 1, robot);
    yVett = percorsi(:, 2, robot);

    distanzeRobotVere(robot, :) = sqrt((xVett(k)-cTag(:,1)).^2+(yVett(k)-cTag(:,2)).^2)';

    xRobotEnd = ekfs(robot).xHatSLAM(1, end);
    yRobotEnd = ekfs(robot).xHatSLAM(2, end);
    xHatTagEnd = ekfs(robot).xHatTagStoria(:, end);
    yHatTagEnd = ekfs(robot).yHatTagStoria(:, end);
    distanzeRobotStimate(robot, :) = sqrt((xRobotEnd-xHatTagEnd).^2+(yRobotEnd-yHatTagEnd).^2);
    
    indice = 0;
    for indTag = 1:nTag-1
        for jndTag = indTag+1:nTag
            indice = indice + 1;
            distanzeInterTagVere(robot, indice) = sqrt((cTag(indTag,1)-cTag(jndTag,1)).^2+(cTag(indTag,2)-cTag(jndTag,2)).^2);
            distanzeInterTagStimate(robot, indice) = sqrt((xHatTagEnd(indTag)-xHatTagEnd(jndTag)).^2+(yHatTagEnd(indTag)-yHatTagEnd(jndTag)).^2);
        end
    end

    posHatTagLoc = [xHatTagEnd'; yHatTagEnd'; ones(1, nTag)];
    posHatTagGlob = (TsGL{robot}(:, :, end)*posHatTagLoc)';
    
    erroriAssolutiTag(robot, :) = sqrt((posHatTagGlob(:, 1)-cTag(:, 1)).^2+(posHatTagGlob(:, 2)-cTag(:, 2)).^2);

    posRobLoc = [xRobotEnd; yRobotEnd; 1];
    posRobGlob = TsGL{robot}(:, :, end)*posRobLoc;
    erroreAssolutoRobot(robot) = sqrt((posRobGlob(1)-xVett(k))^2+(posRobGlob(2)-yVett(k))^2);

    robot_tags = [ekfs(robot).xHatTagStoria(:, end)'; ekfs(robot).yHatTagStoria(:, end)']';
    [RR, tt, inl] = ransacRototranslation(robot_tags, cTag, data.numIterations, 0.25, 5);
    if isempty(inl)
        erroriMediPostICP(robot) = nan;
    else
        TT = [RR, tt; zeros(1, 2), 1];
        temp = TT*[robot_tags'; ones(1, nTag)];
        temp = temp(1:2, :);
        erroriMediPostICP(robot) = mean(vecnorm(cTag' - temp));
    end
    
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
        fprintf("\tMedia distanze tag post ICP: ")
        fprintf("%.3f ", erroriMediPostICP);
        fprintf("\n");
    end
end