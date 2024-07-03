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

    distanzeRobotVere(robot, :) = sqrt((xVett(k)-cTag(:,1)).^2+(yVett(k)-cTag(:,2)).^2)';

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
    posHatTagGlob = (TsGL(:, :, 1, robot)*posHatTagLoc)';
    
    erroriAssolutiTag(robot, :) = sqrt((posHatTagGlob(:, 1)-cTag(:, 1)).^2+(posHatTagGlob(:, 2)-cTag(:, 2)).^2);

    posRobLoc = [x_r; y_r; 1];
    posRobGlob = TsGL(:, :, 1, robot)*posRobLoc;
    erroreAssolutoRobot(robot) = sqrt((posRobGlob(1)-xVett(k))^2+(posRobGlob(2)-yVett(k))^2);
    
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