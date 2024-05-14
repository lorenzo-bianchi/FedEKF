clc; clear; close all;

for i = 1:10
    rng(i);
    
    DISEGNA = 0;
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
        ekfs(robot) = FedEkf(data, stato0, misureRange, sigmaDistanzaModello, sigmaPhi);
    
        if DISEGNA
            figure(robot)
        end
    end
    
    if nRobot > 1 && DISEGNA
        disp("Premi invio...")
        pause
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
        end
    
        if DISEGNA && mod(k,5) == 1
            disegna
        end
    end
    DeltaTsim = toc;
    fprintf("Tempo impiegato: %f s:\n", toc);
end

return

%% Calcolo distanze vere e stimate tag-tag e tag-robot (errori SLAM relativi)
for robot = 1:nRobot
    xVett = percorsi(:, 1, robot);
    yVett = percorsi(:, 2, robot);

    distanzeRobotVere = sqrt((xVett(end)-cTag(:,1)).^2+(yVett(end)-cTag(:,2)).^2)';
    distanzeRobotStimate = zeros(1,nTag);
    x_r = ekfs(robot).xHatSLAM(1,end);
    y_r = ekfs(robot).xHatSLAM(2,end);
    xHatTag = zeros(nTag,1);
    yHatTag = zeros(nTag,1);
    for indTag = 1:nTag
        x_i = ekfs(robot).xHatSLAM(4+(3+nPhi)*(indTag-1),end);
        y_i = ekfs(robot).xHatSLAM(5+(3+nPhi)*(indTag-1),end);
        rho_i = ekfs(robot).xHatSLAM(6+(3+nPhi)*(indTag-1),end);
        x_ti = 0;
        y_ti = 0;
        for jndPhi = 1:nPhi
            phi_ij = ekfs(robot).xHatSLAM(6+(3+nPhi)*(indTag-1)+jndPhi,end);
            cosPhi_ij = cos(phi_ij);
            sinPhi_ij = sin(phi_ij);
            xTag_ij = x_i + rho_i*cosPhi_ij;
            yTag_ij = y_i + rho_i*sinPhi_ij;
            x_ti = x_ti + xTag_ij*ekfs(robot).pesi(indTag, jndPhi);
            y_ti = y_ti + yTag_ij*ekfs(robot).pesi(indTag, jndPhi);
        end
        xHatTag(indTag) = x_ti;
        yHatTag(indTag) = y_ti;
        distanzeRobotStimate(indTag) = sqrt((x_r-x_ti)^2+(y_r-y_ti)^2);
    end
    
    distanzeInterTagVere = zeros(1, nTag*(nTag-1)/2);
    distanzeInterTagStimate = zeros(1, nTag*(nTag-1)/2);
    indice = 0;
    for indTag = 1:nTag-1
        for jndTag = indTag+1:nTag
            indice = indice + 1;
            distanzeInterTagVere(indice) = sqrt((cTag(indTag,1)-cTag(jndTag,1)).^2+(cTag(indTag,2)-cTag(jndTag,2)).^2);
            distanzeInterTagStimate(indice) = sqrt((xHatTag(indTag)-xHatTag(jndTag)).^2+(yHatTag(indTag)-yHatTag(jndTag)).^2);
        end
    end
    
    % Calcolo errori assoluti (= distanza) tra posizione vera e stimata di tag e
    % robot (errore assoluto SLAM)
    erroriAssolutiTag = zeros(1,nTag);
    for indTag = 1:nTag
        erroriAssolutiTag(indTag) = sqrt((xHatTag(indTag)-cTag(indTag,1))^2+(yHatTag(indTag)-cTag(indTag,2))^2);
    end
    erroreAssolutoRobot = sqrt((x_r-xVett(end))^2+(y_r-yVett(end))^2);
    erroreTraiettoria = zeros(nPassi,1);
    
    % Calcolo errore assoluto stima posizione robot nei vari passi
    for k = 1:nPassi
        erroreTraiettoria(k) = sqrt((xVett(k)-ekfs(robot).xHatSLAM(1,k))^2+(yVett(k)-ekfs(robot).xHatSLAM(2,k))^2);
    end
    
    if displayErrori
        fprintf("Robot %d:\n", robot);
        fprintf("\tDistanze vere: ")
        fprintf("%.3f ", distanzeRobotVere);
        fprintf("\n");
        fprintf("\tDistanze stimate: ")
        fprintf("%.3f ", distanzeRobotStimate);
        fprintf("\n");
        fprintf("\tDistanze inter-tag vere: ")
        fprintf("%.3f ", distanzeInterTagVere);
        fprintf("\n");
        fprintf("\tDistanze inter-tag stimate: ")
        fprintf("%.3f ", distanzeInterTagStimate);
        fprintf("\n");
        fprintf("\tErrori assoluti tag: ")
        fprintf("%.3f ", erroriAssolutiTag);
        fprintf("\n");
        fprintf("\tErrore assoluto robot: ")
        fprintf("%.3f ", erroreAssolutoRobot);
        fprintf("\n");
    end
end