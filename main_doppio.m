clc; clear; close all;

load('percorsi.mat', 'percorsi');

dati

ekfsTot = {};
TsGLTot = {};
TsLGTot = {};

erroreAssolutoRobotTot = {};
distanzeRobotVereTot = {};
distanzeRobotStimateTot = {};
distanzeInterTagVereTot = {};
distanzeInterTagStimateTot = {};
erroriAssolutiTagTot = {};
    
for i = 0:1
    sharing = i;
    fprintf('Sharing: %d\n', sharing)

    ekfs = FedEkf.empty(nRobot, 0);

    TsGL = cell(1, nRobot);
    TsLG = cell(1, nRobot);
    for robot = 1:nRobot
        TsGL{robot} = zeros(3, 3, 0);
        TsLG{robot} = zeros(3, 3, 0);
    end

    main_simulazione;

    ekfsTot{i+1} = ekfs;
    TsGLTot{i+1} = TsGL;
    TsLGTot{i+1} = TsLG;

    %%
    calcolo_errori;

    erroreAssolutoRobotTot{i+1} = erroreAssolutoRobot;
    distanzeRobotVereTot{i+1} = distanzeRobotVere;
    distanzeRobotStimateTot{i+1} = distanzeRobotStimate;
    distanzeInterTagVereTot{i+1} = distanzeInterTagVere;
    distanzeInterTagStimateTot{i+1} = distanzeInterTagStimate;
    erroriAssolutiTagTot{i+1} = erroriAssolutiTag;

    fprintf('\n')
end

%% Errori
fprintf('Confronto\n')
for robot = 1:nRobot
    fprintf('Robot %d:\t\t\t\t\tNO SHARING\t SHARING\t MIGLIORAMENTO\n', robot)

    mean1 = mean(abs(distanzeRobotVereTot{1}(robot, :) - distanzeRobotStimateTot{1}(robot, :)));
    mean2 = mean(abs(distanzeRobotVereTot{2}(robot, :) - distanzeRobotStimateTot{2}(robot, :)));
    fprintf('\tMedia differenza distanze robot-tag: \t%.3f \t\t %.3f \t\t %.3f \n', mean1, mean2, mean1-mean2)

    mean1 = mean(abs(distanzeInterTagVereTot{1}(robot, :) - distanzeInterTagStimateTot{1}(robot, :)));
    mean2 = mean(abs(distanzeInterTagVereTot{2}(robot, :) - distanzeInterTagStimateTot{2}(robot, :)));
    fprintf('\tMedia differenza distanze tag-tag: \t%.3f \t\t %.3f \t\t %.3f \n', mean1, mean2, mean1-mean2)

    mean1 = mean(erroriAssolutiTagTot{1}(robot, :));
    mean2 = mean(erroriAssolutiTagTot{2}(robot, :));
    fprintf('\tMedia errori assoluti tag: \t\t%.3f \t\t %.3f \t\t %.3f \n', mean1, mean2, mean1-mean2)

    err1 = erroreAssolutoRobotTot{1}(robot);
    err2 = erroreAssolutoRobotTot{2}(robot);
    fprintf('\tErrore assoluto robot: \t\t\t%.3f \t\t %.3f \t\t %.3f \n', err1, err2, err1-err2)
    fprintf('\n')
end

%% Grafici
DISEGNA_ANIMAZIONE = 0;
grafici;