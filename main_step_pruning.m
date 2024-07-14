clc; clear; close all;

load('percorsi.mat', 'percorsi');

dati
DISEGNA_ANIMAZIONE = 0;
displayErrori = 0;

delta = 20;
t0 = 300;
tf = nPassi*0.7;
steps = t0:delta:tf;
errori = zeros(1, length(steps));
for step = steps

    stepStartPruning = step;

    ekfs = FedEkf.empty(nRobot, 0);
    TsGL = cell(1, nRobot);
    TsLG = cell(1, nRobot);
    for robot = 1:nRobot
        TsGL{robot} = zeros(3, 3, 0);
        TsLG{robot} = zeros(3, 3, 0);
    end
    
    main_simulazione;
    
    %% Calcolo distanze vere e stimate tag-tag e tag-robot (errori SLAM relativi)
    calcolo_errori;

    fprintf('Step %d: errore = %f\n', step, erroreAssolutoRobot(1))
    errori(1, (step-t0)/delta+1) = erroreAssolutoRobot(1);

end

plot(steps, errori)

%% Grafici
grafici;