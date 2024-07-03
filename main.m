clc; clear; close all;

load('percorsi.mat', 'percorsi');

dati

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

%% Grafici
grafici;