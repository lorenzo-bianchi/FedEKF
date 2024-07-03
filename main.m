clc; clear; close all;

load('percorsi.mat', 'percorsi');

dati

ekfs = FedEkf.empty(nRobot, 0);
TsGL = zeros(3, 3, 1, nRobot);
TsLG = zeros(3, 3, 1, nRobot);

main_simulazione;

%% Calcolo distanze vere e stimate tag-tag e tag-robot (errori SLAM relativi)
calcolo_errori;

%% Grafici
grafici;