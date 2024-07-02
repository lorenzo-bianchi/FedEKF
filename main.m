clc; clear; close all;

DISEGNA_ANIMAZIONE = 0;
DISEGNA_ULTIMO = 0;
DISEGNA_PLOT = 0;
DISEGNA_ICP = 0;
displayErrori = 1;

seed = 9;
rng(seed)

load('percorsi.mat', 'percorsi');

dati

ekfs = FedEkf.empty(nRobot, 0);
TsGL = zeros(3, 3, nRobot);
TsLG = zeros(3, 3, nRobot);

main_simulazione;

%% Calcolo distanze vere e stimate tag-tag e tag-robot (errori SLAM relativi)
calcolo_errori;

%% Grafici
grafici;