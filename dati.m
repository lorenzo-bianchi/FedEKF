seed = 23;
rng(seed)

DISEGNA_ANIMAZIONE = 0;
DISEGNA_ULTIMO = 0;
DISEGNA_PLOT = 0;
DISEGNA_ICP = 0;
displayErrori = 0;

data = struct();

nRobot = 6;

nPassi = 1000;
nPhi = 8; % numero ipotesi angolo (si può poi variare in funzione della distanza misurata)
pruning = 1;
minZerosStartPruning = ceil(nPhi*0.6);
stepStartPruning = 100;         % mettere valore piccolo per evitare errori iniziali
sharing = 1;
stepStartSharing = 400;
reset = 0;
resetThr = 200;

sigmaDistanza = 0.2; % std in m della misura di range
sigmaDistanzaModello = sigmaDistanza; % Per prevedere anche un'incertezza sulla deviazione standard dell'errore di misura
sigmaMisuraMedia = 1.0;

% ransac
numIterations = 50;
distanceThreshold = 0.1;
percentMinInliers = 0.8;

Nstep = 1; % passi tra una misura e la successiva
possibiliPhi = linspace(-pi+2*pi/nPhi, pi, nPhi);
sigmaPhi = 2*pi/(1.5*nPhi); %pi/(3*nPhi);

L = 10; % lunghezza lato ambiente (quadrato)
    
% Caratteristiche robot: si assume di aver calibrato l'odometria per
% l'errore sistematico ma ciò non impedisce di considerare che qualche
% erroretto c'è rimasto...
dVera = 0.26; % distanza presunta tra le due ruote
deltaRvera = 1; % fattore collegato con diametro ruota destra
deltaLvera = 1; % fattore collegato con diametro ruota sinistra
% ERRORE SISTEMATICO
d = dVera; %*1.001; % distanza vera tra le due ruote, se d = dVera non c'è errore sistematico
deltaR = deltaRvera; %*1.001; % errore sistematico sulla ruota destra
deltaL = deltaLvera; %*1.001; % errore sistematico sulla ruota sinistra
% COSTANTI ERRORE NON SISTEMATICO
KRvera = 0.0001;   % Costante KR errori Odometrici ruota destra: uR_e = N(uR, KR |uR|)
KLvera = KRvera;   % Costante KL errori Odometrici ruota sinistra: uL_e = N(uL, KL |uL|)
% ERRORE NELLA LORO STIMA
KR = KRvera; %*1.01;
KL = KLvera; %*1.01;

% Possibili configurazioni dei landmark (o ancore o tag): cTag è una matrice dove 
% la riga i-esima contiene le coordinate (x,y) del landmark i-esimo
% cTag = [0.6 0.6;
%         4.2 4.2;
%         1.4 3.8;
%         2.5 3.5;
%         3.8 1.5;
%         2.0 2.0];
% [nTag, ~] = size(cTag);

nTag = 10;
cTag = 0.9*L*rand(nTag, 2) + 0.05*L;

data.nPassi = nPassi;
data.nPhi = nPhi;
data.L = L;
data.possibiliPhi = possibiliPhi;
data.sigmaPhi = sigmaPhi;
data.cTag = cTag;
data.nTag = nTag;

data.sigmaDistanzaModello = sigmaDistanzaModello;
data.sigmaPhi = sigmaPhi;
data.sigmaMisuraMedia = sigmaMisuraMedia;

data.d = d;
data.deltaR = deltaR;
data.deltaL = deltaL;
data.dVera = dVera;
data.deltaRvera = deltaRvera;
data.deltaLvera = deltaLvera;

data.KR = KR;
data.KL = KL;
data.KRvera = KRvera;
data.KLvera = KLvera;

data.pruning = pruning;
data.minZerosStartPruning = minZerosStartPruning;
data.stepStartPruning = stepStartPruning;

data.numIterations = numIterations;
data.distanceThreshold = distanceThreshold;
data.percentMinInliers = percentMinInliers;

data.resetThr = resetThr;
data.reset = reset;