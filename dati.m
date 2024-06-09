data = struct();

nPassi = 6000;
pruning = 1;
minZerosStartPruning = 5;
stepStartPruning = 0;
sharing = 0;
stepStartSharing = 600;

sigmaDistanza = 0.05; % std in m della misura di range
sigmaDistanzaModello = sigmaDistanza; % Per prevedere anche un'incertezza sulla deviazione standard dell'errore di misura
sigmaMisuraMedia = 1.0;

Nstep = 1; % passi tra una misura e la successiva
nPhi = 8; % numero ipotesi angolo (si pu� poi variare in funzione della distanza misurata)
possibiliPhi = linspace(-pi+2*pi/nPhi, pi, nPhi);
sigmaPhi = 2*pi/(1.5*nPhi); %pi/(3*nPhi);

L = 5; % lunghezza lato ambiente (quadrato)
    
% Caratteristiche robot: si assume di aver calibrato l'odometria per
% l'errore sistematico ma ci� non impedisce di considerare che qualche
% erroretto c'� rimasto...
dVera = 0.26; % distanza presunta tra le due ruote
deltaRvera = 1; % fattore collegato con diametro ruota destra
deltaLvera = 1; % fattore collegato con diametro ruota sinistra
% ERRORE SISTEMATICO
d = dVera; %*1.001; % distanza vera tra le due ruote, se d = dVera non c'� errore sistematico
deltaR = deltaRvera; %*1.001; % errore sistematico sulla ruota destra
deltaL = deltaLvera; %*1.001; % errore sistematico sulla ruota sinistra
% COSTANTI ERRORE NON SISTEMATICO
KRvera = 0.0001;   % Costante KR errori Odometrici ruota destra: uR_e = N(uR, KR |uR|)
KLvera = KRvera;   % Costante KL errori Odometrici ruota sinistra: uL_e = N(uL, KL |uL|)
% ERRORE NELLA LORO STIMA
KR = KRvera; %*1.01;
KL = KLvera; %*1.01;

% Possibili configurazioni dei landmark (o ancore o tag): cTag � una matrice dove 
% la riga i-esima contiene le coordinate (x,y) del landmark i-esimo
cTag = [0.6 0.6;
        4.2 4.2;
        1.4 3.8;
        2.5 3.5;
        3.8 1.5;
        2.0 2.0];
[nTag, ~] = size(cTag);

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
data.stepStartPruning = stepStartPruning ;