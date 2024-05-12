data = struct();

nPassi = 200;

Nstep = 1; % passi tra una misura e la successiva
nPhi = 8; % numero ipotesi angolo (si può poi variare in funzione della distanza misurata)
possibiliPhi = linspace(-pi+2*pi/nPhi, pi, nPhi);
sigmaPhi = 2*pi/(1.5*nPhi); %pi/(3*nPhi);

L = 200; % lunghezza lato ambiente (quadrato)
    
% Caratteristiche robot: si assume di aver calibrato l'odometria per
% l'errore sistematico ma ciò non impedisce di considerare che qualche
% erroretto c'è rimasto...
dVera = 26; % distanza presunta tra le due ruote
deltaRvera = 1; % fattore collegato con diametro ruota destra
deltaLvera = 1; % fattore collegato con diametro ruota sinistra
% ERRORE SISTEMATICO
d = dVera; %*1.001; % distanza vera tra le due ruote, se d = dVera non c'è errore sistematico
deltaR = deltaRvera; %*1.001; % errore sistematico sulla ruota destra
deltaL = deltaLvera; %*1.001; % errore sistematico sulla ruota sinistra
% COSTANTI ERRORE NON SISTEMATICO
KRvera = 0.01;   % Costante KR errori Odometrici ruota destra: uR_e = N(uR, KR |uR|)
KLvera = KRvera;   % Costante KL errori Odometrici ruota sinistra: uL_e = N(uL, KL |uL|)
% ERRORE NELLA LORO STIMA
KR = KRvera; %*1.01;
KL = KLvera; %*1.01;

sigmaDistanza = 5; % std in cm della misura di range
sigmaDistanzaModello = sigmaDistanza; % Per prevedere anche un'incertezza sulla deviazione standard dell'errore di misura

% Possibili configurazioni dei landmark (o ancore o tag): cTag è una matrice dove 
% la riga i-esima contiene le coordinate (x,y) del landmark i-esimo

% cTag = [50 + 100*rand(nTagVett(indProva),1) 50 + 100*rand(nTagVett(indProva),1)];
% cTag = [50 + 100*rand(10,1) 50 + 100*rand(10,1)];
% cTag = [60 60;
%         140 60;
%         100 140];
cTag = [60 60;
        60 150;
        150 60;
        150 150];
% cTag = [40 40;
%         160 40;
%         100 100;
%         40 160;
%         160 160];    
% cTag = [133.3333  133.3333
%   133.3333  266.6667
%   266.6667  133.3333
%   266.6667  266.6667
%   200 200];
[nTag, ~] = size(cTag);

data.nPassi = nPassi;
data.nPhi = nPhi;
data.possibiliPhi = possibiliPhi;
data.sigmaPhi = sigmaPhi;
data.nTag = nTag;

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