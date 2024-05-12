% FedEkf class definition
classdef FedEkf < handle
    properties
        data
        probMisura_ij
        innovazione
        pesi
        pesiNuovi
        xHatSLAM
        P
        Pmeno
        Probot
        Ptag
        xHatSLAMmeno
        F
        W
        H
        Rs
        k
        sigmaD
        sigmaPhi
    end
    
    methods
        % Class constructor
        function obj = FedEkf(data, x0, misure, sigmaD, sigmaPhi)
            obj.data = data;

            obj.sigmaD = sigmaD;
            obj.sigmaPhi = sigmaPhi;

            nPhi = data.nPhi;
            nTag = data.nTag;
            nPassi = data.nPassi;

            obj.probMisura_ij = zeros(nPhi, 1);
            obj.innovazione = zeros(nTag*nPhi, 1);
            obj.pesi = (1/nPhi)*ones(nTag, nPhi);
            obj.pesiNuovi = zeros(nTag, nPhi);
            obj.xHatSLAM = zeros(3+(3+nPhi)*nTag, nPassi);
            obj.P = zeros(3+(3+nPhi)*nTag, 3+(3+nPhi)*nTag);
            obj.Probot = zeros(3,3);
            obj.Ptag = diag([0, 0, sigmaD^2, sigmaPhi^2*ones(1, nPhi)]);

            obj.xHatSLAMmeno = zeros(3+(3+nPhi)*nTag, 1);
            obj.F = eye(3+(3+nPhi)*nTag);
            obj.W = zeros(3+(3+nPhi)*nTag, 2);
            obj.H = zeros(nTag*nPhi, 3+(3+nPhi)*nTag);
            obj.Rs = zeros(nTag*nPhi, nTag*nPhi);

            obj.k = 0;

            % Inizializzazione
            % xHatSLAM è inizializzato senza perdita di generalità supponendo di fissare il sistema di riferimento del robot stimato 
            % (che dovrebbe essere in (0,0,0) all'inizio) coincidente con quello vero
            obj.xHatSLAM(1:3, 1) = x0;
            obj.xHatSLAM(4:nPhi+3:end, 1) = obj.xHatSLAM(1,1);
            obj.xHatSLAM(5:nPhi+3:end, 1) = obj.xHatSLAM(2,1);
            obj.xHatSLAM(6:nPhi+3:end, 1) = misure;
            for jndPhi = 1:nPhi
                obj.xHatSLAM(6+jndPhi:nPhi+3:end,1) = data.possibiliPhi(jndPhi);
            end
            for indTag = 1:nTag    
                obj.P(4+(3+nPhi)*(indTag-1):3+(3+nPhi)*indTag,4+(3+nPhi)*(indTag-1):3+(3+nPhi)*indTag) = obj.Ptag;
            end
        end
        
        % 
        function [obj] = prediction(obj, uRe, uLe)
            obj.k = obj.k + 1;

            uk = (uRe + uLe)/2;
            omegak = (uRe - uLe)/obj.data.d;

            cosk = cos(obj.xHatSLAM(3, obj.k));
            sink = sin(obj.xHatSLAM(3, obj.k));
        
            % Gli unici elementi che cambiano sono le coordinate
            % del robot in posizione 1, 2 e 3 del vettore xHatSLAM
            obj.xHatSLAMmeno = obj.xHatSLAM(:, obj.k);
            obj.xHatSLAMmeno(1) = obj.xHatSLAMmeno(1) + uk*cosk;
            obj.xHatSLAMmeno(2) = obj.xHatSLAMmeno(2) + uk*sink;
            obj.xHatSLAMmeno(3) = obj.xHatSLAMmeno(3) + omegak;
        
            % Aggiornamento degli elementi variabili della jacobiana F = df/dx
            obj.F(1,3) = -uk*sink;
            obj.F(2,3) = uk*cosk;
            
            % Jacobiana W = df/dw
            obj.W(1,1) = 0.5*cosk;
            obj.W(1,2) = 0.5*cosk;
            obj.W(2,1) = 0.5*sink;
            obj.W(2,2) = 0.5*sink;
            obj.W(3,1) = 1/obj.data.d;
            obj.W(3,2) = -1/obj.data.d;
            
            % Matrice covarianza errore odometrico
            Q = diag([obj.data.KR*abs(uRe); obj.data.KL*abs(uLe)]);
        
            % Calcolo matrice P^-
            obj.Pmeno = obj.F*obj.P*obj.F' + obj.W*Q*obj.W';
        end
        
        % 
        function [obj] = correction(obj, misureRange)
            nPhi = obj.data.nPhi;
            nTag = obj.data.nTag;

            % Stima a priori posizione robot
            x_r = obj.xHatSLAMmeno(1);
            y_r = obj.xHatSLAMmeno(2);
    
            for indTag = 1:nTag
                % Calcolo della stima a priori della posizione dell'ipotesi 
                % j del landmark i, quindi la misura attesa da questa posizione, 
                % la sua probabilità e la corrispondente riga della jacobiana H
                x_i = obj.xHatSLAMmeno(4+(3+nPhi)*(indTag-1));
                y_i = obj.xHatSLAMmeno(5+(3+nPhi)*(indTag-1));
                rho_i = obj.xHatSLAMmeno(6+(3+nPhi)*(indTag-1));
                for jndPhi = 1:nPhi
                    phi_ij = obj.xHatSLAMmeno(6+(3+nPhi)*(indTag-1)+jndPhi);
                    cosPhi_ij = cos(phi_ij);
                    sinPhi_ij = sin(phi_ij);
                    xTag_ij = x_i + rho_i*cosPhi_ij;
                    yTag_ij = y_i + rho_i*sinPhi_ij;
                    misuraRange_ij = sqrt((xTag_ij-x_r)^2+(yTag_ij-y_r)^2);
                    deltaMisura_ij = misureRange(indTag) - misuraRange_ij;
                    obj.innovazione(nPhi*(indTag-1)+jndPhi) = deltaMisura_ij;
                    obj.probMisura_ij(jndPhi) = exp(-deltaMisura_ij^2/(2*obj.sigmaD^2));
                    obj.pesiNuovi(indTag,jndPhi) = obj.pesi(indTag,jndPhi)*obj.probMisura_ij(jndPhi);
    
                    obj.H(nPhi*(indTag-1)+jndPhi, 1:2) = [x_r-xTag_ij, y_r-yTag_ij];
                    obj.H(nPhi*(indTag-1)+jndPhi, 3+(3+nPhi)*(indTag-1)+1) = xTag_ij-x_r;
                    obj.H(nPhi*(indTag-1)+jndPhi, 3+(3+nPhi)*(indTag-1)+2) = yTag_ij-y_r;
                    obj.H(nPhi*(indTag-1)+jndPhi, 3+(3+nPhi)*(indTag-1)+3) = (xTag_ij-x_r)*cosPhi_ij+(yTag_ij-y_r)*sinPhi_ij;
                    obj.H(nPhi*(indTag-1)+jndPhi, 6+(3+nPhi)*(indTag-1)+jndPhi)= ((x_r-xTag_ij)*sinPhi_ij+(yTag_ij-y_r)*cosPhi_ij)*rho_i;
                    obj.H(nPhi*(indTag-1)+jndPhi, :) = obj.H(nPhi*(indTag-1)+jndPhi,:)/misuraRange_ij;
                end
                lambda_ij = obj.probMisura_ij/sum(obj.probMisura_ij);
                
                % Matrice covarianza misure con formula che tiene conto dell'Information Sharing
                for jndPhi = 1:nPhi
                    obj.Rs(nPhi*(indTag-1)+jndPhi,nPhi*(indTag-1)+jndPhi) = obj.sigmaD^2/(max(.0001, lambda_ij(jndPhi)));
                end
    
            end
    
            % Aggiornamento stima (stima a posteriori)
            KalmanGain = obj.Pmeno*obj.H'*pinv(obj.H*obj.Pmeno*obj.H'+obj.Rs);
            obj.xHatSLAM(:,obj.k+1) = obj.xHatSLAMmeno + KalmanGain*obj.innovazione;
            obj.P = (eye(3+(3+nPhi)*nTag) - KalmanGain*obj.H)*obj.Pmeno;
    
            % Aggiornamento pesi
            for indTag = 1:nTag
                obj.pesi(indTag,:) = obj.pesiNuovi(indTag,:)/sum(obj.pesiNuovi(indTag,:));
            end

        end
    end
end