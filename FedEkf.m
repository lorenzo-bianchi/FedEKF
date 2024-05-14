% FedEkf class definition
classdef FedEkf < handle
    properties
        data
        innovazione
        pesi
        xHatSLAM
        P
        Pmeno
        Ptag
        xHatSLAMmeno
        F
        W
        H
        Rs
        k
        sigmaD
        sigmaPhi
        nPhiVett
        xHatIndices
        xHatCumIndices
    end
    
    methods
        % Class constructor
        function obj = FedEkf(data, x0, misure, sigmaD, sigmaPhi)
            obj.data = data;

            obj.sigmaD = sigmaD;
            obj.sigmaPhi = sigmaPhi;

            nTag = data.nTag;
            nPhiMax = data.nPhi;
            obj.nPhiVett = data.nPhi*ones(1, nTag);
            nPassi = data.nPassi;

            obj.innovazione = zeros(nTag*nPhiMax, 1);
            obj.pesi = (1/nPhiMax)*ones(nTag, nPhiMax);
            obj.xHatSLAM = zeros(3+(3+nPhiMax)*nTag, nPassi);
            obj.P = zeros(3+(3+nPhiMax)*nTag, 3+(3+nPhiMax)*nTag);
            obj.Ptag = diag([0, 0, sigmaD^2, sigmaPhi^2*ones(1, nPhiMax)]);

            obj.xHatSLAMmeno = zeros(3+(3+nPhiMax)*nTag, 1);
            obj.F = eye(3+(3+nPhiMax)*nTag);
            obj.W = zeros(3+(3+nPhiMax)*nTag, 2);
            obj.H = zeros(nTag*nPhiMax, 3+(3+nPhiMax)*nTag);
            obj.Rs = zeros(nTag*nPhiMax, nTag*nPhiMax);

            obj.k = 0;

            obj.xHatIndices = [1 3 (3+nPhiMax)*ones(1, nTag)];
            obj.xHatCumIndices = cumsum(obj.xHatIndices(1:end-1));

            % Inizializzazione
            obj.xHatSLAM(1:3, 1) = x0;
            obj.xHatSLAM(4:nPhiMax+3:end, 1) = obj.xHatSLAM(1,1);
            obj.xHatSLAM(5:nPhiMax+3:end, 1) = obj.xHatSLAM(2,1);
            obj.xHatSLAM(6:nPhiMax+3:end, 1) = misure;
            for jndPhi = 1:nPhiMax
                obj.xHatSLAM(6+jndPhi:nPhiMax+3:end,1) = data.possibiliPhi(jndPhi);
            end
            for indTag = 1:nTag    
                obj.P(4+(3+nPhiMax)*(indTag-1):3+(3+nPhiMax)*indTag,4+(3+nPhiMax)*(indTag-1):3+(3+nPhiMax)*indTag) = obj.Ptag;
            end
        end
        

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
            nTag = obj.data.nTag;

            % Stima a priori posizione robot
            x_r = obj.xHatSLAMmeno(1);
            y_r = obj.xHatSLAMmeno(2);

            indMatCum = cumsum([1 obj.nPhiVett(1:end-1)]);
    
            for indTag = 1:nTag
                % Calcolo della stima a priori della posizione dell'ipotesi 
                % j del landmark i, quindi la misura attesa da questa posizione, 
                % la sua probabilità e la corrispondente riga della jacobiana H
                indMat = indMatCum(indTag);

                nPhi = obj.nPhiVett(indTag);
                ind0 = obj.xHatCumIndices(indTag+1);
                x_i   = obj.xHatSLAMmeno(0+ind0);
                y_i   = obj.xHatSLAMmeno(1+ind0);
                rho_i = obj.xHatSLAMmeno(2+ind0);

                probMisura_ij = zeros(nPhi, 1);
                for indPhi = 1:nPhi
                    phi_ij = obj.xHatSLAMmeno(2+ind0+indPhi);
                    cosPhi_ij = cos(phi_ij);
                    sinPhi_ij = sin(phi_ij);
                    xTag_ij = x_i + rho_i*cosPhi_ij;
                    yTag_ij = y_i + rho_i*sinPhi_ij;
                    misuraRange_ij = sqrt((xTag_ij-x_r)^2+(yTag_ij-y_r)^2);
                    deltaMisura_ij = misureRange(indTag) - misuraRange_ij;

                    
                    obj.innovazione(indMat+indPhi) = deltaMisura_ij;
                    probMisura_ij(indPhi) = exp(-deltaMisura_ij^2/(2*obj.sigmaD^2));
                    obj.pesi(indTag,indPhi) = obj.pesi(indTag,indPhi)*probMisura_ij(indPhi);
    
                    obj.H(indMat+indPhi, 1:2) = [x_r-xTag_ij, y_r-yTag_ij];
                    obj.H(indMat+indPhi, 0+ind0) = xTag_ij-x_r;
                    obj.H(indMat+indPhi, 1+ind0) = yTag_ij-y_r;
                    obj.H(indMat+indPhi, 2+ind0) = (xTag_ij-x_r)*cosPhi_ij+(yTag_ij-y_r)*sinPhi_ij;
                    obj.H(indMat+indPhi, 2+ind0+indPhi)= ((x_r-xTag_ij)*sinPhi_ij+(yTag_ij-y_r)*cosPhi_ij)*rho_i;
                    obj.H(indMat+indPhi, :) = obj.H(indMat+indPhi,:)/misuraRange_ij;
                end
                lambda_ij = probMisura_ij/sum(probMisura_ij);
                
                % Matrice covarianza misure con formula che tiene conto dell'Information Sharing
                for indPhi = 1:nPhi
                    obj.Rs(indMat+indPhi, indMat+indPhi) = obj.sigmaD^2/(max(0.0001, lambda_ij(indPhi)));
                end
    
            end
    
            % Aggiornamento stima (a posteriori)
            KalmanGain = obj.Pmeno*obj.H'*pinv(obj.H*obj.Pmeno*obj.H'+obj.Rs);
            obj.xHatSLAM(1:sum(obj.xHatIndices)-1, obj.k+1) = obj.xHatSLAMmeno(1:sum(obj.xHatIndices)-1) + KalmanGain*obj.innovazione;
            obj.P = (eye(sum(obj.xHatIndices)-1) - KalmanGain*obj.H)*obj.Pmeno;
    
            % Aggiornamento pesi
            obj.pesi = obj.pesi ./ sum(obj.pesi, 2);

            % Pruning
            change = false;
            for indTag = 1:nTag
                nPhi = obj.nPhiVett(indTag);
                indPhi = 1;
                while indPhi <= nPhi
                    if obj.pesi(indTag, indPhi) < 0.00001/nPhi
                        change = true;
                        nPhi = nPhi - 1;

                        temp = obj.pesi(indTag, indPhi+1:end);
                        obj.pesi(indTag, indPhi:indPhi+length(temp)-1) = temp;
                        obj.pesi(indTag, end) = 0;

                        obj.xHatIndices(2+indTag) = obj.xHatIndices(2+indTag) - 1;
                        obj.xHatCumIndices(2+indTag:end) = obj.xHatCumIndices(2+indTag:end) - 1;
                        obj.nPhiVett(indTag) = obj.nPhiVett(indTag) - 1;

                        ind0 = obj.xHatCumIndices(indTag+1);
                        i = ind0 + 2 + indPhi;
                        obj.P = obj.P([1:i-1, i+1:end], [1:i-1, i+1:end]);
                        obj.Pmeno = obj.Pmeno([1:i-1, i+1:end], [1:i-1, i+1:end]);
                        obj.xHatSLAM = obj.xHatSLAM([1:i-1, i+1:end], :);
                    else
                        indPhi = indPhi + 1;
                    end
                end
            end
            if change
                nPhiTagNew = sum(obj.nPhiVett);
                stateLenNew = sum(obj.xHatIndices)-1;

                obj.innovazione = zeros(nPhiTagNew, 1);
    
                obj.xHatSLAMmeno = zeros(stateLenNew, 1);
                obj.F = eye(stateLenNew);
                obj.W = zeros(stateLenNew, 2);
                obj.H = zeros(nPhiTagNew, stateLenNew);
                obj.Rs = zeros(nPhiTagNew, nPhiTagNew);
            end
        end
    end
end