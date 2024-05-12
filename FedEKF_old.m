misureRange(:,1) = sqrt((xVett(1)-cTag(:,1)).^2+(yVett(1)-cTag(:,2)).^2) + sigmaDistanza*randn;

T0 = tic;
for k = 1:nPassi-1
    % PASSO DI PREDIZIONE
    uk = ((uRe(k)+uLe(k))/2);
    omegak = ((uRe(k)-uLe(k))/d);
    cosk = cos(xHatSLAM(3,k));
    sink = sin(xHatSLAM(3,k));

    % Nella predizione gli unici elementi che cambiano sono le coordinate
    % del robot in posizione 1, 2 e 3 del vettore xHatSLAM
    xHatSLAMmeno = xHatSLAM(:,k);
    xHatSLAMmeno(1) = xHatSLAMmeno(1) + uk*cosk;
    xHatSLAMmeno(2) = xHatSLAMmeno(2) + uk*sink;
    xHatSLAMmeno(3) = xHatSLAMmeno(3) + omegak;

    % Aggiornamento degli elementi variabili della jacobiana F = df/dx
    F(1,3) = -uk*sink;
    F(2,3) = uk*cosk;
    
    % Jacobiana W = df/dw
    W(1,1) = 0.5*cosk;
    W(1,2) = 0.5*cosk;
    W(2,1) = 0.5*sink;
    W(2,2) = 0.5*sink;
    W(3,1) = 1/d;
    W(3,2) = -1/d;
    
    % Matrice covarianza errore odometrico
    Q = diag([KR*abs(uRe(k)); KL*abs(uLe(k))]);

    % Calcolo matrice P^-
    Pmeno = F*P*F' + W*Q*W';
    
    % PASSO DI CORREZIONE (ha effetto solo ogni Nstep passi)
    if mod(k+1,Nstep) == 1 || (Nstep == 1) 

        % Incamero la misura di range
        misureRange(:,k+1) = sqrt((xVett(k+1)-cTag(:,1)).^2+(yVett(k+1)-cTag(:,2)).^2) + sigmaDistanza*randn;
        
        H = zeros(nTag*nPhi,3+(3+nPhi)*nTag); % Jacobiana delle misure di range dh/dx
        
        % Stima a priori posizione robot
        x_r = xHatSLAMmeno(1);
        y_r = xHatSLAMmeno(2);

        for indTag = 1:nTag
            % In queste righe si calcola la stima a priori della posizione
            % dell'ipotesi j-esima del landmark i-esimo, quindi la misura attesa
            % da questa posizione, la sua probabilità e la corrispondente
            % riga della jacobiana H
            x_i = xHatSLAMmeno(4+(3+nPhi)*(indTag-1));
            y_i = xHatSLAMmeno(5+(3+nPhi)*(indTag-1));
            rho_i = xHatSLAMmeno(6+(3+nPhi)*(indTag-1));
            for jndPhi = 1:nPhi
                phi_ij = xHatSLAMmeno(6+(3+nPhi)*(indTag-1)+jndPhi);
                cosPhi_ij = cos(phi_ij);
                sinPhi_ij = sin(phi_ij);
                xTag_ij = x_i + rho_i*cosPhi_ij;
                yTag_ij = y_i + rho_i*sinPhi_ij;
                misuraRange_ij = sqrt((xTag_ij-x_r)^2+(yTag_ij-y_r)^2);
                deltaMisura_ij = misureRange(indTag,k+1) - misuraRange_ij;
                innovazione(nPhi*(indTag-1)+jndPhi) = deltaMisura_ij;
                probMisura_ij(jndPhi) = exp(-deltaMisura_ij^2/(2*sigmaDistanzaModello^2));
                pesiNuovi(indTag,jndPhi) = pesi(indTag,jndPhi)*probMisura_ij(jndPhi);

                H(nPhi*(indTag-1)+jndPhi,1:2) = [x_r-xTag_ij, y_r-yTag_ij];
                H(nPhi*(indTag-1)+jndPhi,3+(3+nPhi)*(indTag-1)+1) = xTag_ij-x_r;
                H(nPhi*(indTag-1)+jndPhi,3+(3+nPhi)*(indTag-1)+2) = yTag_ij-y_r;
                H(nPhi*(indTag-1)+jndPhi,3+(3+nPhi)*(indTag-1)+3) = (xTag_ij-x_r)*cosPhi_ij+(yTag_ij-y_r)*sinPhi_ij;
                H(nPhi*(indTag-1)+jndPhi,6+(3+nPhi)*(indTag-1)+jndPhi)= ((x_r-xTag_ij)*sinPhi_ij+(yTag_ij-y_r)*cosPhi_ij)*rho_i;
                H(nPhi*(indTag-1)+jndPhi,:) = H(nPhi*(indTag-1)+jndPhi,:)/misuraRange_ij;
            end
            lambda_ij = probMisura_ij/sum(probMisura_ij);
            
            % Matrice covarianza misure con formula che tiene conto dell'Information Sharing
            for jndPhi = 1:nPhi
                Rs(nPhi*(indTag-1)+jndPhi,nPhi*(indTag-1)+jndPhi) = sigmaDistanzaModello^2/(max(.0001, lambda_ij(jndPhi)));
            end

        end

        % Aggiornamento stima (stima a posteriori)
        KalmanGain = Pmeno*H'*pinv(H*Pmeno*H'+Rs);
        xHatSLAM(:,k+1) = xHatSLAMmeno + KalmanGain*innovazione;
        P = (eye(3+(3+nPhi)*nTag) - KalmanGain*H)*Pmeno;

        % Aggiornamento pesi
        for indTag = 1:nTag
            pesi(indTag,:) = pesiNuovi(indTag,:)/sum(pesiNuovi(indTag,:));
        end
        
    else 
        
        % Se non ho misure confermo la stima a priori
        xHatSLAM(:,k+1) = xHatSLAMmeno;
        P = Pmeno;
        
    end

    if DISEGNA && mod(k,5) == 1
        figure(1)
        hold off
        disegnaFig
        title(num2str(k))
        plot(xVett(max(1,k-30):k+1)/100,yVett(max(1,k-30):k+1)/100,'k','LineWidth',2)
        plot(xVett(k+1)/100,yVett(k+1)/100,'ko')
        axis([0 2 -0 2])
        plot(xHatSLAM(1,k+1)/100,xHatSLAM(2,k+1)/100,'o')
        for indTag = 1:nTag
            x_i = xHatSLAM(4+(3+nPhi)*(indTag-1),k+1);
            y_i = xHatSLAM(5+(3+nPhi)*(indTag-1),k+1);
            rho_i = xHatSLAM(6+(3+nPhi)*(indTag-1),k+1);
            x_ti = 0;
            y_ti = 0;
            for jndPhi = 1:nPhi
                phi_ij = xHatSLAM(6+(3+nPhi)*(indTag-1)+jndPhi,k+1);
                cosPhi_ij = cos(phi_ij);
                sinPhi_ij = sin(phi_ij);
                xTag_ij = x_i + rho_i*cosPhi_ij;
                yTag_ij = y_i + rho_i*sinPhi_ij;
                x_ti = x_ti + xTag_ij*pesi(indTag,jndPhi);
                y_ti = y_ti + yTag_ij*pesi(indTag,jndPhi);
                plot(xTag_ij/100,yTag_ij/100,'m.','MarkerSize',max(1,ceil(10*pesi(indTag,jndPhi))))
            end
            plot(x_ti/100,y_ti/100,'*') % posizione media stimata tag
        end
        axis([0 2 0 2])
%         for indTag = 1:nTag,
%             if k<nPassi,
%                 figure(2+indTag)
%                 bar(pesi(indTag,:))
%             end
%         end
        pause(.01)
    end
end
DeltaTsim = toc;
disp(DeltaTsim)
% toc

% Calcolo distanze vere e stimate tag-tag e tag-robot (errori SLAM relativi)
distanzeRobotVere = sqrt((xVett(end)-cTag(:,1)).^2+(yVett(end)-cTag(:,2)).^2)';
distanzeRobotStimate = zeros(1,nTag);
x_r = xHatSLAM(1,end);
y_r = xHatSLAM(2,end);
xHatTag = zeros(nTag,1);
yHatTag = zeros(nTag,1);
for indTag = 1:nTag
    x_i = xHatSLAM(4+(3+nPhi)*(indTag-1),end);
    y_i = xHatSLAM(5+(3+nPhi)*(indTag-1),end);
    rho_i = xHatSLAM(6+(3+nPhi)*(indTag-1),end);
    x_ti = 0;
    y_ti = 0;
    for jndPhi = 1:nPhi
        phi_ij = xHatSLAM(6+(3+nPhi)*(indTag-1)+jndPhi,end);
        cosPhi_ij = cos(phi_ij);
        sinPhi_ij = sin(phi_ij);
        xTag_ij = x_i + rho_i*cosPhi_ij;
        yTag_ij = y_i + rho_i*sinPhi_ij;
        x_ti = x_ti + xTag_ij*pesi(indTag,jndPhi);
        y_ti = y_ti + yTag_ij*pesi(indTag,jndPhi);
    end
    xHatTag(indTag) = x_ti;
    yHatTag(indTag) = y_ti;
    distanzeRobotStimate(indTag) = sqrt((x_r-x_ti)^2+(y_r-y_ti)^2);
end

distanzeInterTagVere = zeros(1,nTag*(nTag-1)/2);
distanzeInterTagStimate = zeros(1,nTag*(nTag-1)/2);
indice = 0;
for indTag = 1:nTag-1
    for jndTag = indTag+1:nTag
        indice = indice + 1;
        distanzeInterTagVere(indice) = sqrt((cTag(indTag,1)-cTag(jndTag,1)).^2+(cTag(indTag,2)-cTag(jndTag,2)).^2);
        distanzeInterTagStimate(indice) = sqrt((xHatTag(indTag)-xHatTag(jndTag)).^2+(yHatTag(indTag)-yHatTag(jndTag)).^2);
    end
end

% Calcolo errori assoluti (= distanza) tra posizione vera e stimata di tag e
% robot (errore assoluto SLAM)
erroriAssolutiTag = zeros(1,nTag);
for indTag = 1:nTag
    erroriAssolutiTag(indTag) = sqrt((xHatTag(indTag)-cTag(indTag,1))^2+(yHatTag(indTag)-cTag(indTag,2))^2);
end
erroreAssolutoRobot = sqrt((x_r-xVett(end))^2+(y_r-yVett(end))^2);
erroreTraiettoria = zeros(nPassi,1);

% Calcolo errore assoluto stima posizione robot nei vari passi
for k = 1:nPassi
    erroreTraiettoria(k) = sqrt((xVett(k)-xHatSLAM(1,k))^2+(yVett(k)-xHatSLAM(2,k))^2);
end

if displayErrori
    disp(distanzeRobotVere)
    disp(distanzeRobotStimate)
    disp(distanzeInterTagVere)
    disp(distanzeInterTagStimate)
    disp(erroriAssolutiTag)
    disp(erroreAssolutoRobot)
end