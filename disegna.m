for robot = 1:nRobot
    figure(robot)
    hold off

    plot([0 L/100], [0 0], 'k', 'Linewidth', 2)
    hold on
    plot([0 L/100], [L/100 L/100], 'k', 'Linewidth', 2)
    plot([0 0], [0 L/100], 'k', 'Linewidth', 2)
    plot([L/100 L/100], [0 L/100], 'k', 'Linewidth', 2)
    for indTag = 1:nTag
        % plot(cTag(indTag,1)/100, cTag(indTag,2)/100, 'rH');
        plot(cTag(indTag,1)/100, cTag(indTag,2)/100, 'rh', 'Linewidth', 2, 'MarkerSize', 12);
    end

    xVett = percorsi(:, 1, robot);
    yVett = percorsi(:, 2, robot);

    plot(xVett/100, yVett/100, ':', 'Linewidth', 1)    % traiettoria
    plot(xVett(1)/100, yVett(1)/100, 'k^', 'Linewidth', 2)  % pos iniziale
    plot(xVett(end)/100, yVett(end)/100, 'ks', 'Linewidth', 2) % pos finale

    title(num2str(k))
    plot(xVett(max(1,k-30):k)/100, yVett(max(1,k-30):k)/100, 'k', 'LineWidth', 2)   % traccia
    plot(xVett(k)/100, yVett(k)/100, 'ko')  % pos attuale vera
    plot(ekfs(robot).xHatSLAM(1,k)/100, ekfs(robot).xHatSLAM(2,k)/100, 'o') % pos attuale stimata
    for indTag = 1:nTag
        x_i = ekfs(robot).xHatSLAM(4+(3+nPhi)*(indTag-1), k);
        y_i = ekfs(robot).xHatSLAM(5+(3+nPhi)*(indTag-1), k);
        rho_i = ekfs(robot).xHatSLAM(6+(3+nPhi)*(indTag-1),k);
        x_ti = 0;
        y_ti = 0;
        for jndPhi = 1:nPhi
            phi_ij = ekfs(robot).xHatSLAM(6+(3+nPhi)*(indTag-1)+jndPhi,k);
            cosPhi_ij = cos(phi_ij);
            sinPhi_ij = sin(phi_ij);
            xTag_ij = x_i + rho_i*cosPhi_ij;
            yTag_ij = y_i + rho_i*sinPhi_ij;
            x_ti = x_ti + xTag_ij*ekfs(robot).pesi(indTag,jndPhi);
            y_ti = y_ti + yTag_ij*ekfs(robot).pesi(indTag,jndPhi);
            plot(xTag_ij/100, yTag_ij/100, 'm.', 'MarkerSize', max(1,ceil(10*ekfs(robot).pesi(indTag,jndPhi))))
        end
        plot(x_ti/100, y_ti/100, '*') % posizione media stimata tag
    end
    % for indTag = 1:nTag,
    %     if k<nPassi,
    %         figure(2+indTag)
    %         bar(pesi(indTag,:))
    %     end
    % end
    
    axis equal
    axis([0 2 0 2])
end
pause(0.01)