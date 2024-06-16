for robot = 1:nRobot
    fprintf('Robot %d:\n', robot)
    for indTag = 1:nTag
        nPhi = ekfs(robot).nPhiVett(indTag);
        ind0 = ekfs(robot).xHatCumIndices(indTag+1);
        x_i   = ekfs(robot).xHatSLAM(0+ind0, end);
        y_i   = ekfs(robot).xHatSLAM(1+ind0, end);
        rho_i = ekfs(robot).xHatSLAM(2+ind0, end);

        phi_ij = ekfs(robot).xHatSLAM(2+ind0+indPhi, end);
        cosPhi_ij = cos(phi_ij);
        sinPhi_ij = sin(phi_ij);
        xTag_ij = x_i + rho_i*cosPhi_ij;
        yTag_ij = y_i + rho_i*sinPhi_ij;
        pesoTag = ekfs(robot).pesi(indTag, indPhi);

        ind_x = 0+ind0;
        ind_y = 1+ind0;
        ind_r = 2+ind0;
        ind_p = 3+ind0;

        varXi  = ekfs(robot).P(ind_x, ind_x);
        varYi  = ekfs(robot).P(ind_y, ind_y);
        varRho = ekfs(robot).P(ind_r, ind_r);
        varPhi = ekfs(robot).P(ind_p, ind_p);

        covXiYi   = ekfs(robot).P(ind_x, ind_y);
        covXRho   = ekfs(robot).P(ind_x, ind_r);
        covXPhi   = ekfs(robot).P(ind_x, ind_p);
        covYRho   = ekfs(robot).P(ind_y, ind_r);
        covYPhi   = ekfs(robot).P(ind_y, ind_p);
        covRhoPhi = ekfs(robot).P(ind_r, ind_p);

        varX = varXi + cosPhi_ij^2 * varRho + rho_i^2 * sinPhi_ij^2 * varPhi + ...
               2*cosPhi_ij*covXRho - 2*rho_i*sinPhi_ij*covXPhi - 2*rho_i*cosPhi_ij*sinPhi_ij*covRhoPhi;

        varY = varYi + sinPhi_ij^2 * varRho + rho_i^2 * cosPhi_ij^2 * varPhi + ...
               2*sinPhi_ij*covYRho + 2*rho_i*cosPhi_ij*covYPhi + 2*rho_i*cosPhi_ij*sinPhi_ij*covRhoPhi;

        fprintf('\tTag %d:\tsigmaX = %f, sigmaY = %f\n', indTag, sqrt(varX), sqrt(varY))
    end
    fprintf('\n')
end