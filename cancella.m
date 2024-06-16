figure
axis equal
hold on
grid on

for indTag = 1:nTag
    ind0 = obj.xHatCumIndices(indTag+1);
    x_i   = obj.xHatSLAMmeno(0+ind0);
    y_i   = obj.xHatSLAMmeno(1+ind0);
    rho_i = obj.xHatSLAMmeno(2+ind0);
    phi_ij = obj.xHatSLAMmeno(2+ind0+1);
    cosPhi_ij = cos(phi_ij);
    sinPhi_ij = sin(phi_ij);
    xTag_ij = x_i + rho_i*cosPhi_ij;
    yTag_ij = y_i + rho_i*sinPhi_ij;

    plot(xTag_ij, yTag_ij, 'bo', 'LineWidth', 2)
end

plot(measures_weighted(1, :), measures_weighted(2, :), 'ro', 'LineWidth', 2)