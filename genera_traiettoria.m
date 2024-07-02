dati;

passi_traj = 10000;
percorsi = zeros(passi_traj, 5, nRobot);
for robot = 1:nRobot
    percorsi(:, :, robot) = percorsoRandom(data, passi_traj, seed + 10*robot);     % xVett, yVett, thetaVett, uRe, uLe
end
save('percorsi.mat', 'percorsi');
