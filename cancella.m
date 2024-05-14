figure
hold on
axis equal
grid on

robot = 1;

xVett = percorsi(:, 1, robot);
yVett = percorsi(:, 2, robot);
plot(xVett, yVett, ':', 'Linewidth', 1)

a = ekfs(robot).xHatSLAM(1:2, :);       % a locale
plot(a(1, :), a(2, :), 'Linewidth', 2)
a = [a; ones(1, nPassi)];
T = TsGL(:, :, robot);
b = T*a;                % b globale
plot(b(1, :), b(2, :), 'Linewidth', 2)
legend("trajG", "trajL", "boh")

plot([0 L], [0 0], 'k', 'Linewidth', 2)
plot([0 L], [L L], 'k', 'Linewidth', 2)
plot([0 0], [0 L], 'k', 'Linewidth', 2)
plot([L L], [0 L], 'k', 'Linewidth', 2)