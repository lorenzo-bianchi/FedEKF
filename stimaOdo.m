xOdo = zeros(nPassi,1);
yOdo = zeros(nPassi,1);
thetaOdo = zeros(nPassi,1);

xOdo(1) = xVett(1);
yOdo(1) = yVett(1);
thetaOdo(1) = thetaVett(1);

for k = 1:nPassi-1
    
    uk = ((uRe(k)+uLe(k))/2);
    omegak = ((uRe(k)-uLe(k))/d);
    
    xOdo(k+1) = xOdo(k) + uk*cos(thetaOdo(k));
    yOdo(k+1) = yOdo(k) + uk*sin(thetaOdo(k));
    thetaOdo(k+1) = thetaOdo(k) + omegak;
    
end

erroreOdo = zeros(nPassi,1);
for k = 1:nPassi
    erroreOdo(k) = sqrt((xVett(k)-xOdo(k))^2+(yVett(k)-yOdo(k))^2);
end
figure
plot(erroreOdo)
% plot(xOdo/100,yOdo/100,'m:')
