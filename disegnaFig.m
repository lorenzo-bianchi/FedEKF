% figure
plot([0 L/100],[0 0],'k','Linewidth',2)
hold on
% axis([-1*(L/100)/100 (L/100)+1*(L/100)/100 -1*(L/100)/100 (L/100)+1*(L/100)/100])
axis equal
plot([0 (L/100)],[(L/100) (L/100)],'k','Linewidth',2)
plot([0 0],[0 (L/100)],'k','Linewidth',2)
plot([(L/100) (L/100)],[0 (L/100)],'k','Linewidth',2)
plot(xVett/100,yVett/100,':','Linewidth',1);
plot(xVett(1)/100,yVett(1)/100,'k^','Linewidth',2)
plot(xVett(end)/100,yVett(end)/100,'ks','Linewidth',2)
for indTag = 1:nTag
    % disCerchio(cTag(indTag,1)/100,cTag(indTag,2)/100,rangeAntenna/100);
    % plot(cTag(indTag,1)/100,cTag(indTag,2)/100,'rH');
    plot(cTag(indTag,1)/100,cTag(indTag,2)/100,'rh','Linewidth',2,'MarkerSize',12);
end
