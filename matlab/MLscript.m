nb=load('nb.txt');
lm=load('lm.txt');
 
%%
hold on; axis equal; axis off;
for i=1:length(nb)
    plot(lm(nb(i,:),4),lm(nb(i,:),5),'k','LineWidth',4);
end
for i=1:length(lm)
    if(lm(i,2)+lm(i,3)<2)
        plot(lm(i,4),lm(i,5),'bp','MarkerSize',12,'MarkerFaceColor','b');
    else
        plot(lm(i,4),lm(i,5),'kp','MarkerSize',12,'MarkerFaceColor','k');
    end
%     text(lm(i,4)+0.2,lm(i,5)+0.35,num2str(lm(i,1)));
end


%%
samples=load('log.txt');

cmap = jet(50);
u=samples(:,3);
urange = (max(u) - min(u))/(size(cmap,1)-1);
ucolor = round( (u-repmat(min(u),size(u,1),1))/urange)+1;
scatter(samples(:,1),samples(:,2),[],cmap(ucolor,:));