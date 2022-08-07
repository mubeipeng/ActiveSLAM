plot(lm(:,4),lm(:,5),'ro')
hold on;
for i=1:size(lm,1)
    text(lm(i,4),lm(i,5),num2str(lm(i,1)));
end

for i=1:length(nb)
    plot(nb(i,1:2),nb(i,3:4))
end

%%
samples=[];
fid = fopen('samples.txt');
line = fgetl(fid);
i=1;
while line~=-1
    data = str2num(line);
    samples(i,1:2)=data(1:2);
    visible(i).xy=data(1:2);
    visible(i).I = data(3);
    visible(i).lm=data(4:end);
    i=i+1;
    line = fgetl(fid);
end
fclose(fid);

%%
plot(samples(:,1),samples(:,2),'mx');
samples(end+1,:)=[0,0];
for i=1:length(graph)
    plot(samples(graph(i,:)+1,1),samples(graph(i,:)+1,2),'g');
end

%%
I = [visible.I];
figure;plot([visible.I],'.')
idx = find(I>90);

%%
n = length(samples);
Adaj = zeros(n,n);
g = [];
for i=1:n
    for j=i+1:n;
        if checkVisibility(samples(i,1),samples(i,2),samples(j,1),samples(j,2),lm,nb)
            Adaj(i,j) = norm(samples(i,:)-samples(j,:));
            Adaj(j,i) = Adaj(i,j);
            g = [g; [i j]];
        end
    end
end

for i=1:length(g)
    plot(samples(g(i,:),1),samples(g(i,:),2),'g');
end

%%
idx = find(abs(samples(:,1)+1.527)<1e-3);
 while path(1,idx(end))~=-1
    idx=[idx,path(1,idx(end))+1];
end