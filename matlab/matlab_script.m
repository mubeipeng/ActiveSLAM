g2o_filename='topology_graph.g2o';
% for i=1:length(tag)
%     R(:,:,i) = reshape(tag(i,5:13),3,3);
% end
% [x y z]=dcm2angle(R,'XYZ');

%%
fid = fopen(g2o_filename,'w');
odom_cov_vec = [50 0 0 50 0 100]; % Vectorized version of the odometry information matrices
landmark_pos_cov = [10 0 10];

edge_str = 'EDGE2';
edge_format_str = '%s %i %i %10.7f %10.7f %10.7f %10.7f %10.7f %10.7f %10.7f %10.7f %10.7f\n';

landmark_pos_edge_str = 'LANDMARK';
landmark_pos_edge_format_str = '%s %i %i %10.7f %10.7f %10.7f %10.7f %10.7f\n';

jj=1;
id = 0;
for ii = 1:length(odom)
    fprintf(fid,edge_format_str,edge_str,ii-1,ii,odom(ii,2), odom(ii,3), odom(ii,4), ...
        odom_cov_vec(1),odom_cov_vec(2),odom_cov_vec(3),odom_cov_vec(4), ...
        odom_cov_vec(5),odom_cov_vec(6));
    id = [id ii];
    while(jj<=length(tag) && tag(jj,1) == odom(ii,1))
        fprintf(fid,landmark_pos_edge_format_str,landmark_pos_edge_str,...            
            ii,tag(jj,2)+1000, tag(jj,3), tag(jj,4), ...
            landmark_pos_cov(1),landmark_pos_cov(2),landmark_pos_cov(3));

%         fprintf(fid,edge_format_str,edge_str,ii,tag(jj,2)+1000,tag(jj,3), tag(jj,4), y(jj), ...
%             odom_cov_vec(1),odom_cov_vec(2),odom_cov_vec(3),odom_cov_vec(4), ...
%             odom_cov_vec(5),odom_cov_vec(6));        
        id = [id tag(jj,2)+1000];
        jj=jj+1;        
    end
end
fclose(fid); clear odom_cov_ve landmark_pos_cov edge_str edge_format_str...
    landmark_pos_edge_str landmark_pos_edge_format_str;
id = unique(id,'stable');
%%

system(['~/Downloads/isam/bin/isam -B -W ',g2o_filename,' ',g2o_filename]);

%%
pose2d = [];
fid = fopen('topology_graph.g2o');
line = fgets(fid);
while line~=-1
    if line(1:12)=='Point2d_Node'
        line(line=='(')=[];
        line(line==')')=[];
        pose2d = [pose2d; str2num(line(13:end))];
    end
    line=fgets(fid);
end
idx = id>=1000;
% landmarks = pose2d(idx,:);
% landmarks(:,1)=id(idx)-1000;


%% read landmark wall size
fid = fopen('acl_neighbor.txt');
line = fgets(fid);
lm=[];
while line~=-1
    line(line=='(' | line==')')=[];
    lm = [lm; str2num(line)];
    line = fgets(fid);
end
fclose(fid);

%% find neighbors
for i=1:size(laser,1)
    component = laser(i,4:4:end);
    tag_i = tag(tag(:,1)==laser(i,1),1:3);
end

%% plot
plot(pose2d(:,2),pose2d(:,3),'sr'); axis equal; hold on;
% plot(pose2d(idx,2),pose2d(idx,3),'sr'); axis equal;
% hold on; plot(pose2d(~idx,2),pose2d(~idx,3),'k')

% quiver(landmarks(:,2),landmarks(:,3),cos(landmarks(:,4)),sin(landmarks(:,4)),'m');

for i=1:length(landmarks)
    ss = lm(lm(:,1)==landmarks(i,1),5:6);
    x = [landmarks(i,2) - ss(2)*cos(landmarks(i,4)+pi/2) landmarks(i,2) + ss(1)*cos(landmarks(i,4)+pi/2)];
    y = [landmarks(i,3) - ss(2)*sin(landmarks(i,4)+pi/2) landmarks(i,3) + ss(1)*sin(landmarks(i,4)+pi/2)];
    plot(x,y);    
end
    
% neighbors = load('neighbors.txt');
% for i=1:length(neighbors)
%     idx = landmarks(:,1)==neighbors(i,1)|landmarks(:,1)==neighbors(i,2);
%     plot(landmarks(idx,2),landmarks(idx,3),'g');
% end

%%
color = colormap('lines');

for i=1:size(laser,1)
    laser(i,laser(i,:)>100)=0;
    scatter(laser(i,2:4:end), laser(i,3:4:end),[],color(laser(i,4:4:end),:)); hold on; axis equal;
    
    tag_idx = find(tag(:,1)==laser(i,1)); tag_pos = ceil(tag(tag_idx,3));
    for j=1:length(tag_idx)
        quiver(tag(tag_idx(j),3),tag(tag_idx(j),4),cos(y(tag_idx(j))),sin(y(tag_idx(j))),'r');
    end
    hold off;
    pause;    
end