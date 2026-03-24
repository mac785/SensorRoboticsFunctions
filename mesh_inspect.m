addpath('.');
robot = KR120_params();
MESH_SCALE=1.0; MESH_ALPHA=0.28; MESH_COLOR=[0.55 0.72 0.90];

% KR120 R2500 Pro — URDF mesh origins are all rpy="0 0 0", so no rotation
% corrections are needed. Positions are the world-frame link origins at home.
d1=0.675; a1=0.350; a2=1.150; a3=1.000; dz=-0.041;
% Wrist centre world position at home: [a1+a2+a3, 0, d1+dz] = [2.500, 0, 0.634]
wc = [a1+a2+a3; 0; d1+dz];

mesh_T0={
    eye(4);                                    % base_link: origin at world [0,0,0]
    [eye(3), [0;0;d1];        0,0,0,1];        % link_1:   at joint_a1 = [0,0,d1]
    [eye(3), [a1;0;d1];       0,0,0,1];        % link_2:   at joint_a2 = [a1,0,d1]
    [eye(3), [a1+a2;0;d1];    0,0,0,1];        % link_3:   at joint_a3 = [a1+a2,0,d1]
    [eye(3), wc;               0,0,0,1];        % link_4:   at joint_a4 (wrist centre)
    [eye(3), wc;               0,0,0,1];        % link_5:   at joint_a5 (same wrist centre)
    [eye(3), wc;               0,0,0,1];        % link_6:   at joint_a6 (same wrist centre)
};
files={'base_link','link_1','link_2','link_3','link_4','link_5','link_6'};
meshes=cell(1,7);
for mi=1:7; meshes{mi}=stlread(['meshes/',files{mi},'.stl']); end

thetalist = zeros(6,1);
T_fr=cell(1,7); T_fr{1}=eye(4);
for i=1:6; T_fr{i+1}=T_fr{i}*MatrixExp6(vecToSE3(robot.Slist(:,i)*thetalist(i))); end
q_pts=zeros(3,6);
for i=1:6; q_pts(:,i)=T_fr{i}(1:3,1:3)*robot.q_joints(:,i)+T_fr{i}(1:3,4); end
T_ee=T_fr{7}*robot.M; p_ee=T_ee(1:3,4);
link_pts=[q_pts,p_ee];

views = {[0 0],'Front (XZ)'; [-90 0],'Side (YZ)'; [-35 25],'Isometric'; [0 90],'Top (XY)'};

fig=figure('Color','w','Visible','off','Position',[0 0 1600 900]);
for vi=1:4
    ax=subplot(1,4,vi,'Parent',fig);
    hold(ax,'on'); axis(ax,'equal'); grid(ax,'on');
    view(ax, views{vi,1});
    set(ax,'Color','w','XColor','k','YColor','k','ZColor','k','GridColor',[0.82 0.82 0.82]);
    xlabel(ax,'X'); ylabel(ax,'Y'); zlabel(ax,'Z');
    title(ax, views{vi,2}, 'Color','k','FontSize',10);
    for mi=1:7
        T=T_fr{mi}*mesh_T0{mi}; R=T(1:3,1:3); t=T(1:3,4);
        V=bsxfun(@plus,R*(meshes{mi}.Points*MESH_SCALE)',t)';
        patch(ax,'Faces',meshes{mi}.ConnectivityList,'Vertices',V, ...
              'FaceColor',MESH_COLOR,'FaceAlpha',MESH_ALPHA,'EdgeColor','none');
    end
    plot3(ax,link_pts(1,:),link_pts(2,:),link_pts(3,:),'k-','LineWidth',2.5);
    plot3(ax,q_pts(1,:),q_pts(2,:),q_pts(3,:),'o','MarkerSize',6, ...
          'Color',[0.2 0.2 0.2],'MarkerFaceColor',[0.45 0.45 0.45]);
    plot3(ax,p_ee(1),p_ee(2),p_ee(3),'s','MarkerSize',10, ...
          'Color',[0.85 0.1 0.1],'MarkerFaceColor',[0.85 0.1 0.1]);
end
sgtitle(fig,'KR120 R2500 Pro — Home (theta=0)','Color','k','FontSize',13,'FontWeight','bold');
saveas(fig,'fig_inspect_kr120_home.png');
close(fig);
fprintf('Saved fig_inspect_kr120_home.png\n');
