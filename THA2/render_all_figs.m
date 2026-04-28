%% render_all_figs.m
% Generates and saves all validation figures for the KR120 R2500 Pro.
% Each FK / ellipsoid visualisation function creates its own figure
% internally; we grab it with gcf, add the translucent mesh overlay,
% then export with print (reliable in headless -batch mode).
%
% Run from SensorRoboticsFunctions/:
%   matlab -batch "addpath('.'); run('render_all_figs.m')"

addpath('.');
OUT_DIR = 'media';
if ~exist(OUT_DIR, 'dir'), mkdir(OUT_DIR); end
robot = KR120_params();

%% ---- Load meshes once ----
MESH_SCALE = 1.0;
MESH_ALPHA = 0.22;
MESH_COLOR = [0.55 0.72 0.90];

d1=0.675; a1=0.350; a2=1.150; a3=1.000; dz=-0.041;
wc = [a1+a2+a3; 0; d1+dz];
mesh_T0 = {
    eye(4);                                 % base_link
    [eye(3), [0;0;d1];      0,0,0,1];      % link_1
    [eye(3), [a1;0;d1];     0,0,0,1];      % link_2
    [eye(3), [a1+a2;0;d1];  0,0,0,1];      % link_3
    [eye(3), wc;             0,0,0,1];      % link_4
    [eye(3), wc;             0,0,0,1];      % link_5
    [eye(3), wc;             0,0,0,1];      % link_6
};

mesh_names = {'base_link','link_1','link_2','link_3','link_4','link_5','link_6'};
meshes = cell(1,7);
for mi = 1:7
    fp = fullfile('meshes', [mesh_names{mi}, '.stl']);
    if exist(fp, 'file'), meshes{mi} = stlread(fp); end
end

%% ---- Configurations ----
theta_home = zeros(6,1);
theta_bent = [pi/6; pi/4; -pi/3; pi/4; pi/3; pi/6];
theta_ns   = [0.1; 0.2; -0.3; 0.4; 0.5; 0.6];

%% ---- (b) FK_space ----
FK_space(robot.M, robot.Slist, theta_home, true, robot.q_joints);
fig = gcf;  fig.Position = [0 0 1200 700];
title('(b) FK\_space — Home configuration (\theta = 0)');
add_meshes(gca, meshes, mesh_T0, theta_home, robot.Slist, MESH_SCALE, MESH_COLOR, MESH_ALPHA);
print(fig, fullfile(OUT_DIR,'fig_b_fkspace_home'), '-dpng', '-r150');  close(fig);
fprintf('Saved %s\n', fullfile(OUT_DIR,'fig_b_fkspace_home.png'));

FK_space(robot.M, robot.Slist, theta_bent, true, robot.q_joints);
fig = gcf;  fig.Position = [0 0 1200 700];
title('(b) FK\_space — Bent configuration');
add_meshes(gca, meshes, mesh_T0, theta_bent, robot.Slist, MESH_SCALE, MESH_COLOR, MESH_ALPHA);
print(fig, fullfile(OUT_DIR,'fig_b_fkspace_bent'), '-dpng', '-r150');  close(fig);
fprintf('Saved %s\n', fullfile(OUT_DIR,'fig_b_fkspace_bent.png'));

%% ---- (c) FK_body ----
FK_body(robot.M, robot.Blist, theta_home, true, robot.q_joints);
fig = gcf;  fig.Position = [0 0 1200 700];
title('(c) FK\_body — Home configuration (\theta = 0)');
add_meshes(gca, meshes, mesh_T0, theta_home, robot.Slist, MESH_SCALE, MESH_COLOR, MESH_ALPHA);
print(fig, fullfile(OUT_DIR,'fig_c_fkbody_home'), '-dpng', '-r150');  close(fig);
fprintf('Saved %s\n', fullfile(OUT_DIR,'fig_c_fkbody_home.png'));

FK_body(robot.M, robot.Blist, theta_bent, true, robot.q_joints);
fig = gcf;  fig.Position = [0 0 1200 700];
title('(c) FK\_body — Bent configuration');
add_meshes(gca, meshes, mesh_T0, theta_bent, robot.Slist, MESH_SCALE, MESH_COLOR, MESH_ALPHA);
print(fig, fullfile(OUT_DIR,'fig_c_fkbody_bent'), '-dpng', '-r150');  close(fig);
fprintf('Saved %s\n', fullfile(OUT_DIR,'fig_c_fkbody_bent.png'));

%% ---- (g) Ellipsoids — non-singular ----
ellipsoid_plot_angular(robot, theta_ns);
fig = gcf;  fig.Position = [0 0 1200 700];
title('(g) Angular Velocity Manipulability Ellipsoid — Non-singular');
add_meshes(gca, meshes, mesh_T0, theta_ns, robot.Slist, MESH_SCALE, MESH_COLOR, MESH_ALPHA);
print(fig, fullfile(OUT_DIR,'fig_g_ellipsoid_angular_ns'), '-dpng', '-r150');  close(fig);
fprintf('Saved %s\n', fullfile(OUT_DIR,'fig_g_ellipsoid_angular_ns.png'));

ellipsoid_plot_linear(robot, theta_ns);
fig = gcf;  fig.Position = [0 0 1200 700];
title('(g) Linear Velocity Manipulability Ellipsoid — Non-singular');
add_meshes(gca, meshes, mesh_T0, theta_ns, robot.Slist, MESH_SCALE, MESH_COLOR, MESH_ALPHA);
print(fig, fullfile(OUT_DIR,'fig_g_ellipsoid_linear_ns'), '-dpng', '-r150');  close(fig);
fprintf('Saved %s\n', fullfile(OUT_DIR,'fig_g_ellipsoid_linear_ns.png'));

%% ---- (g) Ellipsoids — bent ----
ellipsoid_plot_angular(robot, theta_bent);
fig = gcf;  fig.Position = [0 0 1200 700];
title('(g) Angular Velocity Manipulability Ellipsoid — Bent config');
add_meshes(gca, meshes, mesh_T0, theta_bent, robot.Slist, MESH_SCALE, MESH_COLOR, MESH_ALPHA);
print(fig, fullfile(OUT_DIR,'fig_g_ellipsoid_angular_bent'), '-dpng', '-r150');  close(fig);
fprintf('Saved %s\n', fullfile(OUT_DIR,'fig_g_ellipsoid_angular_bent.png'));

ellipsoid_plot_linear(robot, theta_bent);
fig = gcf;  fig.Position = [0 0 1200 700];
title('(g) Linear Velocity Manipulability Ellipsoid — Bent config');
add_meshes(gca, meshes, mesh_T0, theta_bent, robot.Slist, MESH_SCALE, MESH_COLOR, MESH_ALPHA);
print(fig, fullfile(OUT_DIR,'fig_g_ellipsoid_linear_bent'), '-dpng', '-r150');  close(fig);
fprintf('Saved %s\n', fullfile(OUT_DIR,'fig_g_ellipsoid_linear_bent.png'));

%% ---- Mesh inspection (4-view headless) ----
run('mesh_inspect.m');

fprintf('\nAll figures saved.\n');

%% ---- Helper: add translucent mesh overlay to an existing axes ----
function add_meshes(ax, meshes, mesh_T0, thetalist, Slist, sc, color, alpha)
    n = length(thetalist);
    T_fr = cell(1, n+1);  T_fr{1} = eye(4);
    for i = 1:n
        T_fr{i+1} = T_fr{i} * MatrixExp6(vecToSE3(Slist(:,i) * thetalist(i)));
    end
    for mi = 1:7
        if ~isempty(meshes{mi})
            T = T_fr{mi} * mesh_T0{mi};
            R = T(1:3,1:3);  t = T(1:3,4);
            V = bsxfun(@plus, R * (meshes{mi}.Points * sc)', t)';
            patch(ax, 'Faces', meshes{mi}.ConnectivityList, 'Vertices', V, ...
                  'FaceColor', color, 'FaceAlpha', alpha, 'EdgeColor', 'none', ...
                  'FaceLighting', 'gouraud', 'HandleVisibility', 'off');
        end
    end
    camlight(ax, 'headlight');
end
