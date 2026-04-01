%% ik_animation.m
% Presentation animation: IK convergence with live manipulability metrics.
%
% Produces a 4-panel figure animated over IK iterations and saves it as an
% AVI video. Panels:
%   Left        3D robot arm + linear velocity manipulability ellipsoid
%   Top-right   Condition number κ = σ_max/σ_min  (log scale)
%   Mid-right   Isotropy index   ι = σ_min/σ_max  (linear)
%   Bot-right   Angular and linear error convergence (log scale)
%
% USAGE:
%   Edit the CONFIGURATION section below, then run:
%     >> ik_animation
%
% To produce videos for all four methods, change METHOD each time and re-run.

clc; close all;
addpath('.');

%% ========================================================================
%%  CONFIGURATION — edit here to switch method / scenario
%% ========================================================================

if ~exist('METHOD','var'), METHOD = 'NR'; end  % override from run_all_ik if pre-set
RECORD      = true;      % true = save .avi video
VIDEO_FILE  = ['ik_anim_' lower(METHOD) '.avi'];
FPS         = 8;         % output video frame rate
STRIDE      = 1;         % show every Nth iteration  (set >1 to thin JT frames)
HOLD_FRAMES = 14;        % extra frames held on final converged pose

robot      = KR120_params();
THETA_0    = zeros(6,1);                              % home configuration (all joints zero)
THETA_GOAL = [pi/2; -pi/4; pi/3; pi/4; pi/4; pi/6]; % goal: 90 deg J1 sweep + arm bend

%% ---- Mesh overlay (matches robot_animation.m) ----
MESH_SCALE = 1.0;
MESH_ALPHA = 0.22;
MESH_COLOR = [0.55 0.72 0.90];   % steel-blue tint

md1=0.675; ma1=0.350; ma2=1.150; ma3=1.000; mdz=-0.041;
mwc = [ma1+ma2+ma3; 0; md1+mdz];
mesh_T0 = {
    eye(4);
    [eye(3), [0;0;md1];        0,0,0,1];
    [eye(3), [ma1;0;md1];      0,0,0,1];
    [eye(3), [ma1+ma2;0;md1];  0,0,0,1];
    [eye(3), mwc;               0,0,0,1];
    [eye(3), mwc;               0,0,0,1];
    [eye(3), mwc;               0,0,0,1];
};
clear md1 ma1 ma2 ma3 mdz mwc;

mesh_names = {'base_link','link_1','link_2','link_3','link_4','link_5','link_6'};
meshes = cell(1,7);
SHOW_MESH = false;
for mi = 1:7
    mfp = fullfile('meshes', [mesh_names{mi}, '.stl']);
    if exist(mfp, 'file')
        meshes{mi} = stlread(mfp);
        SHOW_MESH  = true;
    end
end
clear mi mfp;
if SHOW_MESH
    fprintf('Mesh overlay enabled (%d / 7 files loaded).\n', sum(~cellfun(@isempty,meshes)));
end

% Method-specific IK solver options (passed directly to collect_ik_history)
switch METHOD
    case 'NR',  IK_OPTS = {};
    case 'JT',  IK_OPTS = {'alpha',0.1,'max_iter',500}; STRIDE = 8;
    case 'RR',  IK_OPTS = {'k0',5};
    case 'DLS', IK_OPTS = {'lambda_max',0.1,'sigma_thresh',0.05};
end

% Colour and label per method
switch METHOD
    case 'NR',  CLR = [0.13 0.47 0.71]; LABEL = 'Newton-Raphson IK';
    case 'JT',  CLR = [0.89 0.47 0.10]; LABEL = 'Jacobian Transpose IK';
    case 'RR',  CLR = [0.17 0.63 0.17]; LABEL = 'Redundancy Resolution IK';
    case 'DLS', CLR = [0.58 0.18 0.58]; LABEL = 'Damped Least-Squares IK';
end

%% ========================================================================
%%  STEP 1: Collect per-iteration IK history
%% ========================================================================

T_desired = FK_body(robot.M, robot.Blist, THETA_GOAL);

fprintf('Running %s ...\n', LABEL);
history = collect_ik_history(robot, T_desired, THETA_0, METHOD, IK_OPTS{:});
N_iter  = numel(history);

% Frame index sequence: one frame per stride, then hold on final frame
anim_idx   = unique([1 : STRIDE : N_iter, N_iter]);
frame_seq  = [anim_idx, repmat(N_iter, 1, HOLD_FRAMES)];
fprintf('Animation: %d frames from %d iterations (stride=%d).\n', ...
        numel(frame_seq), N_iter, STRIDE);

%% ========================================================================
%%  STEP 2: Pre-compute axis bounds for stable plots
%% ========================================================================

all_kappa = min([history.kappa], 1e6);
all_iso   = [history.iso];
all_omg   = [history.omg_err];
all_lin   = [history.lin_err];

kappa_ylim = [max(0.9, min(all_kappa)*0.7),  min(max(all_kappa)*2, 1e6)];
iso_ylim   = [0,  max(all_iso)*1.2 + 0.005];
err_ylim   = [min([all_omg, all_lin])*0.4,  max([all_omg, all_lin])*2];

%% ========================================================================
%%  STEP 3: Figure and axes layout
%% ========================================================================

fig = figure('Color','w', 'Position',[30 30 1400 750], ...
             'ToolBar','none', 'MenuBar','none', 'Resize','off');

% --- Left: 3D robot view ---
ax3d = axes('Parent',fig, 'Position',[0.06 0.14 0.44 0.73]);
hold(ax3d,'on'); grid(ax3d,'on'); view(ax3d, 140, 22);
set(ax3d,'Color','w','GridColor',[0.5 0.5 0.5],'GridAlpha',1.0,'FontSize',9, ...
    'XColor','k','YColor','k','ZColor','k');
xlabel(ax3d,'X (m)'); ylabel(ax3d,'Y (m)'); zlabel(ax3d,'Z (m)');
xlim(ax3d,[-3.0 3.0]); ylim(ax3d,[-3.0 3.0]); zlim(ax3d,[-0.5 3.0]);
camlight(ax3d,'headlight');

% --- Top-right: condition number ---
ax_kap = axes('Parent',fig, 'Position',[0.58 0.71 0.40 0.24]);
hold(ax_kap,'on'); grid(ax_kap,'on');
set(ax_kap,'YScale','log','XTickLabel',{},'FontSize',9, ...
    'YTick',10.^(0:6), 'YMinorTick','on','XColor','k','YColor','k', ...
    'GridColor',[0.85 0.85 0.85],'GridAlpha',1.0);
ylabel(ax_kap,'\kappa','Color','k');
title(ax_kap,'Condition Number  \kappa = \sigma_{max} / \sigma_{min}','FontSize',9,'Color','k');
xlim(ax_kap,[1, max(N_iter,2)]); ylim(ax_kap, kappa_ylim);
yline(ax_kap, 1, '--', 'Color',[0.7 0.7 0.7], 'LineWidth',0.8);

% --- Mid-right: isotropy ---
ax_iso = axes('Parent',fig, 'Position',[0.58 0.41 0.40 0.24]);
hold(ax_iso,'on'); grid(ax_iso,'on');
set(ax_iso,'XTickLabel',{},'FontSize',9,'XColor','k','YColor','k', ...
    'GridColor',[0.85 0.85 0.85],'GridAlpha',1.0);
ylabel(ax_iso,'\iota','Color','k');
title(ax_iso,'Isotropy Index  \iota = \sigma_{min} / \sigma_{max}','FontSize',9,'Color','k');
xlim(ax_iso,[1, max(N_iter,2)]); ylim(ax_iso, iso_ylim);

% --- Bot-right: error convergence ---
ax_err = axes('Parent',fig, 'Position',[0.58 0.10 0.40 0.24]);
hold(ax_err,'on'); grid(ax_err,'on');
set(ax_err,'YScale','log','FontSize',9, ...
    'YTick',10.^(-6:2), 'YMinorTick','on','XColor','k','YColor','k', ...
    'GridColor',[0.85 0.85 0.85],'GridAlpha',1.0);
xlabel(ax_err,'Iteration','Color','k'); ylabel(ax_err,'Error','Color','k');
title(ax_err,'Convergence: \omega_{err} (—) and v_{err} (- -)','FontSize',9,'Color','k');
xlim(ax_err,[1, max(N_iter,2)]); ylim(ax_err, err_ylim);
yline(ax_err, 1e-3, '--', 'Color',[0.7 0.7 0.7], 'LineWidth',0.8, ...
      'Label','tol','LabelHorizontalAlignment','left','FontSize',8);

%% ========================================================================
%%  STEP 4: Static 3D elements (goal marker, arm legend proxy) + timing label
%% ========================================================================

% Timing annotation — fixed text below the 3D axes for the full video
total_s     = history(end).elapsed_s;
ms_per_iter = total_s / N_iter * 1000;
annotation(fig, 'textbox', [0.06 0.02 0.44 0.08], ...
    'String', sprintf('Solve time: %.3f s     %.2f ms / iteration     %d iterations', ...
                      total_s, ms_per_iter, N_iter), ...
    'EdgeColor','none','BackgroundColor','none','Color','k', ...
    'FontSize',9,'HorizontalAlignment','center','VerticalAlignment','middle');

p_goal = T_desired(1:3,4);
plot3(ax3d, p_goal(1),p_goal(2),p_goal(3), 'r*', ...
     'MarkerSize',14,'LineWidth',2.5,'DisplayName','Target EE');
plot3(ax3d, nan,nan,nan, '-', 'Color',CLR, 'LineWidth',3.5, 'DisplayName','Robot arm');
plot3(ax3d, nan,nan,nan, 's', 'Color','k', 'MarkerFaceColor',[0.9 0.2 0.2], ...
      'MarkerSize',9, 'DisplayName','Current EE');
h_ell_legend = surf(ax3d, nan(2),nan(2),nan(2), ...
      'FaceAlpha',0.35,'EdgeColor','none','FaceColor',CLR,'DisplayName','Lin. vel. ellipsoid');
legend(ax3d,'Location','northwest','FontSize',8);

%% ========================================================================
%%  STEP 5: Animated graphics handles (grow each frame)
%% ========================================================================

h_kap_line = plot(ax_kap, nan,nan, '-', 'Color',CLR, 'LineWidth',1.8);
h_kap_dot  = plot(ax_kap, nan,nan, 'o', 'Color',CLR, 'MarkerFaceColor',CLR,'MarkerSize',8);

h_iso_line = plot(ax_iso, nan,nan, '-', 'Color',CLR, 'LineWidth',1.8);
h_iso_dot  = plot(ax_iso, nan,nan, 'o', 'Color',CLR, 'MarkerFaceColor',CLR,'MarkerSize',8);

h_omg_line = plot(ax_err, nan,nan, '-',  'Color',CLR, 'LineWidth',1.8, 'DisplayName','\omega_{err}');
h_lin_line = plot(ax_err, nan,nan, '--', 'Color',CLR, 'LineWidth',1.8, 'DisplayName','v_{err}');
h_omg_dot  = plot(ax_err, nan,nan, 'o',  'Color',CLR, 'MarkerFaceColor',CLR,'MarkerSize',7);
h_lin_dot  = plot(ax_err, nan,nan, 's',  'Color',CLR, 'MarkerFaceColor',CLR,'MarkerSize',7);
legend(ax_err, 'off');

% 3D elements re-drawn each frame
h_robot = gobjects(0);
h_ell   = gobjects(0);

%% ========================================================================
%%  STEP 6: Video writer
%% ========================================================================

if RECORD
    drawnow;   % let figure settle to its final size before locking frame dimensions
    vw = VideoWriter(VIDEO_FILE, 'Motion JPEG AVI');
    vw.FrameRate = FPS;
    vw.Quality   = 90;
    open(vw);
    fprintf('Recording → %s\n', VIDEO_FILE);
end

%% ========================================================================
%%  STEP 7: Main animation loop
%% ========================================================================

M_an  = numel(anim_idx);
iters = nan(1, M_an); kapps = nan(1, M_an);
isos  = nan(1, M_an); omgs  = nan(1, M_an); lins = nan(1, M_an);
ai    = 0;   % index into the preallocated arrays

for fi = 1:numel(frame_seq)
    si = frame_seq(fi);
    s  = history(si);

    % Accumulate metric history on advancing frames (not on hold frames)
    if fi <= M_an
        ai = ai + 1;
        iters(ai) = si;
        kapps(ai) = min(s.kappa, 1e6);
        isos(ai)  = s.iso;
        omgs(ai)  = s.omg_err;
        lins(ai)  = s.lin_err;
    end

    % --- Refresh 3D view ---
    delete(h_robot);
    delete(h_ell);
    [h_robot, h_ell] = draw_robot_and_ellipsoid(ax3d, robot, s.theta, CLR, ...
                           meshes, mesh_T0, SHOW_MESH, MESH_SCALE, MESH_COLOR, MESH_ALPHA);

    % --- Status title on 3D axes ---
    if s.converged
        status_str = sprintf('Converged at iter %d / %d', si, N_iter);
    else
        status_str = sprintf('Iteration %d / %d', si, N_iter);
    end
    title(ax3d, ...
          {['\bf' LABEL '  \rm—  ' status_str], ...
           sprintf('\\omega_{err} = %.4f rad       v_{err} = %.4f m', ...
                   s.omg_err, s.lin_err)}, ...
          'FontSize',9,'Interpreter','tex','Color','k');

    % --- Update growing metric lines (slice to filled portion only) ---
    set(h_kap_line,'XData',iters(1:ai),'YData',kapps(1:ai));
    set(h_kap_dot, 'XData',iters(ai), 'YData',kapps(ai));
    set(h_iso_line,'XData',iters(1:ai),'YData',isos(1:ai));
    set(h_iso_dot, 'XData',iters(ai), 'YData',isos(ai));
    set(h_omg_line,'XData',iters(1:ai),'YData',omgs(1:ai));
    set(h_lin_line,'XData',iters(1:ai),'YData',lins(1:ai));
    set(h_omg_dot, 'XData',iters(ai), 'YData',omgs(ai));
    set(h_lin_dot, 'XData',iters(ai), 'YData',lins(ai));

    drawnow;

    if RECORD
        writeVideo(vw, getframe(fig));
    end
end

if RECORD
    close(vw);
    fprintf('Saved: %s\n', VIDEO_FILE);
end

%% ========================================================================
%%  LOCAL FUNCTIONS
%% ========================================================================

function [h_all, h_surf] = draw_robot_and_ellipsoid(ax, robot, theta, ell_color, ...
                               meshes, mesh_T0, show_mesh, mesh_scale, mesh_color, mesh_alpha)
% Draw robot skeleton, coordinate frames, STL meshes, and linear velocity
% manipulability ellipsoid. Matches the rendering style of robot_animation.m.
% Returns handle arrays for deletion on the next frame.

    EDISP  = 0.75;   % display size of largest ellipsoid semi-axis (metres)
    N_ell  = 24;     % sphere tessellation resolution
    scale  = norm(robot.M(1:3,4)) * 0.08;   % frame arrow length (matches robot_animation)
    n      = robot.n_dof;

    %% Partial FK transforms
    T_fr = cell(1, n+1);
    T_fr{1} = eye(4);
    for ii = 1:n
        T_fr{ii+1} = T_fr{ii} * MatrixExp6(vecToSE3(robot.Slist(:,ii) * theta(ii)));
    end
    T_ee = T_fr{n+1} * robot.M;
    p_ee = T_ee(1:3,4);

    q_pts = zeros(3, n);
    for ii = 1:n
        q_pts(:,ii) = T_fr{ii}(1:3,1:3)*robot.q_joints(:,ii) + T_fr{ii}(1:3,4);
    end

    % Preallocate: 7 meshes + 3 skeleton objects + n*3 quivers = 28 max
    h_all = gobjects(1, 7 + 3 + n*3);
    hi    = 0;

    %% Translucent STL mesh overlay (drawn first — underneath everything)
    if show_mesh
        for mi = 1:7
            if ~isempty(meshes{mi})
                hi = hi + 1;
                h_all(hi) = draw_mesh(ax, meshes{mi}, T_fr{mi} * mesh_T0{mi}, ...
                                      mesh_scale, mesh_color, mesh_alpha);
            end
        end
    end

    %% Linear velocity manipulability ellipsoid (drawn before skeleton)
    Js          = J_space(robot.Slist, theta);
    Jv          = Js(4:6, :);
    [U_e,S_e,~] = svd(Jv, 'econ');
    sv          = diag(S_e);
    sc          = EDISP / max(sv(1), 1e-6);

    [az_s, el_s] = meshgrid(linspace(0,2*pi,N_ell), linspace(-pi/2,pi/2,N_ell));
    x_u = [cos(el_s(:)).*cos(az_s(:)), ...
           cos(el_s(:)).*sin(az_s(:)), ...
           sin(el_s(:))]';

    pts_e = U_e * diag(sv*sc) * x_u + p_ee;
    Xe = reshape(pts_e(1,:), N_ell, N_ell);
    Ye = reshape(pts_e(2,:), N_ell, N_ell);
    Ze = reshape(pts_e(3,:), N_ell, N_ell);

    h_surf = surf(ax, Xe,Ye,Ze, 'FaceAlpha',0.35,'EdgeColor','none', ...
                  'FaceColor',ell_color,'HandleVisibility','off');

    %% Robot links — black, LineWidth 3 (matches robot_animation.m)
    link_pts = [q_pts, p_ee];
    hi = hi + 1;
    h_all(hi) = plot3(ax, link_pts(1,:),link_pts(2,:),link_pts(3,:), ...
                      'k-','LineWidth',3,'HandleVisibility','off');

    %% Joint markers — dark grey filled circles
    hi = hi + 1;
    h_all(hi) = plot3(ax, q_pts(1,:),q_pts(2,:),q_pts(3,:), 'o', ...
                      'MarkerSize',7,'Color',[0.25 0.25 0.25], ...
                      'MarkerFaceColor',[0.45 0.45 0.45],'HandleVisibility','off');

    %% End-effector marker — red square
    hi = hi + 1;
    h_all(hi) = plot3(ax, p_ee(1),p_ee(2),p_ee(3), 's', ...
                      'MarkerSize',11,'Color',[0.85 0.10 0.10], ...
                      'MarkerFaceColor',[0.85 0.10 0.10],'HandleVisibility','off');

    %% Joint coordinate frames — RGB quivers (matches robot_animation.m)
    fc = {[0.90 0.20 0.20], [0.15 0.70 0.20], [0.15 0.30 0.90]};
    for ii = 1:n
        Rf = T_fr{ii}(1:3,1:3);
        p  = q_pts(:,ii);
        for k = 1:3
            hi = hi + 1;
            h_all(hi) = quiver3(ax, p(1),p(2),p(3), ...
                                Rf(1,k)*scale, Rf(2,k)*scale, Rf(3,k)*scale, ...
                                'Color',fc{k},'LineWidth',1.0, ...
                                'MaxHeadSize',0.7,'AutoScale','off', ...
                                'HandleVisibility','off');
        end
    end

    h_all = h_all(1:hi);   % trim unused slots
end

function h = draw_mesh(ax, tr, T, sc, color, alpha)
% Render an STL mesh with rigid-body transform T (matches robot_animation.m).
    R = T(1:3,1:3);  t = T(1:3,4);
    V = bsxfun(@plus, R * (tr.Points * sc)', t)';
    h = patch(ax, 'Faces',tr.ConnectivityList,'Vertices',V, ...
              'FaceColor',color,'FaceAlpha',alpha, ...
              'EdgeColor','none','FaceLighting','gouraud', ...
              'HandleVisibility','off');
end
