%% ik_animation_rst.m
% Presentation animation: IK convergence with live manipulability metrics.
% RST variant — uses importrobot() + show() for 3D rendering with DAE visual meshes
% instead of the manual STL patch overlay used in ik_animation.m.
%
% Requires: Robotics System Toolbox, load_kr120_rst.m, meshes/*.dae
%
% Panels:
%   Left        3D robot (RST show()) + linear velocity manipulability ellipsoid
%   Top-right   Condition number κ = σ_max/σ_min  (log scale)
%   Mid-right   Isotropy index   ι = σ_min/σ_max  (linear)
%   Bot-right   Angular and linear error convergence (log scale)
%
% USAGE:
%   >> ik_animation_rst
% Or set METHOD before running (e.g. from run_all_ik_rst.m):
%   METHOD = 'DLS'; ik_animation_rst;

close all;
addpath('.');

%% ========================================================================
%%  CONFIGURATION — edit here to switch method / scenario
%% ========================================================================

if ~exist('METHOD','var'), METHOD = 'NR'; end  % override from run_all_ik_rst if pre-set
RECORD      = true;
VIDEO_FILE  = ['ik_anim_rst_' lower(METHOD) '.avi'];
FPS         = 8;
STRIDE      = 1;
HOLD_FRAMES = 14;

robot      = KR120_params();
THETA_0    = zeros(6,1);
THETA_GOAL = [pi/2; -pi/4; pi/3; pi/4; pi/4; pi/6];

%% ---- Load RST robot (DAE visual meshes via importrobot) ----
fprintf('Loading RST robot model...\n');
rbt = load_kr120_rst();

% Method-specific IK solver options
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

anim_idx  = unique([1 : STRIDE : N_iter, N_iter]);
frame_seq = [anim_idx, repmat(N_iter, 1, HOLD_FRAMES)];
fprintf('Animation: %d frames from %d iterations (stride=%d).\n', ...
        numel(frame_seq), N_iter, STRIDE);

%% ========================================================================
%%  STEP 2: Pre-compute axis bounds for stable plots
%% ========================================================================

all_kappa   = min([history.kappa], 1e6);
all_iso     = [history.iso];
all_omg     = [history.omg_err];
all_lin     = [history.lin_err];
all_vol_lin = [history.vol_lin];
all_vol_ang = [history.vol_ang];

kappa_ylim = [max(0.9, min(all_kappa)*0.7),  min(max(all_kappa)*2, 1e6)];
iso_ylim   = [0,  max(all_iso)*1.2 + 0.005];
err_ylim   = [min([all_omg, all_lin])*0.4,  max([all_omg, all_lin])*2];
vol_ylim   = [0,  max([all_vol_lin, all_vol_ang])*1.2 + 0.01];

%% ========================================================================
%%  STEP 3: Figure and axes layout
%% ========================================================================

fig = figure('Color','w', 'Position',[30 30 1920 1080], ...
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
ax_kap = axes('Parent',fig, 'Position',[0.58 0.77 0.40 0.18]);
hold(ax_kap,'on'); grid(ax_kap,'on');
set(ax_kap,'YScale','log','XTickLabel',{},'FontSize',8, ...
    'YTick',10.^(0:6), 'YMinorTick','on','XColor','k','YColor','k', ...
    'GridColor',[0.85 0.85 0.85],'GridAlpha',1.0);
ylabel(ax_kap,'\kappa','Color','k');
title(ax_kap,'Condition Number  \kappa = \sigma_{max} / \sigma_{min}','FontSize',8,'Color','k');
xlim(ax_kap,[1, max(N_iter,2)]); ylim(ax_kap, kappa_ylim);
yline(ax_kap, 1, '--', 'Color',[0.7 0.7 0.7], 'LineWidth',0.8);

% --- Mid-upper-right: isotropy ---
ax_iso = axes('Parent',fig, 'Position',[0.58 0.55 0.40 0.18]);
hold(ax_iso,'on'); grid(ax_iso,'on');
set(ax_iso,'XTickLabel',{},'FontSize',8,'XColor','k','YColor','k', ...
    'GridColor',[0.85 0.85 0.85],'GridAlpha',1.0);
ylabel(ax_iso,'\iota','Color','k');
title(ax_iso,'Isotropy Index  \iota = \sigma_{min} / \sigma_{max}','FontSize',8,'Color','k');
xlim(ax_iso,[1, max(N_iter,2)]); ylim(ax_iso, iso_ylim);

% --- Mid-lower-right: ellipsoid volumes ---
ax_vol = axes('Parent',fig, 'Position',[0.58 0.33 0.40 0.18]);
hold(ax_vol,'on'); grid(ax_vol,'on');
set(ax_vol,'XTickLabel',{},'FontSize',8,'XColor','k','YColor','k', ...
    'GridColor',[0.85 0.85 0.85],'GridAlpha',1.0);
ylabel(ax_vol,'Vol.','Color','k');
title(ax_vol,'Ellipsoid Volume: linear (—)  angular (- -)','FontSize',8,'Color','k');
xlim(ax_vol,[1, max(N_iter,2)]); ylim(ax_vol, vol_ylim);

% --- Bot-right: error convergence ---
ax_err = axes('Parent',fig, 'Position',[0.58 0.10 0.40 0.18]);
hold(ax_err,'on'); grid(ax_err,'on');
set(ax_err,'YScale','log','FontSize',8, ...
    'YTick',10.^(-6:2), 'YMinorTick','on','XColor','k','YColor','k', ...
    'GridColor',[0.85 0.85 0.85],'GridAlpha',1.0);
xlabel(ax_err,'Iteration','Color','k'); ylabel(ax_err,'Error','Color','k');
title(ax_err,'Convergence: v_{err} (—) and \omega_{err} (- -)','FontSize',8,'Color','k');
xlim(ax_err,[1, max(N_iter,2)]); ylim(ax_err, err_ylim);
yline(ax_err, 1e-3, '--', 'Color',[0.7 0.7 0.7], 'LineWidth',0.8, ...
      'Label','tol','LabelHorizontalAlignment','left','FontSize',8);

%% ========================================================================
%%  STEP 4: Static 3D elements + timing annotation
%% ========================================================================

total_s     = history(end).elapsed_s;
ms_per_iter = total_s / N_iter * 1000;
annotation(fig, 'textbox', [0.06 0.02 0.44 0.08], ...
    'String', sprintf('Solve time: %.3f s     %.3f ms / iteration     %d iterations', ...
                      total_s, ms_per_iter, N_iter), ...
    'EdgeColor','none','BackgroundColor','none','Color','k', ...
    'FontSize',9,'HorizontalAlignment','center','VerticalAlignment','middle');

% Goal marker and legend are drawn inside draw_rst_and_ellipsoid each frame
% (required because PreservePlot=false clears axes children on every show() call)
p_goal = T_desired(1:3,4);

%% ========================================================================
%%  STEP 5: Animated graphics handles
%% ========================================================================

h_kap_line = plot(ax_kap, nan,nan, '-', 'Color',CLR, 'LineWidth',1.8);
h_kap_dot  = plot(ax_kap, nan,nan, 'o', 'Color',CLR, 'MarkerFaceColor',CLR,'MarkerSize',8);

h_iso_line = plot(ax_iso, nan,nan, '-', 'Color',CLR, 'LineWidth',1.8);
h_iso_dot  = plot(ax_iso, nan,nan, 'o', 'Color',CLR, 'MarkerFaceColor',CLR,'MarkerSize',8);

h_omg_line = plot(ax_err, nan,nan, '--', 'Color',CLR, 'LineWidth',1.8);
h_lin_line = plot(ax_err, nan,nan, '-',  'Color',CLR, 'LineWidth',1.8);
h_omg_dot  = plot(ax_err, nan,nan, 'o',  'Color',CLR, 'MarkerFaceColor',CLR,'MarkerSize',7);
h_lin_dot  = plot(ax_err, nan,nan, 's',  'Color',CLR, 'MarkerFaceColor',CLR,'MarkerSize',7);
legend(ax_err, 'off');

h_vlin_line = plot(ax_vol, nan,nan, '-',  'Color',CLR, 'LineWidth',1.8);
h_vang_line = plot(ax_vol, nan,nan, '--', 'Color',CLR, 'LineWidth',1.8);
h_vlin_dot  = plot(ax_vol, nan,nan, 'o',  'Color',CLR, 'MarkerFaceColor',CLR,'MarkerSize',7);
h_vang_dot  = plot(ax_vol, nan,nan, 's',  'Color',CLR, 'MarkerFaceColor',CLR,'MarkerSize',7);

h_rst      = [];   % RST show() handle, replaced each frame
h_ell      = [];   % linear ellipsoid surf handle, replaced each frame
h_ell_ang  = [];   % angular ellipsoid surf handle, replaced each frame

%% ========================================================================
%%  STEP 6: Video writer
%% ========================================================================

if RECORD
    fig.Position = [30 30 1920 1080];   % lock size before VideoWriter measures frame
    drawnow;
    vw = VideoWriter(VIDEO_FILE, 'Motion JPEG AVI');
    vw.FrameRate = FPS;
    vw.Quality   = 100;
    open(vw);
    fprintf('Recording → %s\n', VIDEO_FILE);
end

%% ========================================================================
%%  STEP 7: Main animation loop
%% ========================================================================

M_an  = numel(anim_idx);
iters = nan(1, M_an); kapps = nan(1, M_an);
isos  = nan(1, M_an); omgs  = nan(1, M_an); lins = nan(1, M_an);
vols_lin = nan(1, M_an); vols_ang = nan(1, M_an);
ai    = 0;

for fi = 1:numel(frame_seq)
    si = frame_seq(fi);
    s  = history(si);

    if fi <= M_an
        ai = ai + 1;
        iters(ai)    = si;
        kapps(ai)    = min(s.kappa, 1e6);
        isos(ai)     = s.iso;
        omgs(ai)     = s.omg_err;
        lins(ai)     = s.lin_err;
        vols_lin(ai) = s.vol_lin;
        vols_ang(ai) = s.vol_ang;
    end

    % --- Refresh 3D view ---
    % h_ell/h_ell_ang deleted explicitly; h_rst managed via cla() inside draw function
    delete(h_ell);
    delete(h_ell_ang);
    [h_rst, h_ell, h_ell_ang] = draw_rst_and_ellipsoid(ax3d, robot, rbt, s.theta, CLR, p_goal);

    % --- Status title ---
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

    % --- Update metric lines ---
    set(h_kap_line,'XData',iters(1:ai),'YData',kapps(1:ai));
    set(h_kap_dot, 'XData',iters(ai), 'YData',kapps(ai));
    set(h_iso_line,'XData',iters(1:ai),'YData',isos(1:ai));
    set(h_iso_dot, 'XData',iters(ai), 'YData',isos(ai));
    set(h_omg_line,'XData',iters(1:ai),'YData',omgs(1:ai));
    set(h_lin_line,'XData',iters(1:ai),'YData',lins(1:ai));
    set(h_omg_dot, 'XData',iters(ai), 'YData',omgs(ai));
    set(h_lin_dot, 'XData',iters(ai), 'YData',lins(ai));
    set(h_vlin_line,'XData',iters(1:ai),'YData',vols_lin(1:ai));
    set(h_vang_line,'XData',iters(1:ai),'YData',vols_ang(1:ai));
    set(h_vlin_dot, 'XData',iters(ai), 'YData',vols_lin(ai));
    set(h_vang_dot, 'XData',iters(ai), 'YData',vols_ang(ai));

    % Close any stray figures created by show(), then restore current figure
    % without raising the window (figure() causes WM resize events)
    all_figs = findall(0, 'Type', 'figure');
    for fj = 1:numel(all_figs)
        if all_figs(fj) ~= fig
            close(all_figs(fj));
        end
    end
    set(0, 'CurrentFigure', fig);
    drawnow;

    if RECORD
        f = getframe(fig);
        if size(f.cdata,2) ~= 1400 || size(f.cdata,1) ~= 750
            f.cdata = imresize(f.cdata, [1080 1920]);
        end
        writeVideo(vw, f);
    end
end

if RECORD
    close(vw);
    fprintf('Saved: %s\n', VIDEO_FILE);
end

%% ========================================================================
%%  LOCAL FUNCTIONS
%% ========================================================================

function [h_rst, h_ell, h_ell_ang] = draw_rst_and_ellipsoid(ax, robot, rbt, theta, ell_color, p_goal)
% Render the robot via RST show() and overlay linear + angular velocity ellipsoids.
% PreservePlot=false clears axes children each call, so the goal marker and
% legend are re-added here every frame.

    EDISP = 0.75;
    N_ell = 24;

    %% Clear all descendants of the axes (including nested hgtransform groups from show())
    %  Preserve axis label text objects so xlabel/ylabel/zlabel survive the wipe.
    kids   = findall(ax);
    keep   = [ax; ax.XLabel; ax.YLabel; ax.ZLabel];
    delete(kids(~ismember(kids, keep)));
    hold(ax, 'on');
    camlight(ax, 'headlight');

    %% RST robot render — PreservePlot=true keeps the axes alive
    before = ax.Children;
    show(rbt, theta(:)', 'Parent', ax, ...
         'PreservePlot', true, 'Visuals','on', 'Frames','off');
    after  = ax.Children;
    % Hide all objects added by show() from the legend
    new_objs = setdiff(after, before);
    set(new_objs, 'HandleVisibility', 'off');
    h_rst = gobjects(0);   % cleanup is handled by cla() above, not delete()

    %% Re-add goal marker and legend (cleared by cla)
    plot3(ax, p_goal(1),p_goal(2),p_goal(3), 'r*', ...
          'MarkerSize',14,'LineWidth',2.5,'DisplayName','Target EE');
    surf(ax, nan(2),nan(2),nan(2), 'FaceAlpha',0.35,'EdgeColor','none', ...
         'FaceColor',ell_color,'DisplayName','Lin. vel. ellipsoid');
    surf(ax, nan(2),nan(2),nan(2), 'FaceAlpha',0,'EdgeColor',ell_color, ...
         'FaceColor','none','DisplayName','Ang. vel. ellipsoid');
    legend(ax,'Location','northwest','FontSize',8,'Color','w','TextColor','k','EdgeColor','k');

    %% End-effector position via our FK (consistent with IK math)
    T_ee = FK_body(robot.M, robot.Blist, theta);
    p_ee = T_ee(1:3,4);

    %% EE marker — red square on top of RST render
    plot3(ax, p_ee(1),p_ee(2),p_ee(3), 's', ...
          'MarkerSize',11,'Color',[0.85 0.10 0.10], ...
          'MarkerFaceColor',[0.85 0.10 0.10],'HandleVisibility','off');

    Js = J_space(robot.Slist, theta);

    [az_s, el_s] = meshgrid(linspace(0,2*pi,N_ell), linspace(-pi/2,pi/2,N_ell));
    x_u = [cos(el_s(:)).*cos(az_s(:)), ...
           cos(el_s(:)).*sin(az_s(:)), ...
           sin(el_s(:))]';

    %% Linear velocity manipulability ellipsoid
    Jv          = Js(4:6, :);
    [U_v,S_v,~] = svd(Jv, 'econ');
    sv          = diag(S_v);
    sc_v        = EDISP / max(sv(1), 1e-6);
    pts_v = U_v * diag(sv*sc_v) * x_u + p_ee;
    Xe = reshape(pts_v(1,:), N_ell, N_ell);
    Ye = reshape(pts_v(2,:), N_ell, N_ell);
    Ze = reshape(pts_v(3,:), N_ell, N_ell);
    h_ell = surf(ax, Xe,Ye,Ze, 'FaceAlpha',0.35,'EdgeColor','none', ...
                 'FaceColor',ell_color,'HandleVisibility','off');

    %% Angular velocity manipulability ellipsoid
    Jw          = Js(1:3, :);
    [U_w,S_w,~] = svd(Jw, 'econ');
    sw          = diag(S_w);
    sc_w        = EDISP / max(sw(1), 1e-6);
    pts_w = U_w * diag(sw*sc_w) * x_u + p_ee;
    Xa = reshape(pts_w(1,:), N_ell, N_ell);
    Ya = reshape(pts_w(2,:), N_ell, N_ell);
    Za = reshape(pts_w(3,:), N_ell, N_ell);
    h_ell_ang = surf(ax, Xa,Ya,Za, 'FaceAlpha',0,'EdgeColor',ell_color, ...
                     'FaceColor','none','HandleVisibility','off');
end
