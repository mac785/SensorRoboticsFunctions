%% gen_presentation_figs.m
% Generates all THA3 presentation graphics and saves as PNG files.
% Run from the THA3/ directory.

clear; clc; clear cd;
addpath(fullfile(fileparts(mfilename('fullpath')), '..'));

out_dir = fullfile(fileparts(mfilename('fullpath')), 'presentation_graphics');
if ~exist(out_dir, 'dir'), mkdir(out_dir); end

% Force white theme regardless of MATLAB dark mode setting
set(groot, 'defaultFigureColor',    [1 1 1]);
set(groot, 'defaultAxesColor',      [1 1 1]);
set(groot, 'defaultTextColor',      [0 0 0]);
set(groot, 'defaultAxesXColor',     [0 0 0]);
set(groot, 'defaultAxesYColor',     [0 0 0]);
set(groot, 'defaultAxesZColor',     [0 0 0]);
set(groot, 'defaultAxesGridColor',  [0.15 0.15 0.15]);

function fix_axes(ax)
    set(ax, 'Color','w', 'XColor','k', 'YColor','k', 'ZColor','k', ...
        'GridColor',[0.3 0.3 0.3], 'MinorGridColor',[0.5 0.5 0.5]);
end

function lg = wlegend(varargin)
    lg = legend(varargin{:}, 'Color','w', 'TextColor','k', 'EdgeColor',[0.6 0.6 0.6]);
end

%% =========================================================
%  FIGURE 1: PA2 Comparison Bar Chart
% =========================================================
fig1 = figure('Position', [50 50 1800 700], 'Color', 'w');

labels = {'Clean (N=10)', 'Noisy (N=10)', 'Noisy (N=5)'};
colors = [0.22 0.50 0.80; 0.95 0.60 0.10; 0.82 0.22 0.22];

rot_err   = [0,        0.000333, 0.006216];
trans_err = [0,        0.000628, 0.148007];
axb_res   = [0.344146, 0.344205, 0.430885];

titles_sp = {'Rotation Difference', 'Translation Difference', 'Mean AX=XB Residual'};
ylabels   = {'||R_{clean}-R_x||_F', '||p_{clean}-p_x||_2', 'Mean ||AX-XB||_F'};
data_sp   = {rot_err, trans_err, axb_res};

for k = 1:3
    sp = subplot(1,3,k);
    b = bar(data_sp{k}, 'FaceColor', 'flat', 'EdgeColor', 'none');
    for c = 1:3, b.CData(c,:) = colors(c,:); end
    set(sp, 'XTickLabel', labels, 'XTick', 1:3, 'FontSize', 10, 'Color', 'w', ...
        'XColor','k','YColor','k');
    xtickangle(15);
    ylabel(ylabels{k}, 'FontSize', 10);
    title(titles_sp{k}, 'FontSize', 11, 'FontWeight', 'bold');
    grid on; box off;
    if k == 3, ylim([0 0.52]); end
end

sgtitle('PA2 Eye-in-Hand: Effect of Noise and Reduced Configurations', ...
    'FontSize', 13, 'FontWeight', 'bold', 'Color', 'k');

exportgraphics(fig1, fullfile(out_dir, 'fig1_pa2_comparison.png'), 'Resolution', 96);
fprintf('[1/5] Saved fig1_pa2_comparison.png\n');

%% =========================================================
%  FIGURE 2: EM Distortion — C_expected vs C_measured
% =========================================================
[~, d_a, a_a, c_a] = cal_body_data('HW3-PA1/pa1-debug-a-calbody.txt');
[~, fr_a]           = cal_readings_data('HW3-PA1/pa1-debug-a-calreadings.txt');
[~, d_g, a_g, c_g] = cal_body_data('HW3-PA1/pa1-debug-g-calbody.txt');
[~, fr_g]           = cal_readings_data('HW3-PA1/pa1-debug-g-calreadings.txt');

function C_exp = compute_Cexp(d, a, c, frame)
    F_D  = cloud2cloud(d, frame.D);
    F_A  = cloud2cloud(a, frame.A);
    F_Di = inv_transform(F_D);
    C_exp = zeros(size(c,1), 3);
    for j = 1:size(c,1)
        pt = F_Di * (F_A * [c(j,:)'; 1]);
        C_exp(j,:) = pt(1:3)';
    end
end

C_exp_a  = compute_Cexp(d_a, a_a, c_a, fr_a{1});
C_meas_a = fr_a{1}.C;
C_exp_g  = compute_Cexp(d_g, a_g, c_g, fr_g{1});
C_meas_g = fr_g{1}.C;

fig2 = figure('Position', [50 50 1800 950], 'Color', 'w');

subtitles2 = {'Dataset-a  (Clean,  max err = 0.007 mm)', ...
              'Dataset-g  (Distorted,  max err = 4.74 mm)'};
Ce_list = {C_exp_a, C_exp_g};
Cm_list = {C_meas_a, C_meas_g};

h_exp = []; h_meas = []; ax2_handles = gobjects(1,2);
for col = 1:2
    axc = subplot(1,2,col);
    ax2_handles(col) = axc;
    Ce = Ce_list{col};  Cm = Cm_list{col};

    hold on;
    for j = 1:size(Ce,1)
        h = plot3([Ce(j,1) Cm(j,1)], [Ce(j,2) Cm(j,2)], [Ce(j,3) Cm(j,3)], ...
            'k-', 'LineWidth', 0.8);
        h.Color(4) = 0.35;
        h.HandleVisibility = 'off';
    end
    hs_exp  = scatter3(Ce(:,1), Ce(:,2), Ce(:,3), 60, [0.15 0.45 0.80], 'filled');
    hs_meas = scatter3(Cm(:,1), Cm(:,2), Cm(:,3), 60, [0.85 0.20 0.20], '^', 'filled');
    hold off;

    fix_axes(axc);
    xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
    title(subtitles2{col}, 'FontSize', 13, 'FontWeight', 'bold');
    grid on; axis equal; view(35, 22);

    if col == 1
        h_exp = hs_exp;  h_meas = hs_meas;
    end
end

% Push subplots up to open clear space at the bottom for the legend
set(ax2_handles(1), 'Position', [0.06  0.18  0.42  0.72]);
set(ax2_handles(2), 'Position', [0.54  0.18  0.42  0.72]);

% Single shared legend — horizontal strip at bottom center
lg2 = legend([h_exp, h_meas], {'C_{expected}  (optical)', 'C_{measured}  (EM sensor)'}, ...
    'Color','w', 'TextColor','k', 'EdgeColor',[0.6 0.6 0.6], 'FontSize', 13, ...
    'Orientation', 'horizontal');
lg2.Units = 'normalized';
lg2.Position = [0.30 0.01 0.40 0.05];

sgtitle('EM Distortion: C_{expected} vs C_{measured}', ...
    'FontSize', 14, 'FontWeight', 'bold', 'Color', 'k');

exportgraphics(fig2, fullfile(out_dir, 'fig2_em_distortion.png'), 'Resolution', 96);
fprintf('[2/5] Saved fig2_em_distortion.png\n');

%% =========================================================
%  FIGURE 3: Point Cloud Registration — 2-panel
% =========================================================
[~, d, a, ~] = cal_body_data('HW3-PA1/pa1-debug-a-calbody.txt');
[~, fr3]     = cal_readings_data('HW3-PA1/pa1-debug-a-calreadings.txt');

D_meas = fr3{1}.D;
F_D    = cloud2cloud(d, D_meas);
d_xfm  = (F_D * [d'; ones(1,size(d,1))])';
d_xfm  = d_xfm(:,1:3);

% Mean-center for "before" panel
d_c = d     - mean(d,1);
D_c = D_meas - mean(D_meas,1);

fig3 = figure('Position', [50 50 1400 900], 'Color', 'w');

% Panel 1: mean-centered — blue (reference) first, then red (measured)
ax31 = subplot(1,2,1);
hold on;
hs3_ref = scatter3(d_c(:,1), d_c(:,2), d_c(:,3), 90, [0.15 0.45 0.80], 'filled');
hs3_meas = scatter3(D_c(:,1), D_c(:,2), D_c(:,3), 90, [0.85 0.20 0.20], '^', 'filled');
hold off;
fix_axes(ax31);
title('Before Registration (Mean-Centered)', 'FontSize', 13, 'FontWeight', 'bold');
xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
grid on; axis equal; view(35, 22);

% Panel 2: after — blue (transformed reference) first, then red (measured)
ax32 = subplot(1,2,2);
hold on;
scatter3(d_xfm(:,1), d_xfm(:,2), d_xfm(:,3), 90, [0.15 0.45 0.80], 'o', 'filled');
scatter3(D_meas(:,1), D_meas(:,2), D_meas(:,3), 90, [0.85 0.20 0.20], '^', 'filled');
for j = 1:size(D_meas,1)
    h = plot3([d_xfm(j,1) D_meas(j,1)], [d_xfm(j,2) D_meas(j,2)], ...
              [d_xfm(j,3) D_meas(j,3)], 'k--', 'LineWidth', 1.0);
    h.Color(4) = 0.40;
    h.HandleVisibility = 'off';
end
hold off;
fix_axes(ax32);
title('After Registration (F_D Applied)', 'FontSize', 13, 'FontWeight', 'bold');
xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
grid on; axis equal; view(35, 22);

% Push subplots up to open clear space at the bottom for the legend
set(ax31, 'Position', [0.06  0.18  0.42  0.72]);
set(ax32, 'Position', [0.54  0.18  0.42  0.72]);

% Single shared legend — horizontal strip at bottom center
lg3 = legend([hs3_ref, hs3_meas], ...
    {'d_{body}  (known reference)', 'D_{measured}  (optical tracker)'}, ...
    'Color','w', 'TextColor','k', 'EdgeColor',[0.6 0.6 0.6], 'FontSize', 13, ...
    'Orientation', 'horizontal');
lg3.Units = 'normalized';
lg3.Position = [0.23 0.01 0.54 0.05];

sgtitle('cloud2cloud SVD Registration: Dataset-a, Frame 1', ...
    'FontSize', 14, 'FontWeight', 'bold', 'Color', 'k');

exportgraphics(fig3, fullfile(out_dir, 'fig3_point_cloud_registration.png'), 'Resolution', 96);
fprintf('[3/5] Saved fig3_point_cloud_registration.png\n');

%% =========================================================
%  FIGURE 3.1: Alternative Registration — EM probe, ~180 deg rotation
%  Uses empivot G markers: frame 1 vs frame 2 (179.75 deg rotation)
%  Much more dramatic before/after than the near-zero-rotation D example.
% =========================================================
[~, ~, G3] = empivot_data('HW3-PA1/pa1-debug-a-empivot.txt');
G3_ref  = G3{1};
G3_fr2  = G3{2};

% Mean-center both for "before" panel
G3_ref_c = G3_ref - mean(G3_ref, 1);
G3_fr2_c = G3_fr2 - mean(G3_fr2, 1);

% Solve registration and apply for "after" panel
F3alt    = cloud2cloud(G3_ref, G3_fr2);
G3_xfm   = (F3alt * [G3_ref'; ones(1, size(G3_ref,1))])';
G3_xfm   = G3_xfm(:,1:3);

fig3b = figure('Position', [50 50 1400 900], 'Color', 'w');

% Panel 1: before — same shape, very different orientation (~180 deg)
ax3b1 = subplot(1,2,1);
hold on;
hs3b_ref  = scatter3(G3_ref_c(:,1), G3_ref_c(:,2), G3_ref_c(:,3), ...
    110, [0.15 0.45 0.80], 'filled');
hs3b_meas = scatter3(G3_fr2_c(:,1), G3_fr2_c(:,2), G3_fr2_c(:,3), ...
    110, [0.85 0.20 0.20], '^', 'filled');
hold off;
fix_axes(ax3b1);
title('Before Registration  (\approx180\circ rotation)', 'FontSize', 13, 'FontWeight', 'bold');
xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
grid on; axis equal; view(35, 22);

% Panel 2: after — G_ref transformed onto G_fr2
ax3b2 = subplot(1,2,2);
hold on;
scatter3(G3_xfm(:,1), G3_xfm(:,2), G3_xfm(:,3), ...
    110, [0.15 0.45 0.80], 'o', 'filled');
scatter3(G3_fr2(:,1),  G3_fr2(:,2),  G3_fr2(:,3), ...
    110, [0.85 0.20 0.20], '^', 'filled');
for j = 1:size(G3_fr2,1)
    h = plot3([G3_xfm(j,1) G3_fr2(j,1)], [G3_xfm(j,2) G3_fr2(j,2)], ...
              [G3_xfm(j,3) G3_fr2(j,3)], 'k--', 'LineWidth', 1.0);
    h.Color(4) = 0.40;
    h.HandleVisibility = 'off';
end
hold off;
fix_axes(ax3b2);
title('After Registration  (SVD solved)', 'FontSize', 13, 'FontWeight', 'bold');
xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
grid on; axis equal; view(35, 22);

% Push subplots up for legend space
set(ax3b1, 'Position', [0.06  0.18  0.42  0.72]);
set(ax3b2, 'Position', [0.54  0.18  0.42  0.72]);

% Shared legend
lg3b = legend([hs3b_ref, hs3b_meas], ...
    {'g_{body}  (frame 1 reference)', 'G_{measured}  (frame 2, EM tracker)'}, ...
    'Color','w', 'TextColor','k', 'EdgeColor',[0.6 0.6 0.6], 'FontSize', 13, ...
    'Orientation', 'horizontal');
lg3b.Units = 'normalized';
lg3b.Position = [0.20 0.01 0.60 0.05];

sgtitle('cloud2cloud SVD Registration: EM Probe Markers, Frames 1 \rightarrow 2  (\approx180\circ)', ...
    'FontSize', 14, 'FontWeight', 'bold', 'Color', 'k');

exportgraphics(fig3b, fullfile(out_dir, 'fig3_1_registration_alt.png'), 'Resolution', 96);
fprintf('[3.1] Saved fig3_1_registration_alt.png\n');

%% =========================================================
%  FIGURE 4: Pivot Calibration Sweep
% =========================================================
[~, ~, G_frames] = empivot_data('HW3-PA1/pa1-debug-a-empivot.txt');
N_piv = numel(G_frames);
G_ref = G_frames{1};

[P_dim, t_tip] = pivot_cal(G_frames);

centroids = zeros(N_piv, 3);
tip_est   = zeros(N_piv, 3);
for k = 1:N_piv
    centroids(k,:) = mean(G_frames{k}, 1);
    F_k = cloud2cloud(G_ref, G_frames{k});
    pt  = F_k * [t_tip; 1];
    tip_est(k,:) = pt(1:3)';
end

fig4 = figure('Position', [50 50 1100 900], 'Color', 'w');
ax4 = axes(fig4);
hold on;

% One color per frame from the 'lines' colormap
cmap = lines(N_piv);

for k = 1:N_piv
    col = cmap(k,:);
    pts = G_frames{k};
    ctr = centroids(k,:);

    % Markers colored by frame (hidden from legend)
    s = scatter3(pts(:,1), pts(:,2), pts(:,3), 28, col, 'filled');
    s.HandleVisibility = 'off';

    % Thin black lines from each marker to its centroid
    for m = 1:size(pts,1)
        h = plot3([pts(m,1) ctr(1)], [pts(m,2) ctr(2)], [pts(m,3) ctr(3)], ...
            'k-', 'LineWidth', 0.5);
        h.Color(4) = 0.35;
        h.HandleVisibility = 'off';
    end
end

% Blue lines from centroid to P_dimple (hidden from legend)
for k = 1:N_piv
    h = plot3([centroids(k,1) P_dim(1)], [centroids(k,2) P_dim(2)], ...
              [centroids(k,3) P_dim(3)], '-', 'Color', [0.30 0.55 0.85, 0.45], ...
              'LineWidth', 1.0);
    h.HandleVisibility = 'off';
end

% Named objects (appear in legend)
scatter3(centroids(:,1), centroids(:,2), centroids(:,3), 90, ...
    cmap, 'filled', 'DisplayName', 'Probe body (centroid)');
scatter3(tip_est(:,1), tip_est(:,2), tip_est(:,3), 35, ...
    [0.95 0.60 0.10], 'filled', 'DisplayName', 'Estimated tip per frame');
scatter3(P_dim(1), P_dim(2), P_dim(3), 280, [0.85 0.15 0.15], ...
    'p', 'filled', 'DisplayName', 'P_{dimple} (solved)');

hold off;
fix_axes(ax4);
xlabel('X (mm)', 'FontSize', 11);
ylabel('Y (mm)', 'FontSize', 11);
zlabel('Z (mm)', 'FontSize', 11);
title({'Pivot Calibration: EM Probe — Dataset-a', ...
    sprintf('P_{dimple} = [%.2f, %.2f, %.2f] mm', P_dim(1), P_dim(2), P_dim(3))}, ...
    'FontSize', 13, 'FontWeight', 'bold');
wlegend('Location', 'northeast', 'FontSize', 10);
grid on; view(30, 22);

exportgraphics(fig4, fullfile(out_dir, 'fig4_pivot_calibration.png'), 'Resolution', 96);
fprintf('[4/5] Saved fig4_pivot_calibration.png\n');

%% =========================================================
%  FIGURE 5: R_x Frame Visualization (PA2)
% =========================================================
R_x = [-0.00320  0.99999 -0.00011;
        -0.00077  0.00011  1.00000;
         0.99999  0.00320  0.00077];

fig5 = figure('Position', [50 50 1100 900], 'Color', 'w');
ax5 = axes(fig5);
hold on;

sc = 0.75;
ax_colors = {[0.85 0.10 0.10], [0.10 0.65 0.10], [0.10 0.10 0.85]};
ee_labels  = {'X_{EE}', 'Y_{EE}', 'Z_{EE}'};
cam_labels = {'X_{cam}', 'Y_{cam}', 'Z_{cam}'};
ee_dirs  = eye(3);         % columns = EE x,y,z
cam_dirs = R_x;            % columns = camera x,y,z axes expressed in EE frame
off = [2.2 0 0];

for iax = 1:3
    % End-effector frame (solid thick)
    quiver3(0,0,0, ee_dirs(1,iax)*sc, ee_dirs(2,iax)*sc, ee_dirs(3,iax)*sc, ...
        'Color', ax_colors{iax}, 'LineWidth', 3.5, 'MaxHeadSize', 0.5, ...
        'AutoScale', 'off', 'HandleVisibility', 'off');
    ep = ee_dirs(:,iax) * (sc + 0.08);
    text(ep(1), ep(2), ep(3), ee_labels{iax}, 'FontSize', 12, ...
        'Color', ax_colors{iax}, 'FontWeight', 'bold');

    % Camera frame (dashed)
    cdir = cam_dirs(:,iax);
    quiver3(off(1),off(2),off(3), cdir(1)*sc, cdir(2)*sc, cdir(3)*sc, ...
        'Color', ax_colors{iax}, 'LineWidth', 3.5, 'MaxHeadSize', 0.5, ...
        'AutoScale', 'off', 'LineStyle', ':', 'HandleVisibility', 'off');
    ep2 = off' + cdir * (sc + 0.08);
    text(ep2(1), ep2(2), ep2(3), cam_labels{iax}, 'FontSize', 12, ...
        'Color', ax_colors{iax}, 'FontWeight', 'bold');
end

% Frame labels below each origin
text(0,   0,   -0.22, 'End-Effector Frame', 'FontSize', 11, ...
    'HorizontalAlignment', 'center', 'FontWeight', 'bold', 'Color', 'k');
text(off(1), off(2), -0.22, 'Camera Frame  (X)', 'FontSize', 11, ...
    'HorizontalAlignment', 'center', 'FontWeight', 'bold', 'Color', 'k');

% Arrow between frames
annotation('arrow', [0.46 0.54], [0.50 0.50], 'LineWidth', 2, 'Color', 'k', ...
    'HeadWidth', 10, 'HeadLength', 10);

% Solid/dashed legend proxies
plot3(NaN,NaN,NaN, 'k-',  'LineWidth', 3, 'DisplayName', 'End-Effector axes');
plot3(NaN,NaN,NaN, 'k:',  'LineWidth', 3, 'DisplayName', 'Camera axes  (X·EE)');

hold off;
fix_axes(ax5);
xlim([-0.2 3.2]); ylim([-1.1 1.1]); zlim([-0.35 1.1]);
xlabel('X'); ylabel('Y'); zlabel('Z');
title({'PA2: Solved X  (Camera \rightarrow End-Effector)', ...
    'Clean data, N=10 configurations'}, 'FontSize', 12, 'FontWeight', 'bold', 'Color', 'k');
wlegend('Location', 'southeast', 'FontSize', 11);
grid on; view(20, 22); axis equal;

exportgraphics(fig5, fullfile(out_dir, 'fig5_Rx_frame.png'), 'Resolution', 96);
fprintf('[5/5] Saved fig5_Rx_frame.png\n');

fprintf('\nAll figures saved to: %s\n', out_dir);
