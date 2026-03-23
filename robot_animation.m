%% robot_animation.m
% Animates the KUKA KR210 tracing four geometric shapes in 3-D space.
%
%   1. Circle  — closed loop in the YZ plane
%   2. Square  — closed loop with right-angle corners in the YZ plane
%   3. Helix   — two rising turns along Z
%   4. Snake   — right-angle staircase path in the YZ plane
%
% For each shape an interactive figure is opened showing:
%   - Dotted blue line  : full desired path
%   - Solid cyan line   : EE trace so far
%   - Black arm         : current robot pose
%   - Red square marker : end-effector tip
%
% Run from the SensorRoboticsFunctions directory:
%   >> robot_animation

clc; close all;
addpath('.');
robot = KR210_params();

%% ---- Common settings ----
% EE orientation: pointing in -X (toward base) throughout
R_des = [-1  0  0;
          0 -1  0;
          0  0  1];

% Warm-start joint configuration (good manipulability near workspace centre)
theta_init = [0; pi/6; -pi/3; 0; pi/4; 0];

% DLS IK parameters
IK_LAMBDA   = 0.15;
IK_SIGMA    = 0.05;
IK_EOMG     = 1e-3;
IK_EV       = 1e-3;
IK_MAXITER  = 300;

N_PER_SEG   = 12;   % waypoints per straight/arc segment (IK density)
N_INTERP    = 30;   % animation frames interpolated between IK waypoints
                    %   total frames per shape ≈ N_segments * N_PER_SEG * N_INTERP

% ---- Quick test mode ----
% Set FAST_TEST = true to cut IK waypoints and interpolation for fast preview.
% Set FAST_TEST = false for full-quality animation/recording.
FAST_TEST = true;
if FAST_TEST
    N_PER_SEG = 4;
    N_INTERP  = 8;
end

% ---- Recording ----
% Set RECORD = true to save each shape as an MP4 in the current directory.
% Set RECORD = false to watch the animation live without saving.
RECORD    = false;
VID_FPS   = 30;     % playback frames per second

%% ---- Shape definitions ----
% All shapes live at x = 2.4 m (arm facing the YZ plane at that depth).
% Adjust cx/r/h to stay well inside the KR210 workspace.
cx = 2.4;

% --- 1. Circle ---
r_c = 0.45;  cy_c = 0;  cz_c = 1.2;
t_c = linspace(0, 2*pi, 8*N_PER_SEG + 1);
t_c = t_c(1:end-1);                         % open — spline will close it
pts_circle = [cx*ones(1,numel(t_c));
              cy_c + r_c*cos(t_c);
              cz_c + r_c*sin(t_c)];

% --- 2. Square (densified edges) ---
hs = 0.42;   cy_s = 0;  cz_s = 1.2;        % half-side
corners = [cx, cy_s+hs, cz_s+hs;
           cx, cy_s-hs, cz_s+hs;
           cx, cy_s-hs, cz_s-hs;
           cx, cy_s+hs, cz_s-hs]';
pts_square = [];
for i = 1:4
    j = mod(i, 4) + 1;
    pts_square = [pts_square, seg(corners(:,i), corners(:,j), N_PER_SEG)]; %#ok<AGROW>
end

% --- 3. Helix (two full turns, rising in Z) ---
r_h = 0.38;  cz_h0 = 0.70;  rise = 1.00;
t_h = linspace(0, 4*pi, 16*N_PER_SEG);
pts_helix = [cx*ones(1,numel(t_h));
             r_h*cos(t_h);
             cz_h0 + rise * t_h/(4*pi)];

% --- 4. Right-angle snake (horizontal runs connected by vertical steps) ---
%   Three horizontal passes at increasing heights, connected by risers.
run_y   = 0.60;   % half-width of each horizontal run
step_z  = 0.30;   % vertical rise per step
z_start = 0.80;
snake_nodes = [cx, -run_y, z_start;
               cx,  run_y, z_start;
               cx,  run_y, z_start + step_z;
               cx, -run_y, z_start + step_z;
               cx, -run_y, z_start + 2*step_z;
               cx,  run_y, z_start + 2*step_z]';
pts_snake = [];
for i = 1:size(snake_nodes,2)-1
    pts_snake = [pts_snake, seg(snake_nodes(:,i), snake_nodes(:,i+1), N_PER_SEG)]; %#ok<AGROW>
end
pts_snake = [pts_snake, snake_nodes(:,end)];  % include final point

%% ---- Pack into a cell array ----
shapes = {
    struct('name','Circle', 'pts',pts_circle, 'closed',true);
    struct('name','Square', 'pts',pts_square, 'closed',true);
    struct('name','Helix',  'pts',pts_helix,  'closed',false);
    struct('name','Snake',  'pts',pts_snake,  'closed',false);
};

%% ---- Main loop: solve IK then animate each shape ----
for s = 1:numel(shapes)
    shape  = shapes{s};
    wp     = shape.pts;         % 3 x N_wp desired EE positions
    n_wp   = size(wp, 2);

    fprintf('\n=== %s : solving IK for %d waypoints ===\n', shape.name, n_wp);

    %% -- IK pass --
    theta_wp   = zeros(6, n_wp);
    theta_prev = theta_init;

    for k = 1:n_wp
        T_des = [R_des, wp(:,k); 0 0 0 1];
        [th, ok, ~] = DLS_inverse_kinematics(robot, T_des, theta_prev, ...
                        IK_LAMBDA, IK_SIGMA, IK_EOMG, IK_EV, IK_MAXITER);
        if ~ok
            warning('%s wp %d/%d: IK did not converge — using previous angles.', ...
                    shape.name, k, n_wp);
            th = theta_prev;
        end
        theta_wp(:,k) = th;
        theta_prev    = th;
    end

    %% -- Build smooth spline trajectory --
    if shape.closed
        % Wrap: append first waypoint at end so spline closes the loop
        t_knots  = 1:(n_wp+1);
        theta_kn = [theta_wp, theta_wp(:,1)];
    else
        t_knots  = 1:n_wp;
        theta_kn = theta_wp;
    end

    n_frames  = n_wp * N_INTERP;
    t_fine    = linspace(t_knots(1), t_knots(end), n_frames);
    theta_traj = zeros(6, n_frames);
    for j = 1:6
        theta_traj(j,:) = spline(t_knots, theta_kn(j,:), t_fine);
    end

    %% -- Pre-compute actual EE path from trajectory --
    ee_path = zeros(3, n_frames);
    for f = 1:n_frames
        T = FK_space(robot.M, robot.Slist, theta_traj(:,f));
        ee_path(:,f) = T(1:3,4);
    end

    fprintf('   Animating %d frames...\n', n_frames);

    %% -- Animation --
    scale = norm(robot.M(1:3,4)) * 0.08;
    hfig  = figure('Name', sprintf('KR210 — %s', shape.name), ...
                   'Color', 'w', 'Position', [120 80 920 660]);

    if RECORD
        vid_file = sprintf('anim_%s.mp4', lower(shape.name));
        vid = VideoWriter(vid_file, 'MPEG-4');
        vid.FrameRate = VID_FPS;
        open(vid);
        fprintf('   Recording to %s ...\n', vid_file);
    end

    for f = 1:n_frames
        clf(hfig);
        ax = axes('Parent', hfig);
        hold(ax,'on'); axis(ax,'equal'); grid(ax,'on'); view(ax,[-35 25]);
        set(ax,'Color','w','XColor','k','YColor','k','ZColor','k', ...
               'GridColor',[0.82 0.82 0.82]);
        xlabel(ax,'X (m)'); ylabel(ax,'Y (m)'); zlabel(ax,'Z (m)');
        title(ax, sprintf('%s  —  frame %d / %d', shape.name, f, n_frames), ...
              'Color','k');

        thetalist = theta_traj(:,f);

        %% Compute joint positions via partial FK
        T_fr = cell(1, robot.n_dof+1);
        T_fr{1} = eye(4);
        for i = 1:robot.n_dof
            T_fr{i+1} = T_fr{i} * MatrixExp6(vecToSE3(robot.Slist(:,i)*thetalist(i)));
        end
        T_ee = T_fr{robot.n_dof+1} * robot.M;
        q_pts = zeros(3, robot.n_dof);
        for i = 1:robot.n_dof
            q_pts(:,i) = T_fr{i}(1:3,1:3)*robot.q_joints(:,i) + T_fr{i}(1:3,4);
        end
        p_ee = T_ee(1:3,4);

        %% Full desired path — dotted blue
        if shape.closed
            plot3(ax, [wp(1,:), wp(1,1)], [wp(2,:), wp(2,1)], [wp(3,:), wp(3,1)], ...
                  'b:', 'LineWidth', 1.8);
        else
            plot3(ax, wp(1,:), wp(2,:), wp(3,:), 'b:', 'LineWidth', 1.8);
        end

        %% Traced EE path so far — solid cyan
        if f > 1
            plot3(ax, ee_path(1,1:f), ee_path(2,1:f), ee_path(3,1:f), ...
                  '-', 'Color',[0.00 0.65 0.85], 'LineWidth', 2.2);
        end

        %% Robot arm
        link_pts = [q_pts, p_ee];
        plot3(ax, link_pts(1,:), link_pts(2,:), link_pts(3,:), ...
              'k-', 'LineWidth', 3);
        plot3(ax, q_pts(1,:), q_pts(2,:), q_pts(3,:), ...
              'o', 'MarkerSize', 7, 'Color',[0.25 0.25 0.25], ...
              'MarkerFaceColor',[0.45 0.45 0.45]);

        %% End-effector marker (red square)
        plot3(ax, p_ee(1), p_ee(2), p_ee(3), ...
              's', 'MarkerSize', 11, 'Color',[0.85 0.10 0.10], ...
              'MarkerFaceColor',[0.85 0.10 0.10]);

        %% Joint coordinate frames (small arrows)
        fc = {[0.90 0.20 0.20],[0.15 0.70 0.20],[0.15 0.30 0.90]};
        for i = 1:robot.n_dof
            Rf = T_fr{i}(1:3,1:3);
            p  = q_pts(:,i);
            for k = 1:3
                quiver3(ax, p(1),p(2),p(3), ...
                        Rf(1,k)*scale, Rf(2,k)*scale, Rf(3,k)*scale, ...
                        'Color',fc{k},'LineWidth',1.0, ...
                        'MaxHeadSize',0.7,'AutoScale','off');
            end
        end

        drawnow;

        if RECORD
            writeVideo(vid, getframe(hfig));
        end
    end

    if RECORD
        close(vid);
        fprintf('   Saved %s\n', vid_file);
    end
    fprintf('   %s complete.\n', shape.name);
end

fprintf('\nAll animations complete.\n');

%% ---- Helper: densify a segment from p0 to p1 (exclusive of endpoint) ----
function pts = seg(p0, p1, n)
    t = linspace(0, 1, n+1);
    t = t(1:end-1);     % exclude endpoint (next segment starts there)
    pts = p0 + t .* (p1 - p0);
end
