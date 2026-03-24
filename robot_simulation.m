%% robot_simulation.m
% Graphical simulation of the KUKA KR120 R2500 Pro executing a multi-waypoint
% trajectory, using the Product-of-Exponentials FK and DLS inverse kinematics.
%
% The simulation:
%   1. Defines a Cartesian EE trajectory (a tilted ellipse in 3-D space).
%   2. Solves joint angles at each waypoint using DLS_inverse_kinematics, which
%      remains numerically stable near singularities.
%   3. Interpolates smoothly between waypoints (cubic joint-space splines).
%   4. Animates the robot frame-by-frame, displaying the EE path and live
%      manipulability measure w = sqrt(det(Js * Js')).
%
% Run from the SensorRoboticsFunctions directory:
%   >> robot_simulation

clc;
addpath('.');
robot = KR120_params();

%% ---- 1. Define Cartesian waypoints ----
% A tilted ellipse centred roughly in the KR120's dexterous workspace.
% Centre: [1.7, 0, 1.0] m   rx/ry = 0.5 m (XY plane)   rz = 0.35 m
% (KR120 max reach ~2.7 m; keep well inside to ensure IK convergence.)
n_wp   = 12;                        % number of waypoints around the ellipse
phi    = linspace(0, 2*pi, n_wp+1);
phi    = phi(1:end-1);              % drop duplicate endpoint

cx = 1.7;  cy = 0.0;  cz = 1.0;
rx = 0.5;  ry = 0.5;  rz = 0.35;

% Desired EE orientation: pointing along -X at all waypoints (tip toward base)
R_des = [-1  0  0;
          0 -1  0;
          0  0  1];

fprintf('Solving IK for %d waypoints...\n', n_wp);

%% ---- 2. Solve IK at each waypoint (DLS for robustness) ----
theta_wp   = zeros(6, n_wp);
theta_prev = [0; pi/6; -pi/3; 0; pi/4; 0];   % warm-start near a good pose

for k = 1:n_wp
    p_des = [cx + rx*cos(phi(k));
             cy + ry*sin(phi(k));
             cz + rz*sin(phi(k))];
    T_des = [R_des, p_des; 0 0 0 1];

    [th, ok, ~] = DLS_inverse_kinematics(robot, T_des, theta_prev, ...
                                          0.15, 0.05, 1e-3, 1e-3, 300);
    if ~ok
        warning('IK did not converge at waypoint %d; using previous angles.', k);
        th = theta_prev;
    end
    theta_wp(:,k) = th;
    theta_prev    = th;             % warm-start next waypoint from this solution
end
fprintf('IK complete.\n\n');

%% ---- 3. Build smooth interpolated trajectory (cubic spline) ----
n_interp = 30;                      % frames between consecutive waypoints
t_wp   = 0:n_wp;                    % waypoint "times" (integer knots)
t_wp   = [t_wp(end), t_wp];        % close the loop: append initial point at end
theta_loop = [theta_wp, theta_wp(:,1)];  % close trajectory

t_fine = linspace(0, n_wp, n_wp * n_interp);
theta_traj = zeros(6, numel(t_fine));
for j = 1:6
    theta_traj(j,:) = spline(t_wp, theta_loop(j,:), t_fine);
end
n_frames = size(theta_traj, 2);

%% ---- 4. Set up persistent animation figure ----
hfig = figure('Name', '(m) KR120 Trajectory Simulation', ...
              'Color', [0.12 0.12 0.12], 'Position', [100 80 1000 700]);

% Pre-compute EE path for background trace
ee_path = zeros(3, n_frames);
for f = 1:n_frames
    T = FK_space(robot.M, robot.Slist, theta_traj(:,f));
    ee_path(:,f) = T(1:3,4);
end

scale = norm(robot.M(1:3,4)) * 0.10;   % axis arrow scale

% Colour map for manipulability (blue = low, red = high)
cmap = parula(256);

fprintf('Animating %d frames (%d waypoints x %d interp)...\n', n_frames, n_wp, n_interp);

for f = 1:n_frames
    clf(hfig);
    ax = axes('Parent', hfig, 'Color', [0.12 0.12 0.12]);
    hold(ax, 'on'); axis(ax, 'equal'); grid(ax, 'on'); view(ax, 3);

    ax.XColor = 'w'; ax.YColor = 'w'; ax.ZColor = 'w'; ax.GridColor = 'w';
    ax.GridAlpha = 0.15;
    xlabel(ax,'X (m)','Color','w'); ylabel(ax,'Y (m)','Color','w');
    zlabel(ax,'Z (m)','Color','w');

    thetalist = theta_traj(:,f);

    %% Compute partial transforms for link drawing
    T_frames = cell(1, robot.n_dof + 1);
    T_frames{1} = eye(4);
    for i = 1:robot.n_dof
        T_frames{i+1} = T_frames{i} * ...
            MatrixExp6(vecToSE3(robot.Slist(:,i) * thetalist(i)));
    end
    T_ee = T_frames{robot.n_dof+1} * robot.M;

    % Recover current joint positions from stored physical home positions.
    % (cross(w,v) only gives the correct position when q·omega=0; for KR120
    %  joints 4 & 6 that is not true, so we use robot.q_joints instead.)
    q_pts = zeros(3, robot.n_dof);
    for i = 1:robot.n_dof
        q_pts(:,i) = T_frames{i}(1:3,1:3)*robot.q_joints(:,i) + T_frames{i}(1:3,4);
    end
    p_ee = T_ee(1:3,4);

    %% Draw EE trace (past = dim, future = bright)
    plot3(ax, ee_path(1,1:f), ee_path(2,1:f), ee_path(3,1:f), ...
          '-', 'Color', [0.2 0.8 1.0 0.9], 'LineWidth', 1.5);
    plot3(ax, ee_path(1,f:end), ee_path(2,f:end), ee_path(3,f:end), ...
          '--', 'Color', [0.2 0.8 1.0 0.3], 'LineWidth', 0.8);

    %% Draw robot links
    link_pts = [q_pts, p_ee];
    plot3(ax, link_pts(1,:), link_pts(2,:), link_pts(3,:), ...
          '-', 'Color', [0.85 0.85 0.85], 'LineWidth', 3);
    plot3(ax, q_pts(1,:), q_pts(2,:), q_pts(3,:), ...
          'o', 'Color', [1 0.6 0.1], 'MarkerSize', 8, 'MarkerFaceColor', [1 0.6 0.1]);
    plot3(ax, p_ee(1), p_ee(2), p_ee(3), ...
          's', 'Color', [0.2 0.8 1], 'MarkerSize', 10, 'MarkerFaceColor', [0.2 0.8 1]);

    %% Draw joint coordinate frames (X=r, Y=g, Z=b)
    frame_colors = {[1 0.3 0.3],[0.3 1 0.3],[0.3 0.5 1]};
    for i = 1:robot.n_dof
        R_f = T_frames{i}(1:3,1:3);
        p   = q_pts(:,i);
        for k = 1:3
            quiver3(ax, p(1),p(2),p(3), ...
                    R_f(1,k)*scale, R_f(2,k)*scale, R_f(3,k)*scale, ...
                    'Color', frame_colors{k}, 'LineWidth', 1.2, ...
                    'MaxHeadSize', 0.6, 'AutoScale', 'off');
        end
    end

    %% Manipulability indicator
    Js  = J_space(robot.Slist, thetalist);
    w_m = sqrt(abs(det(Js * Js')));

    % Map w to colour (clamp to [0, 2])
    w_norm  = min(w_m / 2, 1);
    ci      = max(1, round(w_norm * 255) + 1);
    bar_col = cmap(ci,:);

    title(ax, sprintf('(m) KR120 Simulation   frame %d/%d\n  w = %.4f', ...
                      f, n_frames, w_m), 'Color', 'w', 'FontSize', 11);

    % Small coloured dot in corner to indicate manipulability level
    annotation(hfig, 'ellipse', [0.88 0.88 0.05 0.05], ...
               'FaceColor', bar_col, 'Color', 'none');

    drawnow;
end

fprintf('Simulation complete.\n');
