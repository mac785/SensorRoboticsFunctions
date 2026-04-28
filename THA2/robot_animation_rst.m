%% robot_animation_rst.m
% Animates the KUKA KR120 R2500 Pro tracing four geometric shapes using the
% RST renderer (importrobot + show) for 3-D visualisation.
%
% Shapes: Circle, Square, Helix, Snake (same trajectories as robot_animation.m)
%
% Run from the THA2/ directory:
%   >> robot_animation_rst

clc; close all;
addpath('.');
robot = KR120_params();
rbt   = load_kr120_rst();

%% ---- Common settings ----
R_des = [-1  0  0;
          0 -1  0;
          0  0  1];

theta_init = [0; pi/6; -pi/3; 0; pi/4; 0];

IK_LAMBDA  = 0.15;
IK_SIGMA   = 0.05;
IK_EOMG    = 1e-3;
IK_EV      = 1e-3;
IK_MAXITER = 300;

N_PER_SEG  = 12;
N_INTERP   = 30;

FAST_TEST  = true;
if FAST_TEST
    N_PER_SEG = 4;
    N_INTERP  = 8;
end

RECORD  = false;
VID_FPS = 30;

%% ---- Shape definitions ----
cx = 1.9;

r_c = 0.45;  cy_c = 0;  cz_c = 1.2;
t_c = linspace(0, 2*pi, 8*N_PER_SEG + 1);
t_c = t_c(1:end-1);
pts_circle = [cx*ones(1,numel(t_c));
              cy_c + r_c*cos(t_c);
              cz_c + r_c*sin(t_c)];

hs = 0.42;   cy_s = 0;  cz_s = 1.2;
corners = [cx, cy_s+hs, cz_s+hs;
           cx, cy_s-hs, cz_s+hs;
           cx, cy_s-hs, cz_s-hs;
           cx, cy_s+hs, cz_s-hs]';
pts_square = [];
for i = 1:4
    j = mod(i, 4) + 1;
    pts_square = [pts_square, seg(corners(:,i), corners(:,j), N_PER_SEG)]; %#ok<AGROW>
end

r_h = 0.38;  cz_h0 = 0.70;  rise = 1.00;
t_h = linspace(0, 4*pi, 16*N_PER_SEG);
pts_helix = [cx*ones(1,numel(t_h));
             r_h*cos(t_h);
             cz_h0 + rise * t_h/(4*pi)];

run_y   = 0.60;  step_z  = 0.30;  z_start = 0.80;
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
pts_snake = [pts_snake, snake_nodes(:,end)];

shapes = {
    struct('name','Circle', 'pts',pts_circle, 'closed',true);
    struct('name','Square', 'pts',pts_square, 'closed',true);
    struct('name','Helix',  'pts',pts_helix,  'closed',false);
    struct('name','Snake',  'pts',pts_snake,  'closed',false);
};

%% ---- Main loop ----
for s = 1:numel(shapes)
    shape = shapes{s};
    wp    = shape.pts;
    n_wp  = size(wp, 2);

    fprintf('\n=== %s : solving IK for %d waypoints ===\n', shape.name, n_wp);

    %% IK pass
    theta_wp   = zeros(6, n_wp);
    theta_prev = theta_init;
    for k = 1:n_wp
        T_des = [R_des, wp(:,k); 0 0 0 1];
        [th, ok, ~] = DLS_inverse_kinematics(robot, T_des, theta_prev, ...
                        IK_LAMBDA, IK_SIGMA, IK_EOMG, IK_EV, IK_MAXITER);
        if ~ok
            warning('%s wp %d/%d: IK did not converge.', shape.name, k, n_wp);
            th = theta_prev;
        end
        theta_wp(:,k) = th;
        theta_prev    = th;
    end

    %% Spline trajectory
    if shape.closed
        t_knots  = 1:(n_wp+1);
        theta_kn = [theta_wp, theta_wp(:,1)];
    else
        t_knots  = 1:n_wp;
        theta_kn = theta_wp;
    end
    n_frames   = n_wp * N_INTERP;
    t_fine     = linspace(t_knots(1), t_knots(end), n_frames);
    theta_traj = zeros(6, n_frames);
    for j = 1:6
        theta_traj(j,:) = spline(t_knots, theta_kn(j,:), t_fine);
    end

    %% Pre-compute EE path
    ee_path = zeros(3, n_frames);
    for f = 1:n_frames
        T = FK_space(robot.M, robot.Slist, theta_traj(:,f));
        ee_path(:,f) = T(1:3,4);
    end

    fprintf('   Animating %d frames...\n', n_frames);

    %% Persistent figure + axes (RST show() reuses them each frame)
    hfig = figure('Name', sprintf('KR120 RST — %s', shape.name), ...
                  'Color','w','Position',[120 80 920 660]);
    ax   = axes('Parent',hfig);
    hold(ax,'on'); grid(ax,'on'); view(ax,[-35 25]);
    set(ax,'Color','w','XColor','k','YColor','k','ZColor','k', ...
           'GridColor',[0.82 0.82 0.82]);
    xlabel(ax,'X (m)'); ylabel(ax,'Y (m)'); zlabel(ax,'Z (m)');

    if RECORD
        vid_file = sprintf('anim_%s.avi', lower(shape.name));
        vid = VideoWriter(vid_file, 'Motion JPEG AVI');
        vid.FrameRate = VID_FPS;
        open(vid);
        fprintf('   Recording to %s ...\n', vid_file);
    end

    for f = 1:n_frames
        thetalist = theta_traj(:,f);

        %% Clear axes children, preserve labels
        kids = findall(ax);
        keep = [ax; ax.XLabel; ax.YLabel; ax.ZLabel];
        delete(kids(~ismember(kids, keep)));
        hold(ax,'on');
        camlight(ax,'headlight');

        %% RST robot render
        show(rbt, thetalist(:)', 'Parent',ax, ...
             'PreservePlot',true, 'Visuals','on', 'Frames','off');

        %% Close any stray figures opened by show()
        all_figs = findall(0,'Type','figure');
        for fj = 1:numel(all_figs)
            if all_figs(fj) ~= hfig, close(all_figs(fj)); end
        end
        set(0,'CurrentFigure',hfig);

        %% EE position (from our FK, consistent with IK math)
        T_ee = FK_space(robot.M, robot.Slist, thetalist);
        p_ee = T_ee(1:3,4);

        %% Desired path
        if shape.closed
            plot3(ax, [wp(1,:),wp(1,1)],[wp(2,:),wp(2,1)],[wp(3,:),wp(3,1)], ...
                  'b:','LineWidth',1.8,'HandleVisibility','off');
        else
            plot3(ax, wp(1,:),wp(2,:),wp(3,:),'b:','LineWidth',1.8,'HandleVisibility','off');
        end

        %% Traced EE path so far
        if f > 1
            plot3(ax, ee_path(1,1:f),ee_path(2,1:f),ee_path(3,1:f), ...
                  '-','Color',[0.00 0.65 0.85],'LineWidth',2.2,'HandleVisibility','off');
        end

        %% EE marker
        plot3(ax, p_ee(1),p_ee(2),p_ee(3), 's','MarkerSize',11, ...
              'Color',[0.85 0.10 0.10],'MarkerFaceColor',[0.85 0.10 0.10], ...
              'HandleVisibility','off');

        title(ax, sprintf('%s  —  frame %d / %d', shape.name, f, n_frames),'Color','k');
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
    t = t(1:end-1);
    pts = p0 + t .* (p1 - p0);
end
