function T = FK_space(M, Slist, thetalist, visualize, q_joints_home)
% FK_space: Computes forward kinematics using space-form PoE and optionally
%           displays the robot's frames and screw axes graphically.
%
% T = exp([S1]theta1) * ... * exp([Sn]thetan) * M
%
% Inputs:
%   M              - 4x4 home configuration of end-effector
%   Slist          - 6xn matrix of space-form screw axes (columns)
%   thetalist      - nx1 vector of joint angles (radians)
%   visualize      - (optional) true to plot frames and screw axes (default: false)
%   q_joints_home  - (optional) 3×n physical joint positions at home config.
%                    When omitted, falls back to cross(ω,v) which is only
%                    correct when q·ω = 0 for every joint (not true for
%                    joints whose axis has a large component along its own
%                    rotation direction, e.g. KR120 joints 4 & 6).
%
% Output:
%   T         - 4x4 end-effector transformation matrix

    if nargin < 4, visualize = false; end
    if nargin < 5, q_joints_home = []; end

    n = length(thetalist);

    % Accumulate partial FK transforms; store each for visualization
    T_frames = cell(1, n + 1);   % T_frames{i} = exp(S1*t1)*...*exp(S_{i-1}*t_{i-1})
    T_frames{1} = eye(4);
    for i = 1:n
        T_frames{i+1} = T_frames{i} * MatrixExp6(vecToSE3(Slist(:,i) * thetalist(i)));
    end

    T = T_frames{n+1} * M;

    if ~visualize, return; end

    %% ---- Visualization ----
    figure('Color','w'); hold on; axis equal; grid on; view(3);
    set(gca, 'Color','w', 'XColor','k', 'YColor','k', 'ZColor','k', 'GridColor','k');
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
    title('FK Space Form: Joint Frames and Screw Axes');

    % Arrow scale: ~12% of total arm reach
    scale = norm(M(1:3,4)) * 0.12;
    if scale < 0.05, scale = 0.05; end

    % Recover home joint positions.
    % Preferred: use the supplied physical joint positions (robot.q_joints).
    % Fallback: cross(ω,v) gives the point on the axis *closest to origin*,
    %   which equals the physical joint position only when q·ω = 0.
    %   For KR120 joints 4 & 6 (ω=[-1;0;0], q_x≈2.5) this fallback returns
    %   [0;0;d1] instead of [2.5;0;d1], making the link chain fold backwards.
    q_home = zeros(3, n);
    for i = 1:n
        if ~isempty(q_joints_home)
            q_home(:,i) = q_joints_home(:,i);
        else
            w = Slist(1:3, i);
            v = Slist(4:6, i);
            if norm(w) > 1e-8      % revolute joint
                q_home(:,i) = cross(w, v);
            end
            % prismatic: leave at zeros(3,1)
        end
    end

    % Current joint positions: transform home position by partial FK
    q_curr = zeros(3, n);
    for i = 1:n
        Tf = T_frames{i};                         % transform before joint i
        q_curr(:,i) = Tf(1:3,1:3) * q_home(:,i) + Tf(1:3,4);
    end
    p_ee = T(1:3,4);

    % Draw robot links (joint positions + end-effector)
    link_pts = [q_curr, p_ee];
    plot3(link_pts(1,:), link_pts(2,:), link_pts(3,:), ...
          'k-o', 'LineWidth', 2, 'MarkerSize', 6, 'MarkerFaceColor', 'k');

    % Draw coordinate frame axes at each joint (x=red, y=green, z=blue)
    colors = {'r','g','b'};
    for i = 1:n
        R_frame = T_frames{i}(1:3,1:3);
        p = q_curr(:,i);
        for k = 1:3
            quiver3(p(1), p(2), p(3), ...
                    R_frame(1,k)*scale, R_frame(2,k)*scale, R_frame(3,k)*scale, ...
                    'Color', colors{k}, 'LineWidth', 1.5, 'MaxHeadSize', 0.5, ...
                    'AutoScale', 'off');
        end
        text(p(1)+scale*0.1, p(2), p(3)+scale*0.1, sprintf('J%d',i), 'FontSize', 9, 'Color', 'k');
    end

    % Draw end-effector frame
    R_ee = T(1:3,1:3);
    for k = 1:3
        quiver3(p_ee(1), p_ee(2), p_ee(3), ...
                R_ee(1,k)*scale, R_ee(2,k)*scale, R_ee(3,k)*scale, ...
                'Color', colors{k}, 'LineWidth', 2, 'MaxHeadSize', 0.5, ...
                'AutoScale', 'off');
    end
    text(p_ee(1)+scale*0.1, p_ee(2), p_ee(3)+scale*0.1, 'EE', 'FontSize', 9, 'Color', 'k');

    % Draw current screw axes (magenta arrows through joint positions)
    for i = 1:n
        w_curr = T_frames{i}(1:3,1:3) * Slist(1:3,i);  % rotated axis direction
        p = q_curr(:,i);
        p0 = p - w_curr * scale;   % arrow starts one unit behind joint
        quiver3(p0(1), p0(2), p0(3), ...
                w_curr(1)*2*scale, w_curr(2)*2*scale, w_curr(3)*2*scale, ...
                'Color', 'm', 'LineWidth', 1, 'MaxHeadSize', 0.3, 'AutoScale', 'off');
    end

    % Legend proxy entries — use plot3 lines so colours render with painters
    h_link  = plot3(nan, nan, nan, 'k-o', 'LineWidth', 2);
    h_x     = plot3(nan, nan, nan, '-',  'Color', 'r',               'LineWidth', 2);
    h_y     = plot3(nan, nan, nan, '-',  'Color', 'g',               'LineWidth', 2);
    h_z     = plot3(nan, nan, nan, '-',  'Color', 'b',               'LineWidth', 2);
    h_screw = plot3(nan, nan, nan, '-',  'Color', [1 0 1],           'LineWidth', 1.5);
    lg = legend([h_link, h_x, h_y, h_z, h_screw], ...
           'Links', 'X-axis', 'Y-axis', 'Z-axis', 'Screw axis', ...
           'Location', 'best');
    set(lg, 'Color','w', 'TextColor','k', 'EdgeColor','k');

    hold off;
end
