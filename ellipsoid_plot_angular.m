function ellipsoid_plot_angular(robot, thetalist)
% ellipsoid_plot_angular: Plots the angular-velocity manipulability ellipsoid
%                         and its principal axes at the current configuration.
%
% The angular manipulability ellipsoid characterises the set of unit-norm
% angular velocities reachable by unit joint-velocity inputs:
%
%   E_w = { omega : omega' * (Jw*Jw')^{-1} * omega <= 1 }
%
% where Jw = rows 1-3 of the space Jacobian (angular part).
% The ellipsoid semi-axes are the singular values of Jw; their directions
% are the left singular vectors (columns of U in Jw = U*S*V').
%
% Inputs:
%   robot     - robot struct from KR120_params()
%   thetalist - n x 1 joint angle vector (radians)

    Js = J_space(robot.Slist, thetalist);
    Jw = Js(1:3, :);

    % SVD: Jw = U * S * V',  semi-axes = diag(S), directions = columns of U
    [U, S, ~] = svd(Jw);
    sigma = diag(S);

    % End-effector position (centre of the ellipsoid in the plot)
    T   = FK_space(robot.M, robot.Slist, thetalist);
    p_c = T(1:3, 4);

    figure('Color','w'); hold on; axis equal; grid on; view(3);
    set(gca, 'Color','w', 'XColor','k', 'YColor','k', 'ZColor','k', 'GridColor','k');
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
    title(sprintf('Angular Velocity Manipulability Ellipsoid\n\\theta = [%s] rad', ...
                  num2str(thetalist', '%.2f ')));

    %% Draw robot arm for spatial context (requires robot.q_joints)
    if isfield(robot, 'q_joints')
        n = size(robot.Slist, 2);
        T_frames = cell(1, n+1);
        T_frames{1} = eye(4);
        for i = 1:n
            T_frames{i+1} = T_frames{i} * MatrixExp6(vecToSE3(robot.Slist(:,i) * thetalist(i)));
        end
        q_curr = zeros(3, n);
        for i = 1:n
            Tf = T_frames{i};
            q_curr(:,i) = Tf(1:3,1:3) * robot.q_joints(:,i) + Tf(1:3,4);
        end
        link_pts = [q_curr, T(1:3,4)];
        plot3(link_pts(1,:), link_pts(2,:), link_pts(3,:), ...
              'k-o', 'LineWidth', 1.5, 'MarkerSize', 4, 'MarkerFaceColor', 'k', ...
              'DisplayName', 'Robot arm');
    end

    %% Draw the ellipsoid surface
    % Parametric unit sphere -> transform by U*diag(sigma)
    [xs, ys, zs] = sphere(40);
    pts = [xs(:)'; ys(:)'; zs(:)'];          % 3 x N unit-sphere points
    epts = U * diag(sigma) * pts;             % stretch & rotate
    xe = reshape(epts(1,:), size(xs)) + p_c(1);
    ye = reshape(epts(2,:), size(ys)) + p_c(2);
    ze = reshape(epts(3,:), size(zs)) + p_c(3);
    surf(xe, ye, ze, 'FaceAlpha', 0.25, 'EdgeColor', 'none', 'FaceColor', [0.2 0.6 1.0]);

    %% Draw principal axes
    colors = {'r','g','b'};
    labels = {'\sigma_1','\sigma_2','\sigma_3'};
    for k = 1:3
        ax_vec = U(:,k) * sigma(k);
        quiver3(p_c(1), p_c(2), p_c(3), ...
                ax_vec(1), ax_vec(2), ax_vec(3), ...
                'Color', colors{k}, 'LineWidth', 2, ...
                'MaxHeadSize', 0.3, 'AutoScale', 'off');
        quiver3(p_c(1), p_c(2), p_c(3), ...
               -ax_vec(1),-ax_vec(2),-ax_vec(3), ...
                'Color', colors{k}, 'LineWidth', 2, ...
                'MaxHeadSize', 0.3, 'AutoScale', 'off');
        text(p_c(1)+ax_vec(1)*1.1, p_c(2)+ax_vec(2)*1.1, p_c(3)+ax_vec(3)*1.1, ...
             sprintf('%s = %.4f', labels{k}, sigma(k)), 'FontSize', 9, 'Color', 'k');
    end

    % Mark centre
    plot3(p_c(1), p_c(2), p_c(3), 'ko', 'MarkerSize', 6, 'MarkerFaceColor', 'k');

    fprintf('Angular ellipsoid semi-axes (sigma): %.4f  %.4f  %.4f\n', sigma(1), sigma(2), sigma(3));
    fprintf('Volume = %.6f\n', (4/3)*pi*prod(sigma(1:3)));
    hold off;
end
