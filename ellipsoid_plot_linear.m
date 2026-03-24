function ellipsoid_plot_linear(robot, thetalist)
% ellipsoid_plot_linear: Plots the linear-velocity manipulability ellipsoid
%                        and its principal axes at the current configuration.
%
% The linear manipulability ellipsoid characterises the set of unit-norm
% linear velocities reachable by unit joint-velocity inputs:
%
%   E_v = { v : v' * (Jv*Jv')^{-1} * v <= 1 }
%
% where Jv = rows 4-6 of the space Jacobian (linear part).
% Semi-axes = singular values of Jv; directions = left singular vectors.
%
% Inputs:
%   robot     - robot struct from KR120_params()
%   thetalist - n x 1 joint angle vector (radians)

    Js = J_space(robot.Slist, thetalist);
    Jv = Js(4:6, :);

    % SVD: Jv = U * S * V',  semi-axes = diag(S), directions = columns of U
    [U, S, ~] = svd(Jv);
    sigma = diag(S);

    % End-effector position (centre of the ellipsoid in the plot)
    T   = FK_space(robot.M, robot.Slist, thetalist);
    p_c = T(1:3, 4);

    figure('Color','w'); hold on; axis equal; grid on; view(3);
    set(gca, 'Color','w', 'XColor','k', 'YColor','k', 'ZColor','k', 'GridColor','k');
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
    title(sprintf('Linear Velocity Manipulability Ellipsoid\n\\theta = [%s] rad', ...
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

    %% Draw ellipsoid surface
    [xs, ys, zs] = sphere(40);
    pts  = [xs(:)'; ys(:)'; zs(:)'];
    epts = U * diag(sigma) * pts;
    xe = reshape(epts(1,:), size(xs)) + p_c(1);
    ye = reshape(epts(2,:), size(ys)) + p_c(2);
    ze = reshape(epts(3,:), size(zs)) + p_c(3);
    surf(xe, ye, ze, 'FaceAlpha', 0.25, 'EdgeColor', 'none', 'FaceColor', [1.0 0.5 0.1]);

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

    plot3(p_c(1), p_c(2), p_c(3), 'ko', 'MarkerSize', 6, 'MarkerFaceColor', 'k');

    fprintf('Linear  ellipsoid semi-axes (sigma): %.4f  %.4f  %.4f\n', sigma(1), sigma(2), sigma(3));
    fprintf('Volume = %.6f\n', (4/3)*pi*prod(sigma(1:3)));
    hold off;
end
