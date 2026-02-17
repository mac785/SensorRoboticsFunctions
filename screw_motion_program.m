function screw_motion_program()
% SCREW_MOTION_PROGRAM
%
% Prompts user for:
%   - initial configuration T (4x4)
%   - screw axis specified by {q, shat, h} in fixed frame {s}
%   - total distance traveled along screw axis theta
%
% Computes:
%   T1 = exp([S]theta)*T
% and intermediate configurations at theta/4, theta/2, 3theta/4
% Plots body frame {b} axes at initial/intermediate/final configurations.
%
% Also computes screw axis S1 and distance theta1 such that:
%   exp([S1]theta1) = inv(T1)
% i.e., following S1 by theta1 takes the body from T1 to the origin pose (I).
% Plots screw axis S1.

    clc; close all;

    % User input stuff

    fprintf("=== Screw Motion Program ===\n\n");

    useTest = input("Use the provided test case? (y/n) [y]: ", "s");
    if isempty(useTest), useTest = "y"; end
    useTest = lower(string(strtrim(useTest)));

    if useTest == "y"
        % Test case from prompt
        q     = [0; 2; 0];
        shat  = [0; 0; 1];
        h     = 2;
        theta = pi;
        T = [1 0 0 2;
             0 1 0 0;
             0 0 1 0;
             0 0 0 1];

        fprintf("\nUsing test case:\n");
        fprintf("q = [0;2;0], shat = [0;0;1], h = 2, theta = pi\n");
        fprintf("T = \n"); disp(T);

    else
        fprintf("\nEnter q (point on screw axis) as [qx qy qz] or [qx; qy; qz]\n");
        q = input("q = ");
        q = q(:);

        fprintf("\nEnter shat (direction of screw axis) as [sx sy sz] or [sx; sy; sz]\n");
        shat = input("shat = ");
        shat = shat(:);

        h = input("\nEnter pitch h (scalar): ");
        theta = input("Enter total distance theta (radians): ");

        fprintf("\nEnter 4x4 homogeneous transform T.\n");
        fprintf("Tip: you can paste it like: [1 0 0 2; 0 1 0 0; 0 0 1 0; 0 0 0 1]\n");
        T = input("T = ");

        if ~isequal(size(T), [4 4])
            error("T must be 4x4.");
        end
    end

    % Actual calculations

    S = screwAxisToTwist(q, shat, h);  % 6x1 [w; v]

    w = S(1:3);
    v = S(4:6);
    
    fprintf("q    = [%g %g %g]^T\n", q);
    fprintf("shat = [%g %g %g]^T\n", shat);
    fprintf("w    = [%g %g %g]^T\n", w);
    fprintf("v    = [%g %g %g]^T\n", v);



    thetas = [0, theta/4, theta/2, 3*theta/4, theta];
    labels = ["0", "\theta/4", "\theta/2", "3\theta/4", "\theta"];

    Ts = zeros(4,4,numel(thetas));
    for k = 1:numel(thetas)
        Ts(:,:,k) = MatrixExp6(VecTose3(S) * thetas(k)) * T;
    end
    T1 = Ts(:,:,end);

    fprintf("\nFinal configuration T1 = exp([S]*theta)*T:\n");
    disp(T1);

    figure; hold on; grid on; axis equal;
    xlabel('X'); ylabel('Y'); zlabel('Z');
    title('Body frame {b} at initial, intermediate, and final configurations');

    scale = 0.5;
    for k = 1:numel(thetas)
        plotFrame(Ts(:,:,k), scale, char(labels(k)));
    end

    G = inv(T1);
    se3mat = MatrixLog6(G);
    Vtheta = se3ToVec(se3mat);           % = S1*theta1
    [S1, theta1] = normalizeTwist(Vtheta);

    fprintf("\nMove-to-origin screw:\n");
    fprintf("theta1 = %.12g\n", theta1);
    fprintf("S1 = [w; v] = \n");
    disp(S1);

    axisLine = plotAxisLine(q, shat, 'k--');

    legend(axisLine, {'Screw axis'})

    legend('Location','bestoutside');
    view(3);
end


function S = screwAxisToTwist(q, shat, h)
% Given point q on axis, unit direction shat, pitch h:
%   w = shat
%   v = -w x q + h w
    q = q(:);
    shat = shat(:);

    if numel(q) ~= 3 || numel(shat) ~= 3
        error('screwAxisToTwist:InvalidInput', 'q and shat must be 3-vectors.');
    end
    if ~isscalar(h) || ~isfinite(h)
        error('screwAxisToTwist:InvalidPitch', 'h must be a finite scalar.');
    end

    n = norm(shat);
    if n < eps
        error('screwAxisToTwist:BadAxis', 'shat must be nonzero.');
    end

    w = shat / n;
    v = -cross(w, q) + h*w;
    S = [w; v];
end


function plotFrame(T, scale, labelStr)
% Plot a coordinate frame located at T (in space coords).
    R = T(1:3,1:3);
    p = T(1:3,4);

    ex = R(:,1); ey = R(:,2); ez = R(:,3);

    quiver3(p(1),p(2),p(3), scale*ex(1),scale*ex(2),scale*ex(3), 'r', 'LineWidth',1.5, 'HandleVisibility','off');
    quiver3(p(1),p(2),p(3), scale*ey(1),scale*ey(2),scale*ey(3), 'g', 'LineWidth',1.5, 'HandleVisibility','off');
    quiver3(p(1),p(2),p(3), scale*ez(1),scale*ez(2),scale*ez(3), 'b', 'LineWidth',1.5, 'HandleVisibility','off');

    text(p(1),p(2),p(3), ['  ' labelStr], 'FontSize',10);
end


function plotScrewAxis(S, styleStr) %#ok<DEFNU>
% Plot the screw axis line corresponding to twist S=[w;v].
    w = S(1:3);
    v = S(4:6);

    if norm(w) < 1e-9
        % Pure translation axis direction
        w = v / norm(v);
        q0 = [0;0;0];
    else
        w = w / norm(w);
        % A point on the axis (perpendicular to w)
        q0 = cross(w, v);
    end

    t = linspace(-5, 5, 2);
    P = q0 + w .* t;  % 3x2 (implicit expansion)

    plot3(P(1,:), P(2,:), P(3,:), styleStr, 'LineWidth', 2);
end

function h = plotAxisLine(q, dir, styleStr)
% Plot the axis line from q and dir
    q = q(:);
    dir = dir(:);
    dir = dir / norm(dir);
    t = linspace(-5, 5, 2);
    P = q + dir .* t;
    h = plot3(P(1,:), P(2,:), P(3,:), styleStr, 'LineWidth', 2);
end


function se3mat = VecTose3(V)
% V = [w; v] -> se(3)
    w = V(1:3); v = V(4:6);
    se3mat = [VecToso3(w), v;
              0 0 0 0];
end


function so3mat = VecToso3(w)
% w -> skew
    so3mat = [  0   -w(3)  w(2);
              w(3)   0   -w(1);
             -w(2)  w(1)   0 ];
end


function V = se3ToVec(se3mat)
% se(3) -> [w; v]
    V = [so3ToVec(se3mat(1:3,1:3));
         se3mat(1:3,4)];
end


function w = so3ToVec(so3mat)
% skew -> w
    w = [so3mat(3,2); so3mat(1,3); so3mat(2,1)];
end


function T = MatrixExp6(se3mat)
% Closed form exp for se(3)
    omgmat = se3mat(1:3,1:3);
    v = se3mat(1:3,4);

    if norm(omgmat,'fro') < 1e-12
        T = [eye(3), v;
             0 0 0 1];
        return;
    end

    [w, theta] = AxisAng3(omgmat);
    w_hat = VecToso3(w);

    R = MatrixExp3(omgmat);

    G = eye(3)*theta + (1 - cos(theta))*w_hat + (theta - sin(theta))*(w_hat*w_hat);
    p = G*(v/theta);

    T = [R, p;
         0 0 0 1];
end


function R = MatrixExp3(so3mat)
% Closed form exp for so(3)
    if norm(so3mat,'fro') < 1e-12
        R = eye(3);
        return;
    end

    [w, theta] = AxisAng3(so3mat);
    w_hat = VecToso3(w);

    R = eye(3) + sin(theta)*w_hat + (1 - cos(theta))*(w_hat*w_hat);
end


function [w, theta] = AxisAng3(so3mat)
% so3mat = [w]*theta
    wtheta = so3ToVec(so3mat);
    theta = norm(wtheta);
    w = wtheta / theta;
end


function se3mat = MatrixLog6(T)
% log of SE(3)
    R = T(1:3,1:3);
    p = T(1:3,4);

    if norm(R - eye(3),'fro') < 1e-12
        se3mat = [zeros(3), p;
                  0 0 0 0];
        return;
    end

    so3mat = MatrixLog3(R);
    [w, theta] = AxisAng3(so3mat);
    w_hat = VecToso3(w);

    cot_half = cos(theta/2)/sin(theta/2);
    Ginv = eye(3)/theta - 0.5*w_hat + (1/theta - 0.5*cot_half)*(w_hat*w_hat);

    v = Ginv*p;

    se3mat = [so3mat, v;
              0 0 0 0];
end


function so3mat = MatrixLog3(R)
% log of SO(3)
    acosArg = (trace(R) - 1)/2;
    acosArg = max(-1, min(1, acosArg));
    theta = acos(acosArg);

    if abs(theta) < 1e-12
        so3mat = zeros(3);
        return;
    end

    so3mat = theta/(2*sin(theta)) * (R - R');
end


function [Sunit, theta] = normalizeTwist(Vtheta)
% Vtheta = S*theta -> return unit S and scalar theta
    wtheta = Vtheta(1:3);
    vtheta = Vtheta(4:6);

    if norm(wtheta) > 1e-9
        theta = norm(wtheta);
        Sunit = Vtheta / theta;
    else
        theta = norm(vtheta);
        if theta < eps
            theta = 0;
            Sunit = zeros(6,1);
        else
            Sunit = Vtheta / theta;
        end
    end
end
