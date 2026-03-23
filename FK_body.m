function T = FK_body(M, Blist, thetalist, visualize, q_joints_home)
% FK_body: Computes forward kinematics using body-form PoE and optionally
%          displays the robot's frames and screw axes graphically.
%
% T = M * exp([B1]theta1) * ... * exp([Bn]thetan)
%
% Inputs:
%   M              - 4x4 home configuration of end-effector
%   Blist          - 6xn matrix of body-form screw axes (columns)
%   thetalist      - nx1 vector of joint angles (radians)
%   visualize      - (optional) true to plot frames and screw axes (default: false)
%   q_joints_home  - (optional) 3×n physical joint positions at home config
%                    (see FK_space for why this matters)
%
% Output:
%   T         - 4x4 end-effector transformation matrix

    if nargin < 4, visualize = false; end
    if nargin < 5, q_joints_home = []; end

    T = M;
    n = length(thetalist);
    for i = 1:n
        T = T * MatrixExp6(vecToSE3(Blist(:,i) * thetalist(i)));
    end

    if ~visualize, return; end

    %% ---- Visualization ----
    % Body screw axes Bi are defined in the end-effector body frame at home.
    % Convert to space-form axes for plotting: Si = Ad(M) * Bi
    AdM = adjoint_map(M);
    Slist_space = zeros(size(Blist));
    for i = 1:size(Blist, 2)
        Slist_space(:,i) = AdM * Blist(:,i);
    end

    % Delegate to FK_space visualization (same robot geometry)
    FK_space(M, Slist_space, thetalist, true, q_joints_home);
    title('FK Body Form: Joint Frames and Screw Axes (expressed in space frame)');
end
