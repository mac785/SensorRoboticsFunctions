function [P_dimple, t_tip] = pivot_cal(G_frames)
    %----------------- DETAILS ------------------------%
    % Performs pivot calibration given tracked marker positions across
    % multiple frames where the probe tip is held fixed at a dimple.
    %
    % Builds and solves the overdetermined linear system:
    %   [R_k | -I3] * [t_tip; P_dimple] = -p_k   for each frame k
    %
    % Frame 1 defines the probe's local reference coordinate system.
    % All input positions must be in the same base coordinate frame
    % (EM base frame for EM pivot; EM base frame for optical pivot
    %  after transforming H markers with inv(F_D)).
    %
    %----------------- Inputs -------------------------%
    % G_frames = N_frames x 1 cell array
    %   G_frames{k} = N_G x 3 matrix of marker positions for frame k
    %
    %----------------- Outputs ------------------------%
    % P_dimple = 3x1 position of the fixed dimple in the base frame
    % t_tip    = 3x1 position of probe tip in the frame-1 coordinate system

    N_frames = numel(G_frames);
    G_ref    = G_frames{1};  % frame 1 defines the probe's local reference

    A = zeros(3*N_frames, 6);
    b = zeros(3*N_frames, 1);

    for k = 1:N_frames
        FG_k = cloud2cloud(G_ref, G_frames{k});
        R_k  = FG_k(1:3, 1:3);
        p_k  = FG_k(1:3, 4);

        idx = (k-1)*3 + (1:3);
        A(idx, :) = [R_k, -eye(3)];  % [R_k | -I3]
        b(idx)    = -p_k;
    end

    % Least-squares solve: x = [t_tip; P_dimple]
    x = A \ b;

    t_tip    = x(1:3);
    P_dimple = x(4:6);
end
