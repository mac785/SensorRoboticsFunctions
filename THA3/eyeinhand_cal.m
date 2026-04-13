function [T, results] = eyeinhand_cal(q_Robot_config,q_camera_config,t_Robot_config,t_camera_config)
    %   This function does an eye - in - hand calibration given several
    %   configurations of the robot. This function will return a
    %   transformation matrix between the camera's frame and the end effectors
    %   frame.
    %
    %   The inputs of this function could be adjusted to take in
    %   SE(3) matrices.
    %
    %   This function solves the AX = BX problem using the quanternion
    %   approach
    %
    % --------------- INPUTS -------------------- %
    %   quanternion representation of rotation in SE(3) matrix
    %       q_robot_config = [q1, q2, ..., qn]'
    %       q_camera_config = [q1, q2, ..., qn]'
    %
    %   origin translation vector
    %       t_Robot_config = [t1, t2, ..., tn]'
    %       t_camera_config = [t1, t2,..., tn]'
    %
    %   N = number of configurations
    % ------------------------------------------- %

    % --------------- OUTPUTS -------------------- %
    %   T = [Rx Px;0 1] - transformation matrix between the camera and end
    %   effector frame
    % -------------------------------------------- %

    %1 - solve for R_x first

    %1.1 - setup parameters and initilize some arrays
    N = size(q_Robot_config,1); %number of configurations
    k = N-1; %number of consecutive configurations
    block_size = 4; %block size for making M matrix
    M = zeros(4*k,4); %initialize M matrix (k consecutive configurations)
    R_E_array = quat2rotm_my(q_Robot_config); %3x3xN
    R_S_array = quat2rotm_my(q_camera_config); %3x3xN
    R_A_array = zeros(3,3,k); %3x3xk
    R_B_array = zeros(3,3,k); %3x3xk
    P_A_matrix = zeros(k,3); %kx3 configuragation translation matrix (row vectors)
    P_B_matrix = zeros(k,3); %kx3 configuragation translation matrix (row vectors)

    %1.2 - obtain M
    for i = 1:k
        %1.2.1 - find quaternions A and B using quat2rot and then rot2quat
        %q_a
        E_i = R_E_array(:,:,i);
        E_i_1 = R_E_array(:,:,i+1);
        R_a = E_i\E_i_1;
        R_A_array(:,:,i) = R_a; %store for later
        q_a = rotm2quat_my(R_a);

        %q_b
        S_i = R_S_array(:,:,i);
        S_i_1 = R_S_array(:,:,i+1);
        R_b = S_i/S_i_1;
        R_B_array(:,:,i) = R_b; %store for later
        q_b = rotm2quat_my(R_b);
        

        if dot(q_a, q_b) < 0 %check that signs are correct
            q_b = -q_b;
        end

        %1.2.2 - construct each M_k block
        %extract scalar S_a and S_b
        S_a = q_a(1);
        S_b = q_b(1);

        %extract V_a and V_b and make column vectors
        V_a = q_a(2:end)';
        V_b = q_b(2:end)';

        %solve for M_i blocks
        M_1 = (S_a - S_b);
        M_2 = -(V_a - V_b)';
        M_3 = (V_a - V_b);
        M_4 = (S_a-S_b)*eye(3) + vecToSO3(V_a + V_b);

        M_i = [M_1 M_2; M_3 M_4];
        idx = (i-1)*block_size + (1:block_size);
        % 
        M(idx,:) = M_i; %make M

        %for later - find P_b_k_matrix and P_a_k_matrix. UNSURE OF THE EXACT ORDER OF SUBTRACTION
        P_A_matrix(i,:) = t_Robot_config(i,:) - t_Robot_config(i+1,:); %consecutive robot translation
        P_B_matrix(i,:) = t_camera_config(i,:) - t_camera_config(i+1,:); %consecutive camera translation
    end

    %1.3 - Do SVD to get V
    [~,~,V] = svd(M); %svd of M to get q_x
    q_x = V(:,4)'; % q_x is the fourth column of V - in row vector form

    R_x = quat2rotm_my(q_x); %covert to SO(3)

    %2 now solve for P_x (translation component of transformation matrix)
    %using least squares
    
    %2.1 - create A matrix and b vector. each kth block is a configuration
    A = zeros((3*k), 3);%3kx3 matrix
    b = zeros((3*k),1);%3kx1 column vector
    for i = 1:k
        idx = (i-1)*3 + (1:3);
        A_i = R_A_array(:,:,i) - eye(3);
        b_i = R_x*P_B_matrix(i,:)' - P_A_matrix(i,:)'; %column vector
        A(idx,:) = A_i;   % 3x3
        b(idx)   = b_i;   % 3x1
    end
    
    %2.2 - solve lsqr for P_x
    [P_x,flag, relres] = lsqr(A,b);

    %3 - set outputs

    T = [R_x P_x; zeros(1,3) 1];
    results = [flag relres]; %results of lsqr
   
end
