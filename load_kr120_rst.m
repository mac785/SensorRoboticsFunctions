function rbt = load_kr120_rst()
% load_kr120_rst: Load the KR120 R2500 Pro into a rigidBodyTree using the
% Robotics System Toolbox, with package:// URIs patched to local paths.
%
% The URDF in meshes/kr120r2500pro.urdf references:
%   package://kuka_kr120_support/meshes/kr120r2500pro/visual/linkX.dae
%   package://kuka_kr120_support/meshes/kr120r2500pro/collision/linkX.stl
% Both are resolved to the local meshes/ directory.
%
% Output:
%   rbt - rigidBodyTree object ready for show() / getTransform()

    urdf_src  = fullfile(fileparts(mfilename('fullpath')), 'meshes', 'kr120r2500pro.urdf');
    mesh_dir  = fullfile(fileparts(mfilename('fullpath')), 'meshes');
    urdf_tmp  = fullfile(tempdir, 'kr120r2500pro_patched.urdf');

    %% Patch package:// URIs → absolute local paths
    txt = fileread(urdf_src);

    % Visual DAE: package://kuka_kr120_support/meshes/kr120r2500pro/visual/X.dae
    txt = regexprep(txt, ...
        'package://kuka_kr120_support/meshes/kr120r2500pro/visual/([^"]+\.dae)', ...
        [strrep(mesh_dir, '\', '/') '/$1']);

    % Collision STL: package://kuka_kr120_support/meshes/kr120r2500pro/collision/X.stl
    txt = regexprep(txt, ...
        'package://kuka_kr120_support/meshes/kr120r2500pro/collision/([^"]+\.stl)', ...
        [strrep(mesh_dir, '\', '/') '/$1']);

    %% Inject <material> color tags into each <visual> element
    % Colors taken from DAE diffuse values:
    %   RAL9005 (near-black) : base_link, link_6
    %   RAL2003 (KUKA orange): link_1 through link_5
    black  = '<material name="ral9005"><color rgba="0.02 0.02 0.02 1"/></material>';
    orange = '<material name="ral2003"><color rgba="0.80 0.33 0.11 1"/></material>';

    link_colors = containers.Map( ...
        {'base_link','link_1','link_2','link_3','link_4','link_5','link_6'}, ...
        {black, orange, orange, orange, orange, orange, black});

    links = keys(link_colors);
    for ii = 1:numel(links)
        lname = links{ii};
        mat   = link_colors(lname);
        % Insert material tag before </visual> only in the visual block for this link's DAE
        txt = regexprep(txt, ...
            sprintf('(<mesh filename="[^"]*%s\\.dae"[^/]*/>[^<]*</geometry>[^<]*)(</visual>)', lname), ...
            ['$1' mat '$2']);
    end

    fid = fopen(urdf_tmp, 'w');
    fwrite(fid, txt);
    fclose(fid);

    %% Import
    rbt = importrobot(urdf_tmp);
    rbt.DataFormat = 'row';   % joints as 1×6 row vector — convenient for getTransform()

    fprintf('Loaded rigidBodyTree: %d bodies, %d joints\n', ...
            rbt.NumBodies, numel(rbt.homeConfiguration));
end
