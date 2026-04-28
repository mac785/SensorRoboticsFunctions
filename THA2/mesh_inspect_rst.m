%% mesh_inspect_rst.m
% 4-view static figure of the KR120 R2500 Pro using the RST renderer
% (importrobot + show). Saves to media/fig_inspect_kr120_home.png.
%
% Run from the THA2/ directory:
%   >> mesh_inspect_rst

addpath('.')
addpath('helpers');
rbt    = load_kr120_rst();
config = homeConfiguration(rbt);

if ~exist('media','dir'), mkdir('media'); end

views = {[0 0],'Front (XZ)'; [-90 0],'Side (YZ)'; [-35 25],'Isometric'; [0 90],'Top (XY)'};

fig = figure('Color','w','Visible','off','Position',[0 0 1600 900]);
for vi = 1:4
    ax = subplot(1,4,vi,'Parent',fig);
    show(rbt, config, 'Parent',ax, 'PreservePlot',false, 'Visuals','on', 'Frames','off');
    set(ax,'Color','w','XColor','k','YColor','k','ZColor','k','GridColor',[0.82 0.82 0.82]);
    title(ax, views{vi,2}, 'Color','k','FontSize',10);
    view(ax, views{vi,1});
end
sgtitle(fig,'KR120 R2500 Pro — Home (theta = 0)', ...
        'Color','k','FontSize',13,'FontWeight','bold');

saveas(fig, fullfile('media','fig_inspect_kr120_home.png'));
close(fig);
fprintf('Saved %s\n', fullfile('media','fig_inspect_kr120_home.png'));
