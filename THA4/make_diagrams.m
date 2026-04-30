%% make_diagrams.m — THA4 Conceptual Diagrams
%  Generates the 5 "TO CREATE" figures from REPORT_OUTLINE.md.
%  Saves PNGs to THA4/figures/.
%
%  FIG 1: Admittance vs. Impedance block diagrams
%  FIG 2: KR120 + tool rendering (RST)
%  FIG 3: Spatial velocity / Jacobian pitfall schematic
%  FIG 4: QP control loop block diagram
%  FIG 5: 3 mm sphere constraint cartoon

clear; close all;
addpath(genpath(fileparts(mfilename('fullpath')) + "/../"));

fig_dir = fullfile(fileparts(mfilename('fullpath')), 'figures');
if ~exist(fig_dir, 'dir'), mkdir(fig_dir); end

set(groot, 'defaultFigureColor','w','defaultAxesColor','w', ...
           'defaultTextColor','k','defaultAxesXColor','k','defaultAxesYColor','k');

%% =========================================================================
%  FIG 1 — Admittance vs. Impedance block diagrams
%  =========================================================================
hf1 = figure('Color','w','Position',[50 50 1100 430]);

ax1L = axes('Parent',hf1,'Position',[0.03 0.12 0.45 0.78],'Visible','off');
hold(ax1L,'on'); xlim(ax1L,[-1 10]); ylim(ax1L,[0 5]);

ax1R = axes('Parent',hf1,'Position',[0.54 0.12 0.45 0.78],'Visible','off');
hold(ax1R,'on'); xlim(ax1R,[-1 10]); ylim(ax1R,[0 5]);

bw = 1.9;  bh = 0.95;  ym = 2.5;
blu = [0.87 0.87 1.00];
red = [1.00 0.87 0.87];

% ---- Admittance (blue) ----
blk(ax1L, 0.0, ym-bh/2, bw, bh, {'Force','Sensor'},         blu);
blk(ax1L, 2.3, ym-bh/2, bw, bh, {'Admittance','F\rightarrow v_d'}, blu);
blk(ax1L, 4.6, ym-bh/2, bw, bh, {'Position','Controller'},  blu);
blk(ax1L, 6.9, ym-bh/2, bw, bh, {'Robot'},                  blu);

plot(ax1L,[-0.7 0.0],[ym ym],'k-','LineWidth',1.2);
arw(ax1L,[-0.5 ym],[0.0 ym]);
text(ax1L,-0.8,ym,'Op.','FontSize',8,'HorizontalAlignment','right');
arw(ax1L,[0.0+bw ym],[2.3 ym]);
arw(ax1L,[2.3+bw ym],[4.6 ym]);
arw(ax1L,[4.6+bw ym],[6.9 ym]);
text(ax1L,1.4,  ym+0.28,'F_{ext}',    'FontSize',8,'HorizontalAlignment','center');
text(ax1L,3.65, ym+0.28,'v_d',         'FontSize',8,'HorizontalAlignment','center');
text(ax1L,5.95, ym+0.28,'\tau',         'FontSize',8,'HorizontalAlignment','center');

% Force feedback loop
cx1L = bw/2;  cx2L = 6.9+bw/2;
plot(ax1L,[cx2L cx2L cx1L cx1L],[ym-bh/2, 0.7, 0.7, ym-bh/2],'k-','LineWidth',1.2);
arw(ax1L,[cx1L cx1L],[0.72 ym-bh/2]);
text(ax1L,(cx1L+cx2L)/2, 0.38,'F_{ext} measured at EE (F/T sensor)', ...
    'FontSize',7,'HorizontalAlignment','center','Color',[0.45 0.45 0.45]);
title(ax1L,'Admittance Control','FontSize',10,'FontWeight','bold','Color','k');

% ---- Impedance (red) ----
blk(ax1R, 0.0, ym-bh/2, bw, bh, {'Position','Sensor'},      red);
blk(ax1R, 2.3, ym-bh/2, bw, bh, {'Impedance','\Deltax\rightarrow F_{cmd}'}, red);
blk(ax1R, 4.6, ym-bh/2, bw, bh, {'Torque','Controller'},    red);
blk(ax1R, 6.9, ym-bh/2, bw, bh, {'Robot'},                  red);

plot(ax1R,[-0.7 0.0],[ym ym],'k-','LineWidth',1.2);
arw(ax1R,[-0.5 ym],[0.0 ym]);
text(ax1R,-0.8,ym,'x_{ref}','FontSize',8,'HorizontalAlignment','right');
arw(ax1R,[0.0+bw ym],[2.3 ym]);
arw(ax1R,[2.3+bw ym],[4.6 ym]);
arw(ax1R,[4.6+bw ym],[6.9 ym]);
text(ax1R,1.4,  ym+0.28,'\Deltax',     'FontSize',8,'HorizontalAlignment','center');
text(ax1R,3.65, ym+0.28,'F_{cmd}',     'FontSize',8,'HorizontalAlignment','center');
text(ax1R,5.95, ym+0.28,'\tau_{cmd}',  'FontSize',8,'HorizontalAlignment','center');

cx1R = bw/2;  cx2R = 6.9+bw/2;
plot(ax1R,[cx2R cx2R cx1R cx1R],[ym-bh/2, 0.7, 0.7, ym-bh/2],'k-','LineWidth',1.2);
arw(ax1R,[cx1R cx1R],[0.72 ym-bh/2]);
text(ax1R,(cx1R+cx2R)/2, 0.38,'q measured (joint encoders)', ...
    'FontSize',7,'HorizontalAlignment','center','Color',[0.45 0.45 0.45]);
title(ax1R,'Impedance Control','FontSize',10,'FontWeight','bold','Color','k');

annotation(hf1,'line',[0.505 0.505],[0.04 0.97],'Color',[0.65 0.65 0.65],'LineStyle','--');
sgtitle(hf1,'Admittance vs. Impedance VF Architectures','Color','k', ...
    'FontSize',12,'FontWeight','bold');
exportgraphics(hf1,fullfile(fig_dir,'fig_diagram1_admittance_vs_impedance.png'), ...
    'Resolution',150);
fprintf('Saved: fig_diagram1_admittance_vs_impedance.png\n');

%% =========================================================================
%  FIG 2 — KR120 RST rendering with 100 mm tool
%  =========================================================================
try
    robot = KR120_params();
    rbt   = load_kr120_rst();
    q_nice = [0.3; -0.35; 0.45; 0.2; 0.4; 0.1];
    L_tool = 0.1;

    hf2 = figure('Color','w','Position',[50 50 800 680]);
    ax2 = axes('Parent',hf2);
    hold(ax2,'on');  grid(ax2,'on');  view(ax2,[-42 22]);
    set(ax2,'Color','w','XColor','k','YColor','k','ZColor','k', ...
        'GridColor',[0.82 0.82 0.82]);
    camlight(ax2,'headlight');

    show(rbt, q_nice(:)', 'Parent',ax2,'PreservePlot',true, ...
         'Visuals','on','Frames','off');

    set(ax2,'Color','w','XColor','k','YColor','k','ZColor','k', ...
        'GridColor',[0.82 0.82 0.82]);
    grid(ax2,'on');  view(ax2,[-42 22]);
    xlim(ax2,[-0.5 3.0]);  ylim(ax2,[-1.5 1.5]);  zlim(ax2,[-0.3 3.0]);
    xlabel(ax2,'X (m)');  ylabel(ax2,'Y (m)');  zlabel(ax2,'Z (m)');

    for ff = findall(0,'Type','figure')'
        if ff ~= hf2, close(ff); end
    end
    set(0,'CurrentFigure',hf2);

    T_ee  = FK_space(robot.M, robot.Slist, q_nice);
    p_ee  = T_ee(1:3,4);
    d_ee  = T_ee(1:3,1:3)*[0;0;1];
    p_tip = p_ee + d_ee*L_tool;
    plot3(ax2,[p_ee(1) p_tip(1)],[p_ee(2) p_tip(2)],[p_ee(3) p_tip(3)], ...
        'r-','LineWidth',6);
    plot3(ax2,p_tip(1),p_tip(2),p_tip(3),'r^','MarkerSize',8,'MarkerFaceColor','r');

    text(ax2, p_tip(1)+0.08, p_tip(2)+0.04, p_tip(3)+0.06, ...
        '100 mm tool tip','Color','r','FontSize',9,'FontWeight','bold');

    title(ax2,'KUKA KR120 R2500 Pro — 100 mm Cylindrical Tool (red)', ...
        'Color','k','FontSize',10);
    exportgraphics(hf2,fullfile(fig_dir,'fig_diagram2_kr120_tool.png'),'Resolution',150);
    fprintf('Saved: fig_diagram2_kr120_tool.png\n');
catch ME
    fprintf('WARNING: FIG 2 skipped (RST renderer unavailable) — %s\n', ME.message);
end

%% =========================================================================
%  FIG 3 — Spatial Jacobian / Jacobian pitfall schematic (2D arm)
%  =========================================================================
hf3 = figure('Color','w','Position',[50 50 900 600]);
ax3 = axes('Parent',hf3,'Visible','off');
hold(ax3,'on');
xlim(ax3,[-1.5 8.5]);  ylim(ax3,[-1.2 6.0]);

% World frame origin
plot(ax3,0,0,'k+','MarkerSize',16,'LineWidth',2.5);
arw_col(ax3,[0 0],[1.2 0],[0 0 0]);  text(ax3,1.35,-0.05,'x','FontSize',9,'FontWeight','bold');
arw_col(ax3,[0 0],[0 1.2],[0 0 0]);  text(ax3,-0.15,1.35,'z','FontSize',9,'FontWeight','bold');
text(ax3,0,-0.55,'{O} world frame','FontSize',8.5,'HorizontalAlignment','center','FontWeight','bold');

% 2-link arm
j1  = [0,   0  ];
j2  = [2.8, 2.4];
ee  = [4.8, 3.4];
d_tool = [0.62, -0.37];  d_tool = d_tool/norm(d_tool);
tip = ee + d_tool*0.85;

plot(ax3,[j1(1) j2(1)],[j1(2) j2(2)],'k-','LineWidth',5);
plot(ax3,[j2(1) ee(1)],[j2(2) ee(2)],'k-','LineWidth',5);
plot(ax3,j1(1),j1(2),'ko','MarkerSize',13,'MarkerFaceColor',[0.7 0.7 0.7],'LineWidth',1.5);
plot(ax3,j2(1),j2(2),'ko','MarkerSize',13,'MarkerFaceColor',[0.7 0.7 0.7],'LineWidth',1.5);
plot(ax3,ee(1),ee(2),'bs','MarkerSize',11,'MarkerFaceColor',[0.5 0.5 1.0],'LineWidth',1.5);
text(ax3,ee(1)+0.12,ee(2)+0.18,'EE','FontSize',9,'Color',[0 0 0.8],'FontWeight','bold');

% Tool shaft
plot(ax3,[ee(1) tip(1)],[ee(2) tip(2)],'r-','LineWidth',4);
plot(ax3,tip(1),tip(2),'r^','MarkerSize',9,'MarkerFaceColor','r','LineWidth',1.5);
text(ax3,tip(1)+0.12,tip(2),'tool tip','FontSize',9,'Color',[0.8 0 0],'FontWeight','bold');

% Position vectors from world origin
arw_col(ax3,[0 0],[ee(1) ee(2)],[0 0 0.7]);
text(ax3,ee(1)/2-0.4,ee(2)/2+0.18,'p_{ee}','FontSize',10,'Color',[0 0 0.7],'FontWeight','bold');
arw_col(ax3,[0 0],[tip(1) tip(2)],[0.75 0 0]);
text(ax3,tip(1)/2+0.35,tip(2)/2-0.25,'p_{tip}','FontSize',10,'Color',[0.75 0 0],'FontWeight','bold');

% "Jv body point" annotation at origin
plot(ax3,0,0,'g^','MarkerSize',12,'MarkerFaceColor','g','LineWidth',1.2);
text(ax3,0.1,-0.82,{'J_v: velocity of body pt','at world origin'}, ...
    'FontSize',8,'Color',[0 0.55 0],'FontWeight','bold');

% Angular velocity arc at EE
th_arc = linspace(0.25*pi, 0.85*pi, 40);
r_arc  = 0.55;
plot(ax3, ee(1)+r_arc*cos(th_arc), ee(2)+r_arc*sin(th_arc), 'm-','LineWidth',2.2);
n = numel(th_arc);
arw_col(ax3, ...
    [ee(1)+r_arc*cos(th_arc(n-1)), ee(2)+r_arc*sin(th_arc(n-1))], ...
    [ee(1)+r_arc*cos(th_arc(n)),   ee(2)+r_arc*sin(th_arc(n))], ...
    [0.6 0 0.7]);
text(ax3, ee(1)-0.85, ee(2)+0.72, '\omega','FontSize',13,'Color',[0.6 0 0.7],'FontWeight','bold');

% Correction vector omega x p_tip
p_hat = [tip(1) tip(2)] / norm([tip(1) tip(2)]);
perp  = [-p_hat(2), p_hat(1)] * 0.9;
arw_col(ax3,[tip(1) tip(2)],[tip(1)+perp(1) tip(2)+perp(2)],[0.85 0.4 0]);
text(ax3, tip(1)+perp(1)+0.1, tip(2)+perp(2)+0.05, '\omega\timesp_{tip}', ...
    'FontSize',10,'Color',[0.85 0.4 0],'FontWeight','bold');

% Formula box
annotation(hf3,'textbox',[0.55 0.03 0.43 0.30], ...
    'String',{ ...
      'J_{linear}(p) = J_v - skew(p) \cdot J_\omega', ...
      '', ...
      'Common pitfall: use skew(r) instead of skew(p_{tip})', ...
      '  r = R_{ee}[0;0;L] — offset only', ...
      '  p_{tip} — absolute world-frame position', ...
      '', ...
      'Error = skew(p_{ee}) \cdot J_\omega \sim arm-length scale'}, ...
    'FontSize',8,'BackgroundColor',[1 1 0.85],'EdgeColor',[0.7 0.6 0], ...
    'FitBoxToText','off','Interpreter','tex');

title(ax3,'Spatial Jacobian: J_{linear}(p_{tip}) = J_v - skew(p_{tip}) \cdot J_\omega', ...
    'FontSize',11,'Color','k');
exportgraphics(hf3,fullfile(fig_dir,'fig_diagram3_jacobian_schematic.png'),'Resolution',150);
fprintf('Saved: fig_diagram3_jacobian_schematic.png\n');

%% =========================================================================
%  FIG 4 — QP control loop block diagram
%  =========================================================================
hf4 = figure('Color','w','Position',[50 50 1060 390]);
ax4 = axes('Parent',hf4,'Visible','off');
hold(ax4,'on');  xlim(ax4,[0 14]);  ylim(ax4,[0 5.5]);

bw4 = 1.65;  bh4 = 1.0;  ym4 = 3.2;
grn = [0.88 1.00 0.88];
yel = [1.00 1.00 0.82];
gry = [0.91 0.91 0.91];

blk(ax4,  0.1, ym4-bh4/2, bw4, bh4, {'q_k'},                               gry);
blk(ax4,  2.1, ym4-bh4/2, bw4, bh4, {'FK + Jacobian','J_{tip},  T_{ee}'},  grn);
blk(ax4,  4.3, ym4-bh4/2, bw4, bh4, {'Build QP','H, f, A, b, lb, ub'},     yel);
blk(ax4,  6.5, ym4-bh4/2, bw4, bh4, {'quadprog'},                          [0.87 0.95 1.00]);
blk(ax4,  8.5, ym4-bh4/2, bw4, bh4, {'dq*'},                               gry);
blk(ax4, 10.5, ym4-bh4/2, bw4, bh4, {'q_{k+1} = q_k + dq*'},              grn);

arw(ax4,[0.1+bw4  ym4],[2.1  ym4]);
arw(ax4,[2.1+bw4  ym4],[4.3  ym4]);
arw(ax4,[4.3+bw4  ym4],[6.5  ym4]);
arw(ax4,[6.5+bw4  ym4],[8.5  ym4]);
arw(ax4,[8.5+bw4  ym4],[10.5 ym4]);

% Parameters input from above into "Build QP"
cx_bqp = 4.3 + bw4/2;
blk(ax4, cx_bqp-1.15, 4.4, 2.3, 0.75, {'p_{goal}, \mu, \lambda, wall, d_{max}'}, [1.00 0.92 0.80]);
arw(ax4,[cx_bqp  4.4],[cx_bqp  ym4+bh4/2]);

% Convergence decision diamond (approximated as a box)
cx_chk = 10.5 + bw4/2;
blk(ax4, 10.5, 0.5, bw4, 0.8, {'converged?'}, [1.0 0.87 0.87]);
arw(ax4,[cx_chk  ym4-bh4/2],[cx_chk  1.3]);
text(ax4, cx_chk+0.05, 1.15-0.1, '\downarrow', 'FontSize',10,'HorizontalAlignment','center');
text(ax4, 12.3, 0.9, 'YES \rightarrow STOP','FontSize',8,'Color',[0.7 0 0]);

% Feedback dashed loop
fb_y = 0.35;
plot(ax4,[cx_chk, cx_chk, 0.1+bw4/2, 0.1+bw4/2], ...
         [0.5, fb_y, fb_y, ym4-bh4/2], 'k--','LineWidth',1.3);
arw(ax4,[0.1+bw4/2  fb_y+0.02],[0.1+bw4/2  ym4-bh4/2]);
text(ax4,5.5,fb_y+0.12,'NO — feed q_{k+1} back as q_k','FontSize',7.5, ...
    'HorizontalAlignment','center','Color',[0.4 0.4 0.4]);

title(ax4,'QP-Based Velocity Control Loop','FontSize',11,'Color','k');
exportgraphics(hf4,fullfile(fig_dir,'fig_diagram4_qp_loop.png'),'Resolution',150);
fprintf('Saved: fig_diagram4_qp_loop.png\n');

%% =========================================================================
%  FIG 5 — 3 mm sphere constraint cartoon
%  =========================================================================
hf5 = figure('Color','w','Position',[50 50 680 580]);
ax5 = axes('Parent',hf5,'Visible','off');
hold(ax5,'on');  axis(ax5,'equal');
xlim(ax5,[-2.8 2.8]);  ylim(ax5,[-3.0 2.8]);

R = 1.6;
th = linspace(0, 2*pi, 300);
xc = R*cos(th);  yc = R*sin(th);

% Outer region (light red — forbidden escape)
fill(ax5,[-2.8 2.8 2.8 -2.8],[-3.0 -3.0 2.8 2.8],[1.0 0.87 0.87], ...
    'EdgeColor','none','FaceAlpha',0.45);
% Inner region (light green — retained / safe)
fill(ax5,xc,yc,[0.85 1.00 0.85],'EdgeColor','none','FaceAlpha',0.6);

% Sphere boundary
plot(ax5,xc,yc,'k-','LineWidth',2.2);

% p_goal at centre
plot(ax5,0,0,'k*','MarkerSize',18,'LineWidth',2.2);
text(ax5,0.15,-0.3,'p_{goal}','FontSize',12,'FontWeight','bold','Color','k');

% Tool tip inside sphere
ptx = 0.70;  pty = 0.85;
plot(ax5,ptx,pty,'bo','MarkerSize',12,'MarkerFaceColor','b','LineWidth',1.8);
text(ax5,ptx+0.15,pty+0.10,'p_{tip}','FontSize',11,'Color','b','FontWeight','bold');

% n_hat_out arrow (radially outward from p_goal through p_tip)
n_hat = [ptx pty]/norm([ptx pty]);
scale_n = 0.85;
arw_col(ax5,[ptx pty],[ptx+n_hat(1)*scale_n, pty+n_hat(2)*scale_n],[0.85 0 0]);
text(ax5, ptx+n_hat(1)*scale_n+0.12, pty+n_hat(2)*scale_n+0.04, ...
    '{\hat n}_{out}','FontSize',12,'Color',[0.85 0 0],'FontWeight','bold');

% Allowed inward direction (green)
arw_col(ax5,[ptx pty],[ptx-n_hat(1)*0.65, pty-n_hat(2)*0.65],[0 0.55 0]);
text(ax5, ptx-n_hat(1)*0.65-0.18, pty-n_hat(2)*0.65-0.18, ...
    'allowed','FontSize',9,'Color',[0 0.55 0]);

% Radius label
plot(ax5,[0 R*cos(pi/5)],[0 R*sin(pi/5)],'k-','LineWidth',1.0);
text(ax5, R*cos(pi/5)/2+0.05, R*sin(pi/5)/2+0.1,'3 mm','FontSize',9,'Color','k');

% Region labels
text(ax5, 1.9, 1.7,{'forbidden','(escape)'},'FontSize',9,'Color',[0.72 0 0], ...
    'HorizontalAlignment','center');
text(ax5,-1.1,-0.9,{'retained','(safe)'},'FontSize',9,'Color',[0 0.5 0], ...
    'HorizontalAlignment','center');

% Constraint equation
text(ax5,0,-2.25,'{\hat n}_{out}^T \cdot J_{tip} \cdot dq \leq 0   (active once inside sphere)', ...
    'FontSize',10,'HorizontalAlignment','center','FontWeight','bold','Color','k');
text(ax5,0,-2.72,'Tip can spiral inward toward p_{goal} but cannot escape back out', ...
    'FontSize',9,'HorizontalAlignment','center','Color',[0.35 0.35 0.35]);

title(ax5,'3 mm Sphere Retention Constraint  (one-sided, radially inward)', ...
    'FontSize',11,'Color','k');
exportgraphics(hf5,fullfile(fig_dir,'fig_diagram5_sphere_constraint.png'),'Resolution',150);
fprintf('Saved: fig_diagram5_sphere_constraint.png\n');

fprintf('\nAll diagrams complete.\n');

%% =========================================================================
%  LOCAL HELPERS
%  =========================================================================

function blk(ax, x, y, w, h, txt, clr)
%  Draw a rounded rectangle with centred text.  txt may be a cell array.
    rectangle(ax,'Position',[x y w h],'Curvature',0.12, ...
        'EdgeColor','k','LineWidth',1.4,'FaceColor',clr);
    text(ax, x+w/2, y+h/2, txt, ...
        'HorizontalAlignment','center','VerticalAlignment','middle', ...
        'FontSize',8,'Color','k','Interpreter','tex');
end

function arw(ax, p1, p2)
%  Black arrow from p1=[x1 y1] to p2=[x2 y2] in data coords.
    arw_col(ax, p1, p2, [0 0 0]);
end

function arw_col(ax, p1, p2, col)
%  Coloured arrow from p1=[x1 y1] to p2=[x2 y2] in data coords.
    sc = 0.025 * max(diff(xlim(ax)), diff(ylim(ax)));
    x1=p1(1); y1=p1(2); x2=p2(1); y2=p2(2);
    plot(ax,[x1 x2],[y1 y2],'-','Color',col,'LineWidth',1.3);
    dx=x2-x1; dy=y2-y1; len=hypot(dx,dy);
    if len < 1e-9, return; end
    ux=dx/len; uy=dy/len; px=-uy; py=ux;
    hw=sc*0.48; hl=sc*0.95;
    fill(ax, [x2-hl*ux+hw*px, x2, x2-hl*ux-hw*px], ...
             [y2-hl*uy+hw*py, y2, y2-hl*uy-hw*py], col, 'EdgeColor',col);
end
