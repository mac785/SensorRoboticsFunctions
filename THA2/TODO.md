# THA2 — Pending Fixes

## render_all_figs.m (RST mode)
- FK figures in RST mode only show black skeleton lines instead of the full 3D RST model.
  The STL mesh overlay was removed for the RST path but no RST equivalent rendering was added
  to the FK figures. Needs the RST robot rendered onto each FK/ellipsoid figure.

## Performance
- Explore hardware acceleration (GPU) or other optimisation to speed up RST
  animations — robot_animation_rst.m is noticeably slow frame-to-frame.

