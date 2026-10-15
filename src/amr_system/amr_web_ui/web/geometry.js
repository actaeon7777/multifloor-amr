/* Shared coordinate functions. Screen Y is down; ROS map Y is up. */
window.AMRGeometry = {
  renderOrigin(info) {
    if (info?.transform_valid !== true) return null;
    const origin = info.render_origin;
    return Array.isArray(origin) && origin.length === 3 && origin.every(Number.isFinite) ? origin : null;
  },
  gridToWorld(x, y, info) {
    const o = info.render_origin || info.origin, c = Math.cos(o[2]), s = Math.sin(o[2]);
    return {x:o[0] + (c*x-s*y)*info.resolution, y:o[1] + (s*x+c*y)*info.resolution};
  },
  worldToGrid(x, y, info) {
    const o = info.render_origin || info.origin, c = Math.cos(o[2]), s = Math.sin(o[2]);
    const dx = x-o[0], dy = y-o[1];
    return {x:(c*dx+s*dy)/info.resolution, y:(-s*dx+c*dy)/info.resolution};
  },
  worldToScreen(p, camera, width, height) {
    return {x:width/2 + (p.x-camera.x)*camera.scale, y:height/2 - (p.y-camera.y)*camera.scale};
  },
  screenToWorld(p, camera, width, height) {
    return {x:camera.x + (p.x-width/2)/camera.scale, y:camera.y - (p.y-height/2)/camera.scale};
  },
  heading(start, end) { return Math.atan2(end.y-start.y, end.x-start.x); },
};
