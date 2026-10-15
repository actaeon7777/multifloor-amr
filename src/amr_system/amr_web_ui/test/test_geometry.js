/* Run with gjs; no browser or npm is needed for coordinate regression checks. */
globalThis.window = {};
const GLib = imports.gi.GLib;
const [ok, bytes] = GLib.file_get_contents(ARGV[0]);
if (!ok) throw new Error('Cannot read geometry.js');
new Function(new TextDecoder().decode(bytes))();
const G = window.AMRGeometry;
function near(a,b) { if (Math.abs(a-b)>1e-8) throw new Error(`${a} != ${b}`); }
const info={origin:[-3.8,-1.54,Math.PI/2],resolution:.05};
const p=G.gridToWorld(120,90,info), cell=G.worldToGrid(p.x,p.y,info);
near(cell.x,120);near(cell.y,90);
for(const scale of [2,12,80]) {
 const camera={x:21.55,y:10.01,scale};
 const q=G.screenToWorld(G.worldToScreen(p,camera,1400,900),camera,1400,900);
 near(q.x,p.x);near(q.y,p.y);
}
near(G.heading({x:0,y:0},{x:0,y:1}),Math.PI/2);
const raw = {frame:'odom', origin:[-2.15,-2.85,0]};
for (const info of [raw, {...raw,transform_valid:false},
  {...raw,transform_valid:true}, {...raw,transform_valid:true,render_origin:[NaN,0,0]},
  {...raw,transform_valid:true,render_origin:[1,2,Infinity]},
  {...raw,transform_valid:true,render_origin:[1,2]}]) {
  if (G.renderOrigin(info) !== null) throw new Error('Untransformed/invalid grid must be hidden');
}
const transformed = {...raw,transform_valid:true,render_origin:[9.25,.5,.1]};
near(G.renderOrigin(transformed)[0],9.25);
if (G.renderOrigin({...transformed,transform_valid:false}) !== null) throw new Error('Stale TF must be hidden');
near(G.renderOrigin({frame:'map',origin:[1,2,0],render_origin:[1,2,0],transform_valid:true})[1],2);
print('Coordinates, rotated origins, heading and costmap transform guard: OK');
