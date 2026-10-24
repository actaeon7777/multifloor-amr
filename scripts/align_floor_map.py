#!/usr/bin/env python3
"""Resample another floor's SLAM map into the reference floor's map frame.

Floor switches keep the pose numbers across maps (mission_node RELOCALIZE seeds the
new map with the old pose), so every floor map must share one frame around the
elevator.  The transform is estimated from the walls (coarse 1 deg FFT correlation,
then a truncated-chamfer refinement), or given with --yaw/--tx/--ty.

  scripts/align_floor_map.py SRC.yaml REF.yaml OUT.yaml [--annotations IN.png OUT.png]

OUT keeps REF's cell lattice and grows to cover both maps.  The optional annotation
image (REF's grid) is padded with unknown so it stays aligned with OUT.
"""
import argparse
import math
from pathlib import Path
import numpy as np
from PIL import Image
from scipy import ndimage, signal
import yaml

OCCUPIED, FREE, UNKNOWN = 0, 254, 205


def load(path):
    path = Path(path)
    meta = yaml.safe_load(path.read_text())
    grid = np.asarray(Image.open(path.parent/meta['image']).convert('L'))
    return grid, float(meta['resolution']), tuple(float(v) for v in meta['origin'][:2]), meta


def cells_to_world(rows, cols, shape, res, origin, sub=(.5, .5)):
    return np.stack([origin[0]+(cols+sub[0])*res, origin[1]+(shape[0]-rows-sub[1])*res], 1)


def rotate(points, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return points@np.array([[c, s], [-s, c]])


def raster(points, res, lo, shape):
    grid = np.zeros(shape, np.float32)
    c = ((points[:, 0]-lo[0])/res).astype(int); r = ((points[:, 1]-lo[1])/res).astype(int)
    keep = (c >= 0) & (c < shape[1]) & (r >= 0) & (r < shape[0])
    np.add.at(grid, (r[keep], c[keep]), 1)
    return grid


def estimate(src, dst, coarse_res=.2, fine_res=.05):
    """Return (yaw_rad, tx, ty, distances of src walls to dst walls)."""
    lo = dst.min(0)-5
    shape = tuple(np.ceil((dst.max(0)+5-lo)/coarse_res).astype(int))[::-1]
    target = ndimage.gaussian_filter(np.minimum(raster(dst, coarse_res, lo, shape), 1), 1.5)
    candidates = []
    for deg in np.arange(0., 360., 1.):
        pts = rotate(src, math.radians(deg)); slo = pts.min(0)
        sshape = tuple((np.ceil((pts.max(0)-slo)/coarse_res)+1).astype(int))[::-1]
        tmpl = ndimage.gaussian_filter(np.minimum(raster(pts, coarse_res, slo, sshape), 1), 1.5)
        corr = signal.fftconvolve(target, tmpl[::-1, ::-1], mode='full')
        r, c = np.unravel_index(np.argmax(corr), corr.shape)
        shift = lo+np.array([c-sshape[1]+1, r-sshape[0]+1])*coarse_res-slo
        candidates.append((corr[r, c]/tmpl.sum(), deg, *shift))
    candidates.sort(reverse=True)
    lo = dst.min(0)-5
    shape = tuple(np.ceil((dst.max(0)+5-lo)/fine_res).astype(int))[::-1]
    dist = ndimage.distance_transform_edt(raster(dst, fine_res, lo, shape) == 0)*fine_res

    def distances(p, trunc):
        pts = rotate(src, math.radians(p[0]))+np.array(p[1:])
        c = ((pts[:, 0]-lo[0])/fine_res).astype(int); r = ((pts[:, 1]-lo[1])/fine_res).astype(int)
        inside = (c >= 0) & (c < shape[1]) & (r >= 0) & (r < shape[0])
        d = np.full(len(pts), trunc); d[inside] = np.minimum(dist[r[inside], c[inside]], trunc)
        return d

    best = None
    for _, *start in candidates[:3]:
        cur, step_deg, step_xy, trunc = tuple(start), 1., .4, 1.
        cost = distances(cur, trunc).mean()
        while step_deg > .02 or step_xy > .01:
            moves = [(s*step_deg, 0, 0) for s in (1, -1)]+[(0, s*step_xy, 0) for s in (1, -1)] + \
                    [(0, 0, s*step_xy) for s in (1, -1)]
            trial = min((distances(tuple(a+b for a, b in zip(cur, m)), trunc).mean(), m) for m in moves)
            if trial[0] < cost-1e-6:
                cost, cur = trial[0], tuple(a+b for a, b in zip(cur, trial[1]))
                continue
            step_deg, step_xy = step_deg/2, step_xy/2
            if step_xy < .1 and trunc > .3:
                trunc = .3  # final stage: only walls already near a reference wall count
                cost = distances(cur, trunc).mean()
        d = distances(cur, 9.9)
        if best is None or (d < .15).mean() > (best[3] < .15).mean():
            best = (math.radians(cur[0]), cur[1], cur[2], d)
    return best


def resample(grid, res, origin, yaw, t, ref_res, ref_origin, ref_shape, margin=.3):
    """Nearest-cell labels on REF's lattice; walls are splatted so none thin out."""
    h, w = grid.shape
    rows, cols = np.nonzero(grid != UNKNOWN)
    pts = rotate(cells_to_world(rows, cols, grid.shape, res, origin), yaw)+t
    ref_hi = (ref_origin[0]+ref_shape[1]*ref_res, ref_origin[1]+ref_shape[0]*ref_res)
    lo = np.minimum(pts.min(0)-margin, ref_origin)
    hi = np.maximum(pts.max(0)+margin, ref_hi)
    pad_lo = np.ceil((np.array(ref_origin)-lo)/ref_res-1e-9).astype(int)
    pad_hi = np.ceil((hi-np.array(ref_hi))/ref_res-1e-9).astype(int)
    out_origin = (ref_origin[0]-pad_lo[0]*ref_res, ref_origin[1]-pad_lo[1]*ref_res)
    out_shape = (ref_shape[0]+pad_lo[1]+pad_hi[1], ref_shape[1]+pad_lo[0]+pad_hi[0])
    orows, ocols = np.indices(out_shape).reshape(2, -1)
    back = rotate(cells_to_world(orows, ocols, out_shape, ref_res, out_origin)-t, -yaw)
    sc = np.floor((back[:, 0]-origin[0])/res).astype(int)
    sr = h-1-np.floor((back[:, 1]-origin[1])/res).astype(int)
    inside = (sc >= 0) & (sc < w) & (sr >= 0) & (sr < h)
    out = np.full(out_shape[0]*out_shape[1], UNKNOWN, np.uint8)
    out[inside] = grid[sr[inside], sc[inside]]
    out = out.reshape(out_shape)
    wr, wc = np.nonzero(grid == OCCUPIED)
    for sub in ((.17, .17), (.5, .17), (.83, .17), (.17, .5), (.5, .5), (.83, .5), (.17, .83), (.5, .83), (.83, .83)):
        p = rotate(cells_to_world(wr, wc, grid.shape, res, origin, sub), yaw)+t
        c = np.floor((p[:, 0]-out_origin[0])/ref_res).astype(int)
        r = out_shape[0]-1-np.floor((p[:, 1]-out_origin[1])/ref_res).astype(int)
        out[r, c] = OCCUPIED
    return out, out_origin, (pad_lo, pad_hi)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('source'); parser.add_argument('reference'); parser.add_argument('output')
    parser.add_argument('--yaw', type=float, help='degrees; with --tx/--ty skips the estimate')
    parser.add_argument('--tx', type=float); parser.add_argument('--ty', type=float)
    parser.add_argument('--annotations', nargs=2, metavar=('IN_PNG', 'OUT_PNG'))
    args = parser.parse_args()
    grid, res, origin, _ = load(args.source)
    ref, ref_res, ref_origin, ref_meta = load(args.reference)
    if abs(res-ref_res) > 1e-9:
        parser.error('source and reference resolutions differ')
    walls = lambda g, r, o: cells_to_world(*np.nonzero(g == OCCUPIED), g.shape, r, o)
    src, dst = walls(grid, res, origin), walls(ref, ref_res, ref_origin)
    if args.yaw is None:
        yaw, tx, ty, d = estimate(src, dst)
    else:
        if args.tx is None or args.ty is None:
            parser.error('--yaw needs --tx and --ty')
        yaw, tx, ty, d = math.radians(args.yaw), args.tx, args.ty, None
    t = np.array([tx, ty])
    out, out_origin, (pad_lo, pad_hi) = resample(grid, res, origin, yaw, t, ref_res, ref_origin, ref.shape)
    output = Path(args.output)
    Image.fromarray(out).save(output.with_suffix('.pgm'))
    meta = dict(ref_meta, image=output.with_suffix('.pgm').name, origin=[round(float(out_origin[0]), 4), round(float(out_origin[1]), 4), 0])
    output.write_text(yaml.safe_dump(meta, sort_keys=False, default_flow_style=None))
    print(f'transform (source map -> reference map): yaw {math.degrees(yaw):.3f} deg, t ({tx:.3f}, {ty:.3f}) m')
    if d is not None:
        print('source walls within 0.10/0.15/0.30 m of a reference wall: '
              f'{(d < .10).mean():.1%} / {(d < .15).mean():.1%} / {(d < .30).mean():.1%}')
    print(f'{output}: {out.shape[1]}x{out.shape[0]} cells, origin {meta["origin"]}')
    if args.annotations:
        image = Image.open(args.annotations[0]).convert('RGBA')
        if image.size != (ref.shape[1], ref.shape[0]):
            parser.error('annotation image must match the reference map size')
        canvas = Image.new('RGBA', (out.shape[1], out.shape[0]), (UNKNOWN, UNKNOWN, UNKNOWN, 255))
        canvas.paste(image, (int(pad_lo[0]), int(pad_hi[1])))
        canvas.save(args.annotations[1])
        print(f'{args.annotations[1]}: padded left {pad_lo[0]}, top {pad_hi[1]} cells')


if __name__ == '__main__':
    main()
