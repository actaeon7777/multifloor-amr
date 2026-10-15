# TIM relocalization_toolbox ROS 2 port

Upstream: https://github.com/tsdworks/relocalization_toolbox

Pinned revision: `78fddd92cf695a2d6b230705f96d1ef04fa9e825` (retrieved 2026-10-04).
Original author: ZHANG Muhua, Southwest Jiaotong University.
Paper: https://arxiv.org/abs/2511.01219

The upstream GPL-3.0 license and original source attribution headers are retained.
This package ports the actual RRT sampling, SMAD ranking, TAM likelihood/consistency
scoring, batched angular search, and PCL ICP code to native ROS 2 messages. It is
not a claim of an unmodified reproduction of the paper's reported benchmarks.

Integration changes:

- A request/response service receives a known occupancy map and stationary scan;
  returns LASER poses in map coordinates with original scan timestamps. There are
  no motion, TF, or initialpose publishers. AMCL owns map-to-odom and the startup
  supervisor owns validation and laser-to-base extrinsics.
- Normalize the map's translated/rotated origin for the axis-aligned upstream
  core and compose it back onto output poses. Reject non-planar map origins.
- Seed an RRT forest in every feasible connected component instead of (0,0).
  After RRT expansion, add sparse free-cell anchors to fill coverage gaps. Every
  feasible cell is within anchor_spacing of an anchor. Clearance concerns the
  laser proposal only; downstream robot-footprint validation remains required.
- Multiple requested candidates disable the upstream early-exit behavior. NMS
  retains geometrically distinct candidates. The single-best orientation per
  anchor and top candidates per batch remain finite heuristic approximations;
  this is not exhaustive proof of global uniqueness.
- Use monotonic wall-clock deadline checkpoints and bounded map/scan/ICP limits.
  Expired searches discard partial results. The deadline is cooperative: one
  in-progress PCL ICP / distance-transform operation can finish after the budget.
  The startup supervisor must reject late responses using its own wall deadline.
- Sanitize finite rays, respect actual LaserScan angular metadata, evenly stride
  to at most max_beams, and reject insufficient usable returns. The default
  30 m range cap is algorithm tuning, not a statement of S2 hardware range.
- Fix map-cache key/ratio, uninitialized preprocessing fields, out-of-map endpoint
  clamping, selected-subset variance, sparse-scan voxel size, and nonfinite ICP
  result handling; project ICP rotation to SE(2) before TAM evaluation. Keep IEEE NaN/Inf behavior (no fast-math).
- Fixed random seed and bounded OpenMP thread count support repeatable trials; scheduling and deadlines can still affect candidate order.
  ROS 1 node/TF/debug publishers and unused map file loader are removed.

Defaults use 360 rays, 30 m, 0.40 m anchor spacing and 11.25 degree initial angular
search, followed by ICP. TAM scores are diagnostics, not calibrated confidence
probabilities. Similar corridors can yield multiple good matches or wrong matches;
independent rival and fresh-observation validation are required before navigation.
Algorithm parameters and `service_name` are read-only after startup: edit the YAML
configuration and restart the node to change them. ROS `use_sim_time` retains its
normal runtime behavior.

Local validation informed two departures from the first integration defaults:
30 m retains distinctive corridor ends in the saved 4th-floor map (20 m yielded
periodic rivals); 0.40 m anchor spacing preserves the correct basin for a backward
facing scanner near unknown map borders, where coarse 0.75 m anchors failed. These
are synthetic map-raycast observations on this PC, not Jetson or field benchmarks.

## Bounded dense refinement (2026-10-04)

`fine_enabled` keeps the global search at 360 beams, then tries at most five
returned candidates with up to 1080 original scan samples and map sampling ratio
1. The scan and map are still independently voxel-filtered at the original map
resolution; this is not sub-cell ground-truth sensing. Both the original and
refined pose are scored on exactly the same dense TAM observation. All retained
candidates use that metric before final sorting and NMS. If dense sampling has
fewer than 12 valid returns, all original poses and original scores are retained.

An ICP correction is accepted only with a strict dense score improvement (default
1e-6), a converged termination other than the iteration cap, finite residuals,
at least 70% point correspondences, sufficient planar geometry information, and
translation/yaw corrections no greater than 0.25 m / 0.10 rad. Local map point
normals form a normalized 2D information matrix; its minimum/maximum eigenvalue
ratio must be at least 0.002. This is a degeneracy heuristic, not a pose covariance
or proof of global uniqueness. Parallel-wall geometry triggers coarse fallback.

Dense map-distance preparation and baseline scoring use the overall request
budget. Subsequent fine ICP preparation/attempts have an additional cooperative
3-second budget (`fine_budget_sec`) inside the same overall 30-second limit.
PCL operations are checked between operations; this is not a hard real-time
3-second bound. Expired fine attempts retain coarse poses and consistently dense
scores; expiration of the total request budget still fails closed. Fine-stage
exceptions preserve coarse candidates. The supervisor's independent rival and
fresh-observation checks remain required.

`tim_fine` logs candidate index, accept/fallback reason, before/after dense TAM,
correction size, PCL termination enum, RMS residual, correspondence ratio and
planar geometry ratio. PCL termination codes are 0=not converged, 1=iteration
limit, 2=transform change, 3=absolute MSE, 4=relative MSE, 5=no correspondences,
6=failure after iteration limit. Scores are not probabilities.
