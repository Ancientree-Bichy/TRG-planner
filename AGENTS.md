# AGENTS.md

This repository copy is used inside the RoboCup workspace as the terrain-aware global planner.

## Integration context

- Primary integration target is ROS 2 Humble.
- This `AGENTS.md` is part of the TRG-planner git history. Keep it updated when
  ROS 2 launch, RViz, topic wiring, or RoboCup integration assumptions change.
- The default RoboCup wiring is:
  - odometry: `/laser_odometry`
  - observation cloud: `/laser_cloud_map`
  - goal pose: `/goal_pose`
  - prebuilt map visualization: `/trg/output/prebuilt_map`
  - output path: `/trg/output/path`
- TRG can optionally load a RoboCup allowed-area YAML through the map config
  `boundary` section. When enabled, graph samples, edges, start poses, and goals
  outside the polygon are rejected.
- Path search mode is selected in YAML with `trg.pathSearchMode`. Keep
  `native` as the default for compatibility; use `trg_astar` for the
  heading-aware C++ TRG-AStar search. In RoboCup configs, `trg_astar` also
  enables the migrated Path_Planing_QRC rectangular body/footprint cost through
  `trgAStar.footprintCostEnabled`.
- In this workspace, TRG is used as a global planner. It is not the path follower or local controller.
- The workspace-level RoboCup RViz config is
  `$ROBOCUP_WS/bringup/rviz/robocup_navigation.rviz`. Prefer it for
  SuperOdom/TRG/follower integration. Package-local RViz files are only for
  upstream demos or narrow TRG-only debugging.

## Editing rules

- Prefer runtime-configurable parameters and launch arguments over hardcoded map names or absolute scene paths.
- Keep the C++ core usable outside ROS.
- Keep ROS 2 pipeline changes compatible with both static prior-map use and prior-map-plus-online-update use.
- Keep TRG source hyperparameters in `config/robocup_default.yaml`. Do not add
  one YAML per scene or map; pass prior-map PCDs through `map_path:=...` or a
  generated runtime `mapConfigPath`.
- Keep `trg_planner_wMap.py` compatible with explicit `rviz_config` and with the
  RoboCup workspace RViz fallback when a config-specific RViz file is absent.
- For TRG-only RViz debugging, `trg_planner_wMap.py` starts the RViz initial-pose
  bridge by default. RViz `2D Pose Estimate` publishes `/initialpose`, the bridge
  converts it to `/laser_odometry`, and RViz `2D Goal Pose` publishes
  `/goal_pose` directly to TRG.
- For prior-map-only simulations, TRG still needs an odometry or pose input for
  the start state. The RoboCup `run_trg_follower_sim.sh` path supplies fake
  `/laser_odometry` from the follower simulator and should not require
  `/laser_cloud_map` unless online update is explicitly enabled.
- For repeatable RoboCup prior-map tests, prefer `trg.deterministicSampling:
  true` plus a fixed `trg.randomSeed`. Leave `randomSeed: -1` only when
  intentionally testing stochastic sampling behavior.
- Keep allowed-area logic independent from the terrain map: the prior map
  describes geometry/risk, while the boundary YAML describes competition
  legality.
- Avoid new dependencies unless they are necessary for Humble compatibility.

## Verification

- Verify the core library still builds after planner changes.
- Verify the ROS 2 package still builds after launch or topic changes.
- For startup behavior, prefer smoke tests that confirm:
  - map config resolves correctly
  - the planner waits for pose when configured to do so
  - topic names match the integration contract
  - workspace RViz fallback resolves for `config:=robocup_default` when `rviz:=true`
