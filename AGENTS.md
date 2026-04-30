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
- In this workspace, TRG is used as a global planner. It is not the path follower or local controller.
- The workspace-level RoboCup RViz config is
  `/home/bichy/robocup_ws/bringup/rviz/robocup_navigation.rviz`. Prefer it for
  SuperOdom/TRG/follower integration. Package-local RViz files are only for
  upstream demos or narrow TRG-only debugging.

## Editing rules

- Prefer runtime-configurable parameters and launch arguments over hardcoded map names or absolute scene paths.
- Keep the C++ core usable outside ROS.
- Keep ROS 2 pipeline changes compatible with both static prior-map use and prior-map-plus-online-update use.
- Keep `trg_planner_wMap.py` compatible with explicit `rviz_config` and with the
  RoboCup workspace RViz fallback when a map-specific RViz file is absent.
- For prior-map-only simulations, TRG still needs an odometry or pose input for
  the start state. The RoboCup `run_trg_follower_sim.sh` path supplies fake
  `/laser_odometry` from the follower simulator and should not require
  `/laser_cloud_map` unless online update is explicitly enabled.
- Avoid new dependencies unless they are necessary for Humble compatibility.

## Verification

- Verify the core library still builds after planner changes.
- Verify the ROS 2 package still builds after launch or topic changes.
- For startup behavior, prefer smoke tests that confirm:
  - map config resolves correctly
  - the planner waits for pose when configured to do so
  - topic names match the integration contract
  - workspace RViz fallback resolves for `superodom_k_rail` when `rviz:=true`
