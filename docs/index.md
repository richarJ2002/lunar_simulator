# lunar_simulator

> Code is truth — this site describes; the source and YAML on `main` win on conflict.

## Vision

`lunar_simulator` is a proving ground for Mars-mission rover architectures:
autonomy stacks (localisation, control, supervision) developed and tested in
simulation first, on the Moon today and on Mars surfaces next.

## What and where

A ROS 2 Jazzy + Gazebo Harmonic rover simulator. The repository root is the
colcon workspace: `src/` (nodes and libraries), `parameters/` (runtime
tuning), `launch/` (system launch), `worlds/` (environments), `config/`
(bridge wiring), `post_processing/` (run reports).

Alpha is the first and currently only system: a single-process `AlphaNode`
rover driving on the `lunar_surface` world.

## Start here

- [Quickstart](quickstart.md) — build, launch, and wait for READY.
- [Architecture](architecture.md) — how the system fits together.
