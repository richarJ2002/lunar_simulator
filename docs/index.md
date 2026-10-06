# Space Robotics Simulator (SRS)

> Code is truth — this site describes; the source and YAML on `main` win on conflict.

## Vision

`space_robotics_simulator` is the workspace for developing rover autonomy stacks,
starting with basic algorithms and building toward mission-specific systems.
ROS is used for quick development of qualified algorithms; lower-level
software and middleware are kept reusable so proven work ports quickly, with
custom middleware/firmware in practical application. The codebase aims toward
JSF-compliant R&D practice (proposed, not certified).

## What and where

A ROS 2 Jazzy + Gazebo Harmonic rover simulator. The repository root is the
colcon workspace: `src/` (nodes and libraries), `parameters/` (runtime
tuning), `launch/` (system launch), `environment/` (worlds), `config/`
(bridge wiring), `post_processing/` (run reports).

A physical system composes modules. Alpha is the current and only physical
system: an ExoMars-scale replica (Europe's first rover) used as the first
step to establish an architecture for low-level code that people can pick up
quickly. Its near-term goal is following simple commands. Current modules
(`localisation`, `control`, `common`) are basic first implementations and
will grow with new modules and mission-specific stacks.

## Start here

- [Quickstart](quickstart.md) — build, launch, and wait for READY.
- [Architecture](architecture.md) — how the system fits together.
