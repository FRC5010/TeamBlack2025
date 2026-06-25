# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

FRC team 5010's 2026 robot code (WPILib, Java 17, command-based). It is built on a reusable in-house framework (`org.frc5010.common`, "FRC5010Lib") that is **JSON-configuration-driven**: robots, subsystems, drivetrains, controllers, and cameras are described by deploy-time JSON rather than hardcoded, so one codebase runs multiple physical robots. AdvantageKit (`org.littletonrobotics.junction`) provides logging/replay; the `Robot` class extends `LoggedRobot`.

## Build / run commands

The build runs Spotless (`compileJava.dependsOn(spotlessApply)`), and Spotless's `googleJavaFormat()` crashes on newer JDKs in some environments (`NoSuchMethodError ... JCImport.getQualifiedIdentifier`). When that happens, exclude the Spotless tasks:

```bash
./gradlew compileJava -x spotlessJava -x spotlessJavaCheck   # compile only
./gradlew build       -x spotlessJava -x spotlessJavaCheck   # full build
./gradlew test        -x spotlessJava -x spotlessJavaCheck   # JUnit 5 tests (src/test/java)
./gradlew test --tests 'org.frc5010.common.arch.GenericRobotTest'   # single test class
./gradlew simulateJavaRelease                                 # desktop/maple-sim simulation
./gradlew deploy                                              # deploy to the roboRIO
```

Spotless config: `googleJavaFormat`, `removeUnusedImports`, 100-col limit. Run `./gradlew spotlessApply` locally before pushing where the toolchain works; otherwise match the style by hand (2-space indent, ≤100 col code lines — comments/Javadoc are left alone by the formatter).

Chromium/Playwright are preinstalled in the cloud env; do not run `playwright install`.

## How a robot is selected and built (read these together)

1. `frc.robot.Main` → `frc.robot.Robot` (`LoggedRobot`) → `RobotContainer` → `RobotsParser`.
2. `RobotsParser` reads `src/main/deploy/robots.json`, which maps a robot **name → { id (roboRIO MAC), robotClass, simulate, competition }**. The active robot is chosen by matching the RIO's MAC, or in simulation by the `simulate: true` flag. The matching entry's `robotClass` (e.g. `frc.robot.blackteam.BlackRobot`) is instantiated via reflection with its **deploy directory name** (e.g. `buttercup_swerve`).
3. `GenericRobot(String directory)` constructs `RobotParser` + `SubsystemParser`, which read the JSON files under `src/main/deploy/<directory>/` and build the drivetrain, subsystems, cameras, and controllers. **Exceptions during this phase are caught in `GenericRobot`'s constructor, printed, and swallowed** — a failed parse leaves a half-built robot rather than a crash, so "the drivetrain didn't get created" usually means an exception was thrown and logged during config parsing.

So robot behavior is determined largely by `src/main/deploy/<robot>/*.json`, not by Java edits. The two configured robots share `BlackRobot` but have different deploy dirs (`black_robot`, `buttercup_swerve`).

## Code layout

- `src/main/java/org/frc5010/common/` — the framework library:
  - `arch/` — `GenericRobot`, `GenericSubsystem`, `GenericCommand`, NetworkTables value plumbing.
  - `config/` — JSON parsers (`RobotParser`, `SubsystemParser`, `RobotsParser`) and the `config/json/**` POJOs that map 1:1 to the deploy JSON. `UnitsParser` + `config/units/**` parse `{ "val", "uom" }` measurements (units accept aliases, e.g. `m/sec`, `kg*m^2`, `degrees`).
  - `drive/` — drivetrain abstractions. See the swerve section below.
  - `telemetry/` — `DisplayValuesHelper` + `Display*` types wrap NetworkTables entries; `makeConfig*` values are persistent and dashboard-editable (used for live tuning).
  - `sensors/`, `motors/`, `mechanisms/`, `subsystems/`, `vision/`, `auto/`, `commands/`.
- `src/main/java/frc/robot/` — WPILib entrypoint + robot-specific code; `blackteam/BlackRobot` is the concrete `GenericRobot` and its subsystems (flywheels, feeder).
- `src/main/deploy/<robot>/` — per-robot JSON config (robot.json, drivetrain, subsystems, controllers, cameras, pathplanner).
- `src/main/resources/schemas/` — JSON schemas for the config files (e.g. `akit_swerve_drivetrain.schema.json`); validate config changes against these.

## Swerve drivetrain architecture (the most complex area)

`GenericSwerveDrivetrain` (the WPILib `Subsystem`) wraps a `SwerveDriveFunctions` implementation chosen by `robot.json`'s `driveType`:

- `YAGSL_SWERVE_DRIVE` → `YAGSLSwerveDrivetrain` (the YAGSL/`swervelib` library). Configured by `deploy/<robot>/yagsl_swerve/**` JSON (modules, `pidfproperties.json`, `physicalproperties.json`, `swervedrive.json`).
- `AKIT_SWERVE_DRIVE` → `AkitSwerveDrive` (AdvantageKit-style IO-layer swerve). Configured by `deploy/<robot>/akit_swerve_drivetrain.json` (validated by the schema). `type` selects the module IO: `TalonFX`, `SparkTalon` (TalonFX drive + Spark steer), or `Spark` (NEO/SparkMax drive + steer).

Key AKit-swerve facts that are non-obvious and have caused real bugs:

- **The `Spark` IO ignores most of the JSON** — `ModuleIOSpark` reads CAN IDs, gear ratios, drive PID, current limits, and per-module `zeroRotation` from the hardcoded `drive/swerve/akit/DriveConstants.java`, **not** from `akit_swerve_drivetrain.json`. The JSON still drives kinematics/PathPlanner/sim. To change Spark hardware behavior, edit `DriveConstants`. (`SparkTalon`/`TalonFX` IOs do read the JSON via `AkitSwerveConfig`.)
- **Two odometry threads.** `ModuleIOSpark`/`ModuleIOSparkTalon` sample on `SparkOdometryThread`; `GyroIOPigeon2` (a CTRE device) samples on `TalonFXOdometryThread`. Both must be created for the gyro to work, and `AkitSwerveDrive.periodic` must tolerate the two queues having different lengths.
- **Steering runs closed-loop on the absolute encoder.** Canandmags are read via the Spark's absolute-encoder API; offsets rely on the encoder's internal zero (config offsets are `0`). `Module.runSetpoint` cosine-scales drive speed by the *measured* steering angle and `optimize()`s against it, so the module IO must populate `inputs.turnPosition` (not just `turnAbsolutePosition`) or strafe/odometry break. A module whose zero leaves it resting at ±180° sits on the wrap seam and will intermittently flip direction.
- Simulation skips the hardware IO entirely (uses `ModuleIOSim`/`GyroIOSim` + maple-sim) and never starts the odometry threads, so hardware-thread/encoder bugs do **not** reproduce in sim.

## Git / workflow

Active development branches are created per-task off the team's feature branches (e.g. `claude/...`). The default branch is protected — branch before committing. Do not push to a different branch without explicit permission. GitHub operations go through the `mcp__github__*` tools (scope: `frc5010/teamblack2025`); there is no `gh` CLI.
