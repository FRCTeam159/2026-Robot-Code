## Purpose
Short, actionable guidance for AI coding agents working in this FRC Java robot repo (WPILib + GradleRIO).

## Quick architecture overview
- Command-based robot (edu.wpi.first.wpilibj2.command). Key entry points:
  - `src/main/java/frc/robot/Robot.java` — WPILib TimedRobot lifecycle.
  - `src/main/java/frc/robot/RobotContainer.java` — wiring of subsystems, default commands, and SmartDashboard chooser.
- Drivetrain is a swerve implementation formed from:
  - `src/main/java/frc/robot/subsystems/Drivetrain.java` — high-level kinematics, odometry, field-oriented toggle, and `drive(x,y,rot,fieldRelative)`.
  - `src/main/java/frc/robot/subsystems/SwerveModule.java` — per-wheel motor/encoder control and PID-based turning.
- Low-level hardware wrappers live under `src/main/java/frc/robot/objects/` (e.g., `Motor`, `Encoder`, `DriveGyro`). Inspect these before changing motor/encoder behavior.

## Important repo conventions & patterns
- Default-command pattern: subsystems set default commands in `RobotContainer` (example: `m_drivetrain.setDefaultCommand(new DriveWithGamepad(...))`).
- SmartDashboard is used for live tuning and toggles (examples: "Power Value", "Field Oriented", and SendableChooser for autos).
- Robot constants (CAN IDs, module offsets) are centralized in `src/main/java/frc/robot/Constants.java`. When changing CAN IDs or offsets, update this file and the `Drivetrain.setOffsets(...)` usage.
- Path planning assets and deployable static files live in `src/main/deploy/pathplanner` and are copied to the roboRIO on deploy (see `build.gradle` deploy configuration).

## Build / deploy / developer workflows
- Build with the Gradle wrapper from the repo root (Windows PowerShell):
  - `.
\gradlew build`  (builds the project)
  - `.
\gradlew deploy` (uses GradleRIO deploy targets configured in `build.gradle` to send artifacts and `src/main/deploy` files to the RoboRIO)
  - If unsure which tasks exist, run `.
\gradlew tasks` to list available Gradle tasks.
- Unit tests (JUnit 5) are configured but this repo currently has no tests folder; use `.
\gradlew test` if you add tests.

## Integration points & vendor deps
- Vendor dependencies are listed in `vendordeps/` (examples: `REVLib.json`, `Phoenix6.json`) and referenced via `build.gradle`'s vendor configuration—check `build.gradle` before adding new vendor libs.
- WPILib features (simulation, driverstation GUI) are enabled in `build.gradle` (look for `wpi.sim` and `wpi.java` blocks).

## What an AI should check before editing
1. Read the low-level wrapper in `frc.robot.objects` before changing motor/encoder interfaces. These are single points of hardware abstraction.
2. Any changes to module geometry, offsets, or gearing should update `Constants.java` and `src/main/deploy/pathplanner/settings.json` if pathplanner parameters must match.
3. When changing default commands or command bindings, update `RobotContainer.robotInit()` / constructor where defaults and chooser entries are registered.

## Useful examples (where to make common changes)
- Tune swerve PID: edit `SwerveModule` PID gains and then test with `DriveWithGamepad` which calls `Drivetrain.drive(...)`.
- Change CAN ids/offsets: edit `Constants.java` and call `Drivetrain.init()` (already called from `RobotContainer.robotInit()`).
- Add a new auto path: add files under `src/main/deploy/pathplanner/autos` and register chooser entries in `RobotContainer`.

## Final notes
- Keep edits small and compile often (`.
\gradlew build`). The codebase follows typical WPILib command-based layout but uses custom hardware wrappers—always inspect `frc.robot.objects` first.
- If something feels missing or unclear, tell me which area (build, deploy, swerve math, hardware wrappers) and I will expand this guidance.
