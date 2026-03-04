# robot-2026: Arkelon

## Plans:
* Post duluth refactor is a debloat and large rewrite of the current codebase which will require PID tuning and makign sure low level things like gear ratio calculations work.
* Move away from YAMS on simple mechanisims like flywheels.

## Conventions
* Keep the tuner-swerve-project*.json files next to their corresponding java files
* Often you want to commit code just to be able to acess on another computer.
  If so, prefix the commit with `do not use`, `donotuse`, `nouse-` or something similar to that
* Always name generated drivetrain constants in this standard: `TunerConstants<Robot Name><Date as MMDD><Other>`
Eg. TunerConstantsArkelon0221Eagan
* Use `Command` objects as subsystem members sparingly, as they cannot be used in compositions. Read up on the [WPIlib docs](https://docs.wpilib.org/en/stable/docs/software/commandbased/index.html)
* In a similar vain, factory commands should be used whenever possible

## Current Controls
### Driver
  * Left Stick: Translation robot oriented (Works with gas pedal)
  * Right stick X: Rotation
  * Right Trigger: "Gas Pedal" (how fast does the robot go?)
  * Right Bumper: Toggle vision
  * Left Bumper: Reset odometry rotation
  * B: Lockpose & brake
  * Y: Autoalign to preset distance from hub
  * A: Autoalign rotation only to hub
  * D-Pad up: Idle subsytems save for drivetrain and intake pivot
### Codriver
  * D-Pad up: Idle subsytems save for drivetrain and intake pivot
  * A: Run flywheels, indexer, and hood to setpoint to shoot, based on distance from hub
  * Y: Run flywheels, indexer, and hood to setpoint to shoot, from static angle and speed
  * X / Right Bumper: Run intake roller
  * B / Left Bumper: Toggle intake deploy

