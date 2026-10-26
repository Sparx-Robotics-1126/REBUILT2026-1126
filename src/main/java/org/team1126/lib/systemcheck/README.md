# 1126-lib: System Check

The SparX 1126 robot self-test library. It answers one question in the pit: **"Is everything on the robot working?"**

It's adapted from the system-check code in [FRC 3061 Huskie Robotics' 3061-lib](https://github.com/HuskieRobotics/3061-lib) (MIT), which builds on FRC 3015 Ranger Robotics' AdvancedSubsystem. The swerve drive base and AdvantageKit parts were left out. It was rewritten for our GRRBase code, with no AdvantageKit dependency.

## What it checks

| Kind | When it runs | Moves the robot? | Catches |
|---|---|---|---|
| **Device self-checks** | Automatically, about once a second while **disabled**, and at the end of every system check | No | Unplugged or dead CAN devices, an unplugged CANivore, brownouts, reboots, overheating, CANcoder magnet problems, gyro problems, low battery |
| **System checks** | On demand, from the dashboard, with the robot **enabled in Test mode** | **Yes** | Motors that don't spin, a follower fighting its leader, swerve modules that can't reach their target angle or speed |

### Current system checks

| Subsystem | Check | Notes |
|---|---|---|
| Swerve | Drives forward, left, backward and right, then spins both ways. Checks every module reaches its target angle (±10°) and wheel speed. | **Robot must be on blocks.** Catches dead motors, unread encoders and binding. It **can't** catch a wrong CANcoder offset or backwards drive motor, so watch the wheels: in "forward" all four should point straight ahead and roll the same way. Registers 8 TalonFX, 4 CANcoders, the Pigeon2 and the LowerCAN CANivore |
| Shooter | Spins the flywheel at 3 V. The lead must reach ≥ 8 rot/s and the follower ≥ 80% of the lead. | |
| Feeder | Spins at 3 V, must reach ≥ 8 rot/s | |
| Intake | Spins the roller at 3 V, must reach ≥ 8 rot/s | The pivot is **not** moved because it has hard stops. Both pivot SparkFlexes are still watched for faults |
| Storage | Spins the roller at 3 V, must reach ≥ 300 RPM | |
| Hood | Homes gently against the bottom hard stop, moves to 1, 11, 20 and back to 1 rotor rotations (each must be reached within 0.5 rot in 2.5 s), then homes again and checks the hard stop still reads ~0 | See *Checking a position mechanism* below. Keep hands clear of the hood |
| Robot | Battery: warning below 12.3 V, error below 11.8 V. roboRIO CAN bus health | |

## Pit procedure

1. Put the robot **on blocks** with the wheels off the ground, and clear the area around the intake and shooter.
2. Open Elastic (or Shuffleboard) and find the **`SystemStatus`** table.
3. While the robot is still disabled, look at `SystemStatus/AllOK` and the alerts. Any red means a device is already reporting a problem.
4. Enable the robot in **Test** mode. The checks refuse to run in any other mode, and pressing a button then just adds a reminder warning.
5. Press **`SystemStatus/RunAllChecks`**. It runs every check in order and takes about 25 seconds. **Watch the swerve wheels** during the forward phase: all four should point straight ahead and roll the same way (the code can't detect a bad offset or a backwards motor).
6. Every subsystem should show `Status = OK` and `CheckRan = true`. If not, read `SystemStatus/<Subsystem>/Faults`, or the alerts. Each message says which device is affected and what to look at.
7. After fixing something, press **`SystemStatus/ClearAllFaults`** to clear the old faults, including sticky faults stored on the devices, and run the check again.

Safety: checks only run when the robot is enabled in **Test** mode and never when connected to the FMS, so a stray dashboard click can't move the robot during a match. A driver can stop a running check by disabling the robot or using a control that needs the same subsystem.

## Dashboard keys (NetworkTables `SmartDashboard/SystemStatus/...`)

| Key | Type | Meaning |
|---|---|---|
| `AllOK` | boolean | `true` if no subsystem has an error. Warnings are allowed |
| `RunAllChecks` | button | Runs every system check in order |
| `ClearAllFaults` | button | Clears all faults, including sticky faults on the devices |
| `<Subsystem>/Status` | string | `OK`, `WARNING` or `ERROR` |
| `<Subsystem>/SystemOK` | boolean | `false` only when there's an error |
| `<Subsystem>/CheckRan` | boolean | `true` once that subsystem's check has finished |
| `<Subsystem>/Faults` | string[] | Every current fault, e.g. `ERROR: [frontLeft turn] not responding on CAN (ID 4)...` |
| `<Subsystem>/SystemCheck` | button | Runs just that subsystem's check |

Every fault is also raised as a WPILib `Alert`, so it appears in Elastic's alerts widget and in AdvantageScope. The pit display could read `SystemStatus/*/Status` to show a green/yellow/red board.

## Adding the system check to a subsystem

```java
public final class Climber extends GRRSubsystem {
    public static final String SYSTEM_CHECK_NAME = "Climber";

    public Climber() {
        motor = new TalonFX(CLIMBER_MOTOR);
        // ... configure the motor first ...

        // 1. Watch the hardware for faults (always do this; nothing moves)
        FaultReporter.getInstance().registerHardware(SYSTEM_CHECK_NAME, "climber", motor);

        // 2. Optional: register a check that moves the mechanism
        FaultReporter.getInstance().registerSystemCheck(SYSTEM_CHECK_NAME, this::systemCheck);
    }

    public Command systemCheck() {
        // A simple "does it spin" check
        return SystemChecks.spinTalonFX(SYSTEM_CHECK_NAME, this, "climber", motor, 2.0, 5.0);
    }
}
```

For a custom check, build any command and call `FaultReporter.getInstance().addFault(SYSTEM_CHECK_NAME, "what went wrong")` when something is off. `SystemChecks.expectNear(...)` is a helper for "is this value close to that value".

```java
public Command systemCheck() {
    return Commands.sequence(
        goToPosition(0.5),
        Commands.waitSeconds(1.0),
        SystemChecks.expectNear(SYSTEM_CHECK_NAME, "[arm] position", this::getPosition, 0.5, 0.05, "rot"),
        goToPosition(0.0)
    );
}
```

Rules for checks:
- Pass a **factory** (`this::systemCheck`), not a `Command`. The same check is used by its own button and by `RunAllChecks`.
- Make the command **require the subsystem** so the default command (e.g. joystick drive) is paused during the check.
- Prefer **small open-loop voltages** (2–4 V) for "does it spin" checks. They test the wiring and motor without depending on PID tuning. They can't tell if a motor is configured backwards, because inversion flips the command and the measurement together.
- **Don't drive into hard stops at full power.** Only move position mechanisms to safe, known positions, or home them gently with a current limit (like the hood does).

Supported devices for `registerHardware`: `TalonFX`, `CANcoder`, `Pigeon2`, `SparkMax` / `SparkFlex` (any `SparkBase`), Redux `Canandgyro` / `Canandmag`, a `CANBus`, or any custom `SelfChecking`. For GRRBase objects whose type is only known at runtime (e.g. `module.moveMotor.getAPI()`), use `registerDevice(...)`.

## Checking a position mechanism (hood, arm, pivot, elevator)

A "does it spin" check doesn't fit a mechanism that moves to a position and has hard stops. Instead, check that it **gets where it's told to go, in a reasonable time**, using the subsystem's own position command so you're testing the same code the robot uses in a match. The hood check (`Hood.systemCheck()`) is the example:

1. **Home first.** The hood drives gently down (velocity control with a 10 A torque limit) until it stalls against the bottom hard stop, then sets the encoder to 0. If it never stalls within 4 s, that's an error.
2. **Move to a few positions** inside the soft limits (low, middle, high, back to low) with `SystemChecks.moveToPosition(...)`. It runs the hood's normal `goTo()` command and waits until the position is within tolerance. If it isn't there by the timeout, that's an error: binding, a loose belt, a dead motor, or something in the way.
3. **Home again and compare.** After all that movement the hard stop should still read about 0. If it reads, say, 1.3 rotations, the encoder lost its place, which usually means a slipping belt or gear or skipped teeth. That's reported as a warning.

To add the same kind of check to another position mechanism:

```java
public Command systemCheck() {
    return Commands.sequence(
        SystemChecks.moveToPosition(SYSTEM_CHECK_NAME, "arm", goTo(() -> 0.25), this::getPosition, 0.25, 0.02, 2.0, "rot"),
        SystemChecks.moveToPosition(SYSTEM_CHECK_NAME, "arm", goTo(() -> 0.05), this::getPosition, 0.05, 0.02, 2.0, "rot")
    );
}
```

Pick targets well inside the travel, a tolerance a bit bigger than the mechanism normally settles to, and a timeout about twice as long as a normal move takes. If a healthy mechanism fails, loosen those numbers, don't remove the check.

One limit: if the motor is completely dead, homing "finds" the hard stop right away, because nothing is moving. The moves after it will still fail, so the check still reports a problem.

## Files

```
lib/systemcheck/
  FaultReporter.java          - collects faults, polls devices, publishes to the dashboard, runs checks
  SystemChecks.java           - ready-made checks: spinTalonFX, spinSpark, spin, moveToPosition, expectNear
  SubsystemFault.java         - one fault (description + warning/error)
  SystemStatus.java           - OK / WARNING / ERROR
  selfcheck/
    SelfChecking.java         - interface every device check implements
    PhoenixSelfChecking.java  - shared CTRE logic (connection + fault signals)
    SelfCheckingTalonFX.java, SelfCheckingCANcoder.java, SelfCheckingPigeon2.java
    SelfCheckingSpark.java    - SparkMax / SparkFlex
    SelfCheckingCanandgyro.java, SelfCheckingCanandmag.java   - Redux
    SelfCheckingCANBus.java   - CAN bus / CANivore health
    SelfCheckingBattery.java  - battery voltage
```

The only change outside this folder and the subsystems is a small `getIMUAPI()` getter added to `lib/swerve/SwerveAPI.java`, so the gyro can be registered.

## First time on the real robot: verify it

These are worth doing once, so the team trusts the results:

- [ ] With everything plugged in and the robot disabled, `AllOK` is `true`. Look at any warnings. Sticky faults from earlier sessions are normal; press `ClearAllFaults`.
- [ ] **Unplug one TalonFX's CAN** (or pull its breaker). Within about 2 seconds you should get `not responding on CAN`. Plug it back in and the fault clears by itself.
- [ ] **Do the same with the storage SPARK MAX.** REV doesn't offer a direct "is connected" call, so this one reads the bus voltage and checks REVLib's error code. If it doesn't show up, or a plugged-in SPARK shows as not responding, tell a programmer.
- [ ] Unplug the CANivore's USB. `LowerCAN bus` should report `CAN bus not available`.
- [ ] Run `RunAllChecks` on blocks. Everything should pass. If a spin check fails on a healthy mechanism, the minimum speed (8 rot/s, 300 RPM) or the 3 V test voltage in that subsystem's `systemCheck()` may need adjusting.
- [ ] Swerve: if the check fails on a good module, look at `CHECK_ANGLE_TOLERANCE_DEG` / `CHECK_SPEED_TOLERANCE` at the bottom of `Swerve.java`.

## Notes and limits

- Devices are only polled while **disabled**, so matches aren't slowed down. Anything that happened during a match shows up afterward as a `[STICKY]` warning.
- Polling is turned off in **simulation**, where device faults don't mean anything.
- Phoenix fault checks read signals the devices already broadcast, so they add no CAN traffic. If someone calls `optimizeBusUtilization(0)` on a device, the fault signals are turned back on at 4 Hz.

## Credits and license

The design is adapted from FRC Team 3061 Huskie Robotics' 3061-lib (`frc.lib.team3015.subsystem.FaultReporter`), which is derived from FRC Team 3015 Ranger Robotics' AdvancedSubsystem.

```
MIT License

Copyright (c) 2022-2025 Huskie Robotics

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```
