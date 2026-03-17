# CLAUDE.md

## Project Overview

This repository contains an FRC robot codebase written in Java using WPILib.
The robot architecture follows an IO abstraction pattern and uses AdvantageKit for logging and replay-based simulation.

The codebase is organized around subsystems and shared hardware interfaces. Simulation utilities (such as projectile simulation for the shooter) are used for analysis and visualization through AdvantageScope.

Primary goals of this project:

* Maintain deterministic robot behavior
* Support replay simulation using logged data
* Separate hardware logic from robot logic
* Enable offline analysis and physics simulation

---

# Architecture

## Subsystem Pattern

Each subsystem follows this structure:

Subsystem
IO Interface
IOReal (hardware implementation)
IOSim (simulation implementation)
InputsAutoLogged (AdvantageKit generated)

Example:

```
shooter/
    Shooter.java
    ShooterIO.java
    ShooterIOReal.java
    ShooterIOSim.java
    ShooterIOInputsAutoLogged.java
```

Subsystems update hardware inputs every loop:

```
io.updateInputs(inputs);
Logger.processInputs("SubsystemName", inputs);
```

Robot logic must read data from the `inputs` object rather than directly from hardware.

---

# AdvantageKit Usage

AdvantageKit is used for:

* telemetry logging
* log replay simulation
* AdvantageScope visualization

Important rules:

1. Hardware data must come from `InputsAutoLogged` objects.
2. Do NOT instantiate new inputs objects outside the subsystem.
3. Logged values should only be read after `Logger.processInputs()` is called.
4. Never read logged values during static initialization.

Incorrect pattern:

```
private static final double velocity = inputs.velocity;
```

Correct pattern:

```
double velocity = inputs.velocity;
```

inside methods that run during the robot loop.

---

# Simulation

Projectile simulation for the shooter is implemented in:

```
FuelSimulator.java
SimulatedFuel.java
```

The simulator tracks active shots and updates their physics each robot loop.

Responsibilities of the simulator:

* compute launch velocity
* integrate projectile motion
* detect hub collisions
* log projectile poses for visualization

Logged outputs:

```
FuelSimulator/ActiveFuel
FuelSimulator/ShotsMade
FuelSimulator/ShotsMissed
```

These values are displayed in AdvantageScope.

---

# Shooter Physics Model

Launch velocity depends on:

* hood pitch angle
* flywheel surface velocity
* robot chassis velocity
* turret orientation

Basic projectile equations:

```
vx = v * cos(hoodPitch)
vz = v * sin(hoodPitch)
```

Gravity is applied during simulation updates.

Robot velocity may be added to the projectile velocity when firing.

---

# Logging Conventions

All logging uses AdvantageKit:

```
Logger.recordOutput(key, value);
```

Subsystem inputs should be logged using:

```
Logger.processInputs("SubsystemName", inputs);
```

Naming convention:

```
Subsystem/Variable
FuelSimulator/ActiveFuel
Shooter/FlywheelVelocity
Turret/Angle
```

---

# Code Guidelines

General rules:

* Avoid static state unless required.
* Avoid hardware access outside IO classes.
* Physics calculations should live in simulation classes.
* Subsystems should contain only robot logic.

Preferred patterns:

```
Subsystem -> Inputs -> Simulator
Subsystem -> Commands
```

Avoid:

```
Simulator creating its own hardware inputs
Direct hardware calls in subsystems
```

---

# Coordinate System

The code uses WPILib geometry classes:

* `Pose2d`
* `Translation2d`
* `Translation3d`
* `Rotation2d`

Units:

* meters
* r
