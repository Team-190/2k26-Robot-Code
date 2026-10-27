// Copyright 2021-2024 FRC 6328
package edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.team190.gompeilib.core.GompeiLib;
import edu.wpi.team190.gompeilib.core.io.components.inertial.GyroIO;
import edu.wpi.team190.gompeilib.core.io.components.inertial.GyroIOInputsAutoLogged;
import edu.wpi.team190.gompeilib.core.io.components.inertial.GyroIOPigeon2;
import edu.wpi.team190.gompeilib.core.logging.Trace;
import edu.wpi.team190.gompeilib.core.utility.control.Gains;
import edu.wpi.team190.gompeilib.core.utility.phoenix.PhoenixOdometryThread;
import java.util.List;
import java.util.Optional;
import java.util.Queue;
import java.util.function.Consumer;
import java.util.function.Supplier;
import java.util.stream.IntStream;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.RobotState;
import org.wpilib.driverstation.internal.DriverStationBackend;
import org.wpilib.math.controller.PIDController;
import org.wpilib.math.filter.LinearFilter;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Twist2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.kinematics.SwerveDriveKinematics;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.kinematics.SwerveModuleVelocity;
import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.math.linalg.Vector;
import org.wpilib.math.numbers.N2;
import org.wpilib.math.util.Units;

public class SwerveDrive extends SubsystemBase {
  private final SwerveDriveConstants driveConstants;

  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs;
  private final SwerveModule[] modules;

  private final LinearFilter xFilter;
  private final LinearFilter yFilter;
  private double filteredX;
  private double filteredY;

  private final SwerveDriveKinematics kinematics;
  @Getter private Rotation2d rawGyroRotation;
  private final SwerveModulePosition[] lastModulePositions;
  @Getter private ChassisVelocities measuredChassisVelocities;

  private final Supplier<Pose2d> robotPoseSupplier;

  private final PIDController autoXController;
  private final PIDController autoYController;
  private final PIDController autoHeadingController;

  private final Optional<Queue<Double>> yawTimestampQueue;
  private final Optional<Queue<Double>> yawPositionQueue;

  private RobotConfig config;

  public SwerveDrive(
      SwerveDriveConstants driveConstants,
      GyroIO gyroIO,
      SwerveModuleIO flModuleIO,
      SwerveModuleIO frModuleIO,
      SwerveModuleIO blModuleIO,
      SwerveModuleIO brModuleIO,
      Supplier<Pose2d> robotPoseSupplier,
      Consumer<Pose2d> resetPoseConsumer) {
    this.driveConstants = driveConstants;
    this.gyroIO = gyroIO;
    gyroInputs = new GyroIOInputsAutoLogged();
    modules = new SwerveModule[4]; // FL, FR, BL, BR
    modules[0] = new SwerveModule(driveConstants, flModuleIO, 0);
    modules[1] = new SwerveModule(driveConstants, frModuleIO, 1);
    modules[2] = new SwerveModule(driveConstants, blModuleIO, 2);
    modules[3] = new SwerveModule(driveConstants, brModuleIO, 3);

    xFilter = LinearFilter.movingAverage(10);
    yFilter = LinearFilter.movingAverage(10);
    filteredX = 0;
    filteredY = 0;

    kinematics = this.driveConstants.driveConfig.kinematics();
    rawGyroRotation = new Rotation2d();
    lastModulePositions = // For delta tracking
        new SwerveModulePosition[] {
          new SwerveModulePosition(),
          new SwerveModulePosition(),
          new SwerveModulePosition(),
          new SwerveModulePosition()
        };

    this.robotPoseSupplier = robotPoseSupplier;

    boolean isGryoHighFrequency = gyroIO instanceof GyroIOPigeon2;

    if (isGryoHighFrequency) {
      // Start threads (no-op if no signals have been created)
      PhoenixOdometryThread.getInstance(driveConstants).start();
      this.yawTimestampQueue =
          Optional.of(PhoenixOdometryThread.getInstance(driveConstants).makeTimestampQueue());
      this.yawPositionQueue =
          Optional.of(
              PhoenixOdometryThread.getInstance(driveConstants).registerSignal(gyroIO.getYaw()));
    } else {
      this.yawTimestampQueue = Optional.empty();
      this.yawPositionQueue = Optional.empty();
    }

    autoHeadingController =
        new PIDController(
            driveConstants.autoRotationGains.kP().get(),
            0.0,
            driveConstants.autoRotationGains.kD().get(),
            GompeiLib.getLoopPeriod());
    autoXController =
        new PIDController(
            driveConstants.autoTranslationGains.kP().get(),
            0.0,
            driveConstants.autoTranslationGains.kD().get(),
            GompeiLib.getLoopPeriod());
    autoYController =
        new PIDController(
            driveConstants.autoTranslationGains.kP().get(),
            0.0,
            driveConstants.autoTranslationGains.kD().get(),
            GompeiLib.getLoopPeriod());

    autoHeadingController.enableContinuousInput(-Math.PI, Math.PI);
    autoHeadingController.setTolerance(Units.degreesToRadians(1.0));

    Gains driveGains = driveConstants.driveGains;
    Gains turnGains = driveConstants.turnGains;
    Runnable applyModuleGains =
        () -> {
          setPIDGains(driveGains.getKP(), driveGains.getKD(), turnGains.getKP(), turnGains.getKD());
          setFFGains(driveGains.getKS(), driveGains.getKV());
        };
    driveGains.onChange(gains -> applyModuleGains.run());
    turnGains.onChange(gains -> applyModuleGains.run());
    driveConstants.autoTranslationGains.onChange(
        gains -> setAutoControllers(gains, driveConstants.autoRotationGains));
    driveConstants.autoRotationGains.onChange(
        gains -> setAutoControllers(driveConstants.autoTranslationGains, gains));

    measuredChassisVelocities = new ChassisVelocities();

    try {
      config = RobotConfig.fromGUISettings();
      AutoBuilder.configure(
          this.robotPoseSupplier,
          resetPoseConsumer, // resetPose
          () -> getChassisVelocities(), // get robotRelativeSpeeds
          (speeds, feedforwards) -> {
            // Retained from Choreo; switch to runVelocityTorque(speeds, forces) if needed.
            @SuppressWarnings("unused")
            List<Vector<N2>> forces =
                IntStream.range(0, 4)
                    .mapToObj(
                        i ->
                            VecBuilder.fill(
                                feedforwards.robotRelativeForcesXNewtons()[i],
                                feedforwards.robotRelativeForcesYNewtons()[i]))
                    .toList();

            runVelocity(speeds);
          },
          new PPHolonomicDriveController(
              new PIDConstants(
                  driveConstants.autoTranslationGains.kP().getAsDouble(),
                  driveConstants.autoTranslationGains.kI().getAsDouble(),
                  driveConstants.autoTranslationGains.kD().getAsDouble()),
              new PIDConstants(
                  driveConstants.autoRotationGains.kP().getAsDouble(),
                  driveConstants.autoRotationGains.kI().getAsDouble(),
                  driveConstants.autoRotationGains.kD().getAsDouble())),
          config,
          () -> {
            var alliance = DriverStationBackend.getAlliance();
            if (alliance.isPresent()) {
              return alliance.get() == Alliance.RED;
            }
            return false;
          },
          this);
    } catch (Exception e) {
      throw new RuntimeException("Failed to load PathPlanner robot config", e);
    }
  }

  @Trace
  public void periodic() {
    driveConstants.reentrantLock.lock();

    if (yawTimestampQueue.isPresent() && yawPositionQueue.isPresent()) {
      gyroIO.updateInputs(gyroInputs, yawTimestampQueue.get(), yawPositionQueue.get());
      yawTimestampQueue.get().clear();
      yawPositionQueue.get().clear();
    } else {
      gyroIO.updateInputs(gyroInputs);
    }

    for (int i = 0; i < 4; i++) {
      modules[i].updateInputs();
    }

    driveConstants.reentrantLock.unlock();

    Logger.processInputs("Drive/Gyro", gyroInputs);

    for (int i = 0; i < 4; i++) {
      modules[i].periodic();
    }

    // Stop moving when disabled
    if (RobotState.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }

      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleVelocity[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleVelocity[] {});
    }

    Logger.recordOutput("SwerveStates/Measured", getModuleStates());
    Logger.recordOutput("SwerveChassisVelocities/Measured", measuredChassisVelocities);

    // Update odometry
    double[] sampleTimestamps =
        modules[0].getOdometryTimestamps(); // All signals are sampled together
    int sampleCount = sampleTimestamps.length;
    for (int i = 0; i < sampleCount; i++) {
      // Read wheel positions and deltas from each module
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
        moduleDeltas[moduleIndex] =
            new SwerveModulePosition(
                modulePositions[moduleIndex].distance - lastModulePositions[moduleIndex].distance,
                modulePositions[moduleIndex].angle);
        lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
      }

      // Update gyro angle
      if (gyroInputs.connected) {
        // Use the real gyro angle
        rawGyroRotation = gyroInputs.odometryYawPositions[i];
      } else {
        // Use the angle delta from the kinematics and module deltas
        Twist2d twist = kinematics.toTwist2d(moduleDeltas);
        rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      }

      ChassisVelocities chassisSpeeds = kinematics.toChassisVelocities(getModuleStates());
      measuredChassisVelocities = chassisSpeeds;
      Translation2d rawFieldRelativeVelocity =
          new Translation2d(chassisSpeeds.vx, chassisSpeeds.vy).rotateBy(getRawGyroRotation());

      filteredX = xFilter.calculate(rawFieldRelativeVelocity.getX());
      filteredY = yFilter.calculate(rawFieldRelativeVelocity.getY());
    }
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  @Trace
  public void runVelocity(ChassisVelocities speeds) {
    // Calculate module setpoints
    ChassisVelocities optimizedSpeeds = speeds.discretize(GompeiLib.getLoopPeriod());
    SwerveModuleVelocity[] setpointStates = kinematics.toSwerveModuleVelocities(optimizedSpeeds);
    SwerveDriveKinematics.desaturateWheelVelocities(
        setpointStates, driveConstants.driveConfig.maxLinearVelocityMetersPerSecond());

    // Log unoptimized setpoints and setpoint speeds
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveChassisVelocities/Setpoints", speeds);

    // Send setpoints to modules
    for (int i = 0; i < 4; i++) {
      setpointStates[i] = modules[i].runSetpoint(setpointStates[i], new SwerveModuleVelocity());
    }

    // Log optimized setpoints
    Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
  }

  /**
   * Runs the drive at the desired velocity and torque.
   *
   * @param speeds Speeds in meters/sec
   */
  @Trace
  public void runVelocityTorque(ChassisVelocities speeds, List<Vector<N2>> forces) {
    if (forces.size() != 4) {
      throw new IllegalArgumentException("Forces array must have 4 elements");
    }
    // Calculate module setpoints
    ChassisVelocities optimizedSpeeds = speeds.discretize(GompeiLib.getLoopPeriod());
    SwerveModuleVelocity[] setpointStates = kinematics.toSwerveModuleVelocities(optimizedSpeeds);
    SwerveModuleVelocity[] setpointTorques = new SwerveModuleVelocity[4];
    SwerveDriveKinematics.desaturateWheelVelocities(
        setpointStates, driveConstants.driveConfig.maxLinearVelocityMetersPerSecond());

    // Send setpoints to modules
    for (int i = 0; i < 4; i++) {
      Vector<N2> wheelDirection =
          VecBuilder.fill(setpointStates[i].angle.getCos(), setpointStates[i].angle.getSin());
      setpointTorques[i] =
          new SwerveModuleVelocity(
              forces.get(i).dot(wheelDirection)
                  * driveConstants.driveConfig.frontLeft().DriveMotorGearRatio,
              setpointStates[i].angle);

      setpointStates[i] = setpointStates[i].optimize(modules[i].getAngle());
      setpointTorques[i] = setpointTorques[i].optimize(modules[i].getAngle());

      setpointStates[i] = modules[i].runSetpoint(setpointStates[i], setpointTorques[i]);
    }

    // Log optimized setpoints
    Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
    Logger.recordOutput("SwerveStates/TorquesOptimized", setpointTorques);
  }

  /** Runs the drive in a straight line with the specified drive current. */
  @Trace
  public void runCharacterization(double amps) {
    for (int i = 0; i < 4; i++) {
      modules[i].runCharacterization(amps);
    }
  }

  /** Stops the drive. */
  @Trace
  public void stop() {
    runVelocity(new ChassisVelocities());
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  @Trace
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] =
          driveConstants.driveConfig.getModuleTranslations()[i].getAngle().orElse(Rotation2d.ZERO);
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  /** Returns the module states (turn angles and drive velocities) for all of the modules. */
  @Trace
  private SwerveModuleVelocity[] getModuleStates() {
    SwerveModuleVelocity[] states = new SwerveModuleVelocity[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  /** Returns the module positions (turn angles and drive positions) for all of the modules. */
  @Trace
  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getPosition();
    }
    return states;
  }

  /** Returns the measured chassis speeds of the robot. */
  @Trace
  private ChassisVelocities getChassisVelocities() {
    return kinematics.toChassisVelocities(getModuleStates());
  }

  /** Returns the position of each module in radians. */
  @Trace
  public double[] getWheelRadiusCharacterizationPositions() {
    double[] values = new double[4];
    for (int i = 0; i < 4; i++) {
      values[i] = modules[i].getWheelRadiusCharacterizationPosition();
    }
    return values;
  }

  /** Returns the average velocity of the modules in rotations/sec (Phoenix native units). */
  @Trace
  public double getFFCharacterizationVelocity() {
    double output = 0.0;
    for (int i = 0; i < 4; i++) {
      output += modules[i].getFFCharacterizationVelocity() / 4.0;
    }
    return output;
  }

  /** Returns the maximum linear speed in meters per sec. */
  @Trace
  public double getMaxLinearSpeedMetersPerSec() {
    return driveConstants.driveConfig.maxLinearVelocityMetersPerSecond();
  }

  /** Returns the maximum angular speed in radians per sec. */
  @Trace
  public double getMaxAngularSpeedRadPerSec() {
    return getMaxLinearSpeedMetersPerSec() / driveConstants.driveConfig.driveBaseRadius();
  }

  /** Returns the field relative velocity in X and Y. */
  @Trace
  public Translation2d getFieldRelativeVelocity() {
    return new Translation2d(filteredX, filteredY);
  }

  /** Returns the current yaw velocity */
  @Trace
  public double getYawVelocity() {
    return gyroInputs.yawVelocityRadPerSec;
  }

  /** Sets PID gains for modules */
  @Trace
  public void setPIDGains(double drive_Kp, double drive_Kd, double turn_Kp, double turn_Kd) {
    for (var module : modules) {
      module.setPID(drive_Kp, drive_Kd, turn_Kp, turn_Kd);
    }
  }

  /** Sets FF gains for modules */
  @Trace
  public void setFFGains(double kS, double kV) {
    for (var module : modules) {
      module.setFF(kS, kV);
    }
  }

  // TODO: restore choreoDrive(SwerveSample) once ChoreoLib ships a WPILib 2027 alpha build;
  // it has no compatible release yet, so the choreo dependency and this method are removed.

  public void setAutoControllers(Gains translationGains, Gains rotationGains) {
    autoXController.setPID(translationGains.kP().get(), 0.0, translationGains.kD().get());
    autoYController.setPID(translationGains.kP().get(), 0.0, translationGains.kD().get());
    autoHeadingController.setPID(rotationGains.kP().get(), 0.0, rotationGains.kD().get());
  }

  /**
   * Updates current limits.
   *
   * @param driveCurrentLimit The drive current limit.
   * @param turnCurrentLimit The turn current limit.
   */
  @Trace
  public void updateCurrentLimits(double driveCurrentLimit, double turnCurrentLimit) {
    for (SwerveModule s : modules) {
      s.updateCurrentLimits(driveCurrentLimit, turnCurrentLimit);
    }
  }
}
