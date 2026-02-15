package frc.robot.commands.shared;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.team190.gompeilib.core.logging.Trace;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveDrive;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveDriveConstants;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveDriveConstants.AutoAlignNearConstants;
import frc.robot.FieldConstants;
import frc.robot.util.AllianceFlipUtil;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public final class DriveCommands {

  /**
   * A command that drives a SwerveDrive using joystick input.
   *
   * @param drive The SwerveDrive to control.
   * @param driveConstants The constants for the SwerveDrive.
   * @param xSupplier The supplier of the x-axis joystick input.
   * @param ySupplier The supplier of the y-axis joystick input.
   * @param omegaSupplier The supplier of the omega joystick input.
   * @param rotationSupplier The supplier of the rotation of the robot.
   * @param hijackX The supplier of whether to hijack (override) the x-axis velocity.
   * @param hijackY The supplier of whether to hijack the y-axis velocity.
   * @param hijackOmega The supplier of whether to hijack (override) the omega.
   * @param hijackedX The supplier of the hijacked x-axis value velocity.
   * @param hijackedY The supplier of the hijacked y-axis value velocity.
   * @param hijackedOmega The supplier of the hijacked omega value.
   * @return A command that drives the SwerveDrive.
   */
  @Trace
  public static Command joystickDrive(
      SwerveDrive drive,
      SwerveDriveConstants driveConstants,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier,
      Supplier<Rotation2d> rotationSupplier,
      BooleanSupplier hijackX,
      BooleanSupplier hijackY,
      BooleanSupplier hijackOmega,
      DoubleSupplier hijackedX,
      DoubleSupplier hijackedY,
      DoubleSupplier hijackedOmega) {
    return Commands.run(
        () -> {
          // Apply deadband
          double linearMagnitude =
              MathUtil.applyDeadband(
                  Math.hypot(xSupplier.getAsDouble(), ySupplier.getAsDouble()),
                  driveConstants.driverDeadband);
          Rotation2d linearDirection =
              new Rotation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble());

          double omega =
              MathUtil.applyDeadband(omegaSupplier.getAsDouble(), driveConstants.driverDeadband);
          linearMagnitude *= linearMagnitude;

          // Calculate new linear velocities

          double fieldRelativeXVel =
              linearMagnitude * linearDirection.getCos() * drive.getMaxLinearSpeedMetersPerSec();
          double fieldRelativeYVel =
              linearMagnitude * linearDirection.getSin() * drive.getMaxLinearSpeedMetersPerSec();

          double angular = omega * drive.getMaxAngularSpeedRadPerSec();

          fieldRelativeXVel = hijackX.getAsBoolean() ? hijackedX.getAsDouble() : fieldRelativeXVel;

          fieldRelativeYVel = hijackY.getAsBoolean() ? hijackedY.getAsDouble() : fieldRelativeYVel;

          angular = hijackOmega.getAsBoolean() ? hijackedOmega.getAsDouble() : angular;

          ChassisSpeeds chassisSpeeds =
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  fieldRelativeXVel,
                  fieldRelativeYVel,
                  angular,
                  AllianceFlipUtil.apply(rotationSupplier.get()));

          Logger.recordOutput("Drive/JoystickDrive/chassisSpeeds", chassisSpeeds);

          drive.runVelocity(chassisSpeeds);
        },
        drive);
  }

  public static Command joystickDrive(
      SwerveDrive drive,
      SwerveDriveConstants driveConstants,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier,
      Supplier<Rotation2d> rotationSupplier) {
    return joystickDrive(
        drive,
        driveConstants,
        xSupplier,
        ySupplier,
        omegaSupplier,
        rotationSupplier,
        () -> false,
        () -> false,
        () -> false,
        () -> 0.0,
        () -> 0.0,
        () -> 0.0);
  }

  public static Command joystickDriveAlignToHub(
      SwerveDrive drive,
      SwerveDriveConstants driveConstants,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier,
      Supplier<Rotation2d> rotationSupplier,
      BooleanSupplier hijackOmega,
      DoubleSupplier setpoint) {
    ProfiledPIDController omegaController =
        new ProfiledPIDController(
            driveConstants.autoAlignConstants.omegaPIDConstants().kP().get(),
            0.0,
            driveConstants.autoAlignConstants.omegaPIDConstants().kD().get(),
            new TrapezoidProfile.Constraints(
                driveConstants.autoAlignConstants.omegaPIDConstants().maxVelocity().get(),
                Double.POSITIVE_INFINITY));
    return joystickDrive(
        drive,
        driveConstants,
        xSupplier,
        ySupplier,
        omegaSupplier,
        rotationSupplier,
        () -> false,
        () -> false,
        hijackOmega,
        () -> 0.0,
        () -> 0.0,
        () ->
            AutoAlignCommand.calculate(
                omegaController,
                setpoint.getAsDouble(),
                rotationSupplier.get().getRadians(),
                drive.getMeasuredChassisSpeeds().omegaRadiansPerSecond));
  }

  public static Command inchMovement(SwerveDrive drive, double velocity, double time) {
    return Commands.run(() -> drive.runVelocity(new ChassisSpeeds(0.0, velocity, 0.0)))
        .withTimeout(time);
  }

  public static Command stop(SwerveDrive drive) {
    return Commands.run(drive::stopWithX);
  }

  public static Command feedforwardCharacterization(SwerveDrive drive) {
    return new KSCharacterization(
        drive, drive::runCharacterization, drive::getFFCharacterizationVelocity);
  }

  public static Command autoAlignPoseCommand(
      SwerveDrive drive,
      Supplier<Pose2d> robotPoseSupplier,
      Pose2d targetPose,
      AutoAlignNearConstants constants) {
    return new AutoAlignCommand(
        drive, targetPose, () -> true, robotPoseSupplier, constants, Double.POSITIVE_INFINITY);
  }

  public static Command autoAlignTowerCommand(
      SwerveDrive drive, Supplier<Pose2d> robotPoseSupplier, AutoAlignNearConstants constants) {
    return autoAlignPoseCommand(
        drive,
        robotPoseSupplier,
        new Pose2d(FieldConstants.Tower.centerPoint, new Rotation2d()),
        constants);
  }

  public static Command wheelRadiusCharacterization(SwerveDrive drive) {
    double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
    double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2
    SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
    WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

    return Commands.parallel(
        // SwerveDrive control sequence
        Commands.sequence(
            // Reset acceleration limiter
            Commands.runOnce(
                () -> {
                  limiter.reset(0.0);
                }),

            // Turn in place, accelerating up to full speed
            Commands.run(
                () -> {
                  double speed = limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY);
                  drive.runVelocity(new ChassisSpeeds(0.0, 0.0, speed));
                },
                drive)),

        // Measurement sequence
        Commands.sequence(
            // Wait for modules to fully orient before starting measurement
            Commands.waitSeconds(1.0),

            // Record starting measurement
            Commands.runOnce(
                () -> {
                  state.positions = drive.getWheelRadiusCharacterizationPositions();
                  state.lastAngle = drive.getRawGyroRotation();
                  state.gyroDelta = 0.0;
                }),

            // Update gyro delta
            Commands.run(
                    () -> {
                      var rotation = drive.getRawGyroRotation();
                      state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                      state.lastAngle = rotation;
                    })

                // When cancelled, calculate and print results
                .finallyDo(
                    () -> {
                      double[] positions = drive.getWheelRadiusCharacterizationPositions();
                      double wheelDelta = 0.0;
                      for (int i = 0; i < 4; i++) {
                        wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                      }
                      double wheelRadius =
                          (state.gyroDelta * /*DriveConstants.DRIVE_CONFIG.driveBaseRadius()*/ 0)
                              / wheelDelta;

                      NumberFormat formatter = new DecimalFormat("#0.000");
                      System.out.println(
                          "********** Wheel Radius Characterization Results **********");
                      System.out.println(
                          "\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
                      System.out.println(
                          "\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
                      System.out.println(
                          "\tWheel Radius: "
                              + formatter.format(wheelRadius)
                              + " meters, "
                              + formatter.format(Units.metersToInches(wheelRadius))
                              + " inches");
                    })));
  }

  private static class WheelRadiusCharacterizationState {
    double[] positions = new double[4];
    Rotation2d lastAngle = new Rotation2d();
    double gyroDelta = 0.0;
  }
}
