package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.team190.gompeilib.core.logging.Trace;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveDrive;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveDriveConstants;
import frc.robot.FieldConstants;
import frc.robot.subsystems.v1_Gamma.V1_GammaConstants;
import frc.robot.util.AllianceFlipUtil;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public final class DriveCommands {

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   */
  @Trace
  public static Command joystickDrive(
      SwerveDrive drive,
      SwerveDriveConstants driveConstants,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier,
      Supplier<Rotation2d> rotationSupplier) {
    return Commands.run(
        () -> {
          // Apply deadband
          double linearMagnitude =
              MathUtil.applyDeadband(
                  Math.hypot(xSupplier.getAsDouble(), ySupplier.getAsDouble()),
                  driveConstants.DRIVER_DEADBAND);
          Rotation2d linearDirection =
              new Rotation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble());

          double omega =
              MathUtil.applyDeadband(omegaSupplier.getAsDouble(), driveConstants.DRIVER_DEADBAND);
          linearMagnitude *= linearMagnitude;

          // Calculate new linear velocities

          double fieldRelativeXVel =
              linearMagnitude * linearDirection.getCos() * drive.getMaxLinearSpeedMetersPerSec();
          double fieldRelativeYVel =
              linearMagnitude * linearDirection.getSin() * drive.getMaxLinearSpeedMetersPerSec();

          double angular = omega * drive.getMaxAngularSpeedRadPerSec();

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

  public static Command autoAlignTowerCommand(
      SwerveDrive drive, Supplier<Pose2d> robotPoseSupplier) {
    return new AutoAlignCommand(
        drive,
        new Pose2d(FieldConstants.Tower.centerPoint, new Rotation2d()),
        () -> true,
        robotPoseSupplier,
        V1_GammaConstants.AUTO_ALIGN_NEAR_CONSTANTS);
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
