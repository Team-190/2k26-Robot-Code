package first.robot.shared.util;

import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;
import edu.wpi.team190.gompeilib.core.GompeiLib;
import edu.wpi.team190.gompeilib.core.io.components.inertial.GyroIO;
import edu.wpi.team190.gompeilib.core.utility.control.Gains;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveDrive;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveDriveConstants;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveModuleIO;
import java.util.function.Consumer;
import java.util.function.Supplier;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;
import org.wpilib.math.controller.PIDController;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.util.Units;

/**
 * A {@link SwerveDrive} that can follow Choreo trajectories. GompeiLib no longer depends on Choreo,
 * so the {@link AutoFactory} and trajectory follower live here instead.
 */
public class ChoreoSwerveDrive extends SwerveDrive {
  private final Supplier<Pose2d> robotPoseSupplier;

  private final PIDController autoXController;
  private final PIDController autoYController;
  private final PIDController autoHeadingController;

  @Getter private final AutoFactory autoFactory;

  public ChoreoSwerveDrive(
      SwerveDriveConstants driveConstants,
      GyroIO gyroIO,
      SwerveModuleIO flModuleIO,
      SwerveModuleIO frModuleIO,
      SwerveModuleIO blModuleIO,
      SwerveModuleIO brModuleIO,
      Supplier<Pose2d> robotPoseSupplier,
      Consumer<Pose2d> resetPoseConsumer) {
    super(
        driveConstants,
        gyroIO,
        flModuleIO,
        frModuleIO,
        blModuleIO,
        brModuleIO,
        robotPoseSupplier,
        resetPoseConsumer);
    this.robotPoseSupplier = robotPoseSupplier;

    autoXController = new PIDController(0.0, 0.0, 0.0, GompeiLib.getLoopPeriod());
    autoYController = new PIDController(0.0, 0.0, 0.0, GompeiLib.getLoopPeriod());
    autoHeadingController = new PIDController(0.0, 0.0, 0.0, GompeiLib.getLoopPeriod());
    autoHeadingController.enableContinuousInput(-Math.PI, Math.PI);
    autoHeadingController.setTolerance(Units.degreesToRadians(1.0));

    setChoreoControllers(driveConstants.autoTranslationGains, driveConstants.autoRotationGains);
    driveConstants.autoTranslationGains.onChange(
        gains -> setChoreoControllers(gains, driveConstants.autoRotationGains));
    driveConstants.autoRotationGains.onChange(
        gains -> setChoreoControllers(driveConstants.autoTranslationGains, gains));

    autoFactory =
        new AutoFactory(robotPoseSupplier, resetPoseConsumer, this::choreoDrive, true, this);
  }

  private void setChoreoControllers(Gains translationGains, Gains rotationGains) {
    autoXController.setPID(translationGains.getKP(), 0.0, translationGains.getKD());
    autoYController.setPID(translationGains.getKP(), 0.0, translationGains.getKD());
    autoHeadingController.setPID(rotationGains.getKP(), 0.0, rotationGains.getKD());
  }

  /** Runs a choreo path from swerve samples */
  public void choreoDrive(SwerveSample sample) {
    Pose2d pose = robotPoseSupplier.get();
    double xFF = sample.vx;
    double yFF = sample.vy;
    double rotationFF = sample.omega;

    double xFeedback = autoXController.calculate(pose.getX(), sample.x);
    double yFeedback = autoYController.calculate(pose.getY(), sample.y);
    double rotationFeedback =
        autoHeadingController.calculate(pose.getRotation().getRadians(), sample.heading);

    ChassisVelocities velocity =
        new ChassisVelocities(xFF + xFeedback, yFF + yFeedback, rotationFF + rotationFeedback)
            .toRobotRelative(Rotation2d.fromRadians(sample.heading));

    runVelocity(velocity);
    Logger.recordOutput("Auto/Setpoint", sample.getPose());
  }
}
