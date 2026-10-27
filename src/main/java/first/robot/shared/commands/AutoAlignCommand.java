package first.robot.shared.commands;

import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.Radians;
import static org.wpilib.units.Units.RadiansPerSecond;

import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveDrive;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveDriveConstants.AutoAlignConstants;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command2.Command;
import org.wpilib.math.controller.ProfiledPIDController;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.trajectory.TrapezoidProfile;

public class AutoAlignCommand extends Command {
  private final SwerveDrive drive;
  private final Pose2d targetPose;
  private final BooleanSupplier valid;
  private final Supplier<Pose2d> robotPose;

  private ChassisVelocities speeds;

  private final ProfiledPIDController alignXController;
  private final ProfiledPIDController alignYController;
  private final ProfiledPIDController alignHeadingController;

  /**
   * Creates a new AutoAlignCommand.
   *
   * @param drive The swerve drive subsystem on which this command will run
   * @param targetPose The pose to which the robot will attempt to align
   * @param valid A boolean supplier that returns true when auto aligning is possible (e.g. when
   *     tags are visible)
   * @param robotPose A supplier that returns the robot's current pose
   * @param constants The swerve drive constants
   */
  public AutoAlignCommand(
      SwerveDrive drive,
      Pose2d targetPose,
      BooleanSupplier valid,
      Supplier<Pose2d> robotPose,
      AutoAlignConstants constants,
      double maxAccelerationMetersPerSecond) {
    this.addRequirements(drive);

    this.drive = drive;
    this.targetPose = targetPose;
    this.valid = valid;
    this.robotPose = robotPose;

    alignXController =
        new ProfiledPIDController(
            constants.xGains().kP().get(),
            0.0,
            constants.xGains().kD().get(),
            new TrapezoidProfile.Constraints(
                constants.xConstraints().maxVelocity().get().in(MetersPerSecond),
                maxAccelerationMetersPerSecond));
    alignYController =
        new ProfiledPIDController(
            constants.yGains().kP().get(),
            0.0,
            constants.yGains().kD().get(),
            new TrapezoidProfile.Constraints(
                constants.yConstraints().maxVelocity().get().in(MetersPerSecond),
                maxAccelerationMetersPerSecond));
    alignHeadingController =
        new ProfiledPIDController(
            constants.rotationGains().kP().get(),
            0.0,
            constants.rotationGains().kD().get(),
            new TrapezoidProfile.Constraints(
                constants.rotationConstraints().maxVelocity().get().in(RadiansPerSecond),
                Double.POSITIVE_INFINITY));

    alignXController.setTolerance(constants.xConstraints().goalTolerance().get().in(Meters), 0);
    alignYController.setTolerance(constants.xConstraints().goalTolerance().get().in(Meters), 0);

    alignHeadingController.enableContinuousInput(-Math.PI, Math.PI);
    alignHeadingController.setTolerance(
        constants.rotationConstraints().goalTolerance().get().in(Radians), 0);
    speeds = new ChassisVelocities();
  }

  @Override
  public void initialize() {
    Logger.recordOutput("Drive/Auto Align/Goal", targetPose);

    alignHeadingController.reset(
        robotPose.get().getRotation().getRadians(), drive.getMeasuredChassisVelocities().omega);
    alignXController.reset(robotPose.get().getX(), drive.getMeasuredChassisVelocities().vx);
    alignYController.reset(robotPose.get().getY(), drive.getMeasuredChassisVelocities().vy);
  }

  @Override
  public void execute() {

    if (valid.getAsBoolean()) {
      ChassisVelocities measuredSpeeds = drive.getMeasuredChassisVelocities();

      double adjustedXSpeed =
          calculate(alignXController, targetPose.getX(), robotPose.get().getX(), measuredSpeeds.vx);
      double adjustedYSpeed =
          calculate(alignYController, targetPose.getY(), robotPose.get().getY(), measuredSpeeds.vy);
      double adjustedThetaSpeed =
          calculate(
              alignHeadingController,
              targetPose.getRotation().getRadians(),
              robotPose.get().getRotation().getRadians(),
              measuredSpeeds.omega);
      speeds =
          new ChassisVelocities(adjustedXSpeed, adjustedYSpeed, adjustedThetaSpeed)
              .toRobotRelative(robotPose.get().getRotation());

    } else {
      speeds = new ChassisVelocities();
    }
    Logger.recordOutput("Drive/Auto Align/speeds", speeds);
    drive.runVelocity(speeds);
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
    alignHeadingController.reset(robotPose.get().getRotation().getRadians());
    alignXController.reset(robotPose.get().getX());
    alignYController.reset(robotPose.get().getY());
  }

  @Override
  public boolean isFinished() {
    // Return true when the command should end
    return alignXController.atGoal()
        && alignYController.atGoal()
        && alignHeadingController.atGoal();
  }

  /**
   * Calculates the PID output for the given setpoint, measurement, and speed. If the controller is
   * not at the setpoint, calculate the PID output. Otherwise, reset the controller with the given
   * measurement and speed.
   *
   * @param controller The profiled PID controller to use.
   * @param setpoint The setpoint of the controller.
   * @param measurement The current measurement.
   * @param speed The current speed.
   * @return The calculated PID output.
   */
  public static double calculate(
      ProfiledPIDController controller, double setpoint, double measurement, double speed) {
    double pidOutput = 0.0;

    if (!controller.atSetpoint()) pidOutput = controller.calculate(measurement, setpoint);
    else controller.reset(measurement, speed);

    return pidOutput;
  }
}
