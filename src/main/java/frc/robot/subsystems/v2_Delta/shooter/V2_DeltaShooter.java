package frc.robot.subsystems.v2_Delta.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.team190.gompeilib.core.logging.Trace;
import edu.wpi.team190.gompeilib.subsystems.generic.flywheel.GenericFlywheel;
import edu.wpi.team190.gompeilib.subsystems.generic.flywheel.GenericFlywheelIO;
import frc.robot.subsystems.shared.hood.Hood;
import frc.robot.subsystems.shared.hood.HoodIO;
import frc.robot.subsystems.shared.turret.Turret;
import frc.robot.subsystems.shared.turret.TurretIO;
import frc.robot.subsystems.v1_DoomSpiral.V1_DoomSpiralRobotState;
import frc.robot.subsystems.v2_Delta.V2_DeltaRobotState;
import frc.robot.subsystems.v2_Delta.shooter.V2_DeltaShooterConstants.HoodGoal;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class V2_DeltaShooter extends SubsystemBase {

  private final Hood hood;

  private HoodGoal hoodGoal;

  private final GenericFlywheel flywheel;

  private final Turret turret;

  public V2_DeltaShooter(GenericFlywheelIO flywheelIO, HoodIO hoodIO, TurretIO turretIO) {
    setName("Shooter");

    flywheel = new GenericFlywheel(flywheelIO, this, V2_DeltaShooterConstants.SHOOT_CONSTANTS, "");
    hood = new Hood(hoodIO, V2_DeltaShooterConstants.HOOD_CONSTANTS, this, "");

    hoodGoal = HoodGoal.STOW;

    turret =
        new Turret(
            turretIO,
            this,
            " ",
            V2_DeltaRobotState::getGlobalPose,
            V2_DeltaShooterConstants.TURRET_CONSTANTS);
  }

  @Trace
  public void periodic() {
    hood.periodic();
    flywheel.periodic();

    Logger.recordOutput("Shooter/Hood/Goal", hoodGoal);

    // if (hoodGoal.equals(HoodGoal.SCORE) || hoodGoal.equals(HoodGoal.FEED)) {
    //   V1_DoomSpiralRobotState.getLedStates().setShooterPrepping(true);
    //   V1_DoomSpiralRobotState.getLedStates().setShooterShooting(atGoal());
    // } else {
    //   V1_DoomSpiralRobotState.getLedStates().setShooterPrepping(false);
    //   V1_DoomSpiralRobotState.getLedStates().setShooterShooting(false);
    // }
  }

    /**
   * Returns the angle from the robot's current position to the target position. This is calculated
   * by subtracting the robot's current position from the target position, adding the turret's
   * translation (rotated by the robot's current angle), and then taking the angle of the resulting
   * translation from the robot's current angle.
   *
   * @param robotPose the robot's current pose
   * @param targetTranslation the target position
   * @return the angle from the robot's current position to the target position
   */
  public static Angle fieldToTurret(Pose2d robotPose, Translation2d targetTranslation) {
    Translation2d turretToTargetTranslation = targetTranslation.minus(robotPose.getTranslation());
    // .plus(V2_DeltaShooterConstants.TURRET_TRANSLATION.rotateBy(robotPose.getRotation()));
    Rotation2d turretRotation = turretToTargetTranslation.getAngle().minus(robotPose.getRotation());
    return turretRotation.getMeasure();
  }

  public Command setHoodGoal(HoodGoal goal) {
    return Commands.runOnce(
        () -> {
          hoodGoal = goal;
          hood.setPositionGoal(getHoodGoal(goal));
        });
  }

  private Rotation2d getHoodGoal(HoodGoal goal) {
    Rotation2d rotation =
        switch (goal) {
          case SCORE -> V1_DoomSpiralRobotState.getScoreAngle();
          case FEED -> V1_DoomSpiralRobotState.getFeedAngle();
          default -> Rotation2d.kZero;
        };
    Logger.recordOutput("Shooter/Hood/DebugGoal", rotation);
    Logger.recordOutput("Shooter/Hood/debugGoal1", goal);
    return rotation;
  }

  public Command setOverrideHoodGoal(Rotation2d position) {
    return Commands.runOnce(() -> hood.setPositionGoal(position))
        .andThen(Commands.runOnce(() -> hoodGoal = HoodGoal.OVERRIDE));
  }

  public Command setHoodVoltage(double volts) {
    return Commands.runOnce(() -> hood.setVoltageGoal(Volts.of(volts)));
  }

  public Command stopHood() {
    return Commands.runOnce(() -> hood.setVoltageGoal(Volts.zero()));
  }

  public Command zeroHood() {
    return hood.resetHoodZero();
  }

  public Command setFlywheelGoal(AngularVelocity velocityGoal) {
    return Commands.runOnce(() -> flywheel.setVelocityGoal(velocityGoal));
  }

  public Command setFlywheelGoal(AngularVelocity velocityGoal, Current feedforward) {
    return Commands.runOnce(() -> flywheel.setVelocityGoal(velocityGoal, feedforward));
  }

  public Command setFlywheelVoltage(double volts) {
    return Commands.runOnce(() -> flywheel.setVoltageGoal(Volts.of(volts)));
  }

  public Command stopFlywheel() {
    return Commands.runOnce(flywheel::stop);
  }

  public Command setGoal(HoodGoal hoodGoal, double velocityRadiansPerSecond) {
    return Commands.parallel(
        setHoodGoal(hoodGoal), setFlywheelGoal(RadiansPerSecond.of(velocityRadiansPerSecond)));
  }

  public Command setGoal(HoodGoal hoodGoal, DoubleSupplier velocityRadiansPerSecond) {
    return Commands.parallel(
        setHoodGoal(hoodGoal),
        Commands.run(
            () ->
                flywheel.setVelocityGoal(
                    RadiansPerSecond.of(velocityRadiansPerSecond.getAsDouble()))));
  }

  public boolean atGoal() {
    return hood.atPositionGoal() && flywheel.atVelocityGoal();
  }

  public Command waitUntilAtGoal() {
    return hood.waitUntilAtGoal().alongWith(flywheel.waitUntilAtGoal());
  }

  public Command waitUntilFlywheelAtGoal() {
    return flywheel.waitUntilAtGoal();
  }

  public Command hoodSysId() {
    return hood.runSysId();
  }

  public Command flywheelSysId() {
    return flywheel.sysIdRoutineTorque();
  }

  public Command incrementFlywheelVelocity() {
    return Commands.runOnce(flywheel.getVelocityGoal()::increment);
  }

  public Command decrementFlywheelVelocity() {
    return Commands.runOnce(flywheel.getVelocityGoal()::decrement);
  }

  public Command incrementHoodAngle() {
    return Commands.runOnce(hood.getPositionGoal()::increment);
  }

  public Command decrementHoodAngle() {
    return Commands.runOnce(hood.getPositionGoal()::decrement);
  }
}
