package frc.robot.subsystems.shared.intake;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.team190.gompeilib.core.GompeiLib;
import edu.wpi.team190.gompeilib.core.utility.Setpoint;
import edu.wpi.team190.gompeilib.subsystems.generic.roller.GenericRoller;
import edu.wpi.team190.gompeilib.subsystems.generic.roller.GenericRollerIO;
import frc.robot.subsystems.shared.fourbarlinkage.FourBarLinkage;
import frc.robot.subsystems.shared.fourbarlinkage.FourBarLinkageIO;
import frc.robot.subsystems.shared.intake.IntakeConstants.IntakeState;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final GenericRoller roller;
  @Getter private final FourBarLinkage linkage;

  @Getter private IntakeState intakeState;
  private final Setpoint<VoltageUnit> intakeVoltage;

  private Rotation2d agitationAngle;

  public Intake(GenericRollerIO rollerIO, FourBarLinkageIO linkageIO) {
    setName("Intake");
    roller = new GenericRoller(rollerIO, this, IntakeConstants.INTAKE_ROLLER_CONSTANTS_TOP, " 1");
    linkage = new FourBarLinkage(linkageIO, IntakeConstants.LINKAGE_CONSTANTS, this, " 1");

    intakeState = IntakeState.STOW;
    intakeVoltage = new Setpoint<>(Volts.of(IntakeConstants.INTAKE_VOLTAGE), Volts.of(0.5));

    agitationAngle = Rotation2d.fromDegrees(150);
  }

  @Override
  public void periodic() {
    roller.periodic();
    linkage.periodic();

    Logger.recordOutput("Intake/Intake State", intakeState);
    Logger.recordOutput("Intake/Agitation Angle", agitationAngle);

    // This needs to change now that its shared, we can't reference the v1 robot state

    // V1_DoomSpiralRobotState.getLedStates().setIntakeIn(false);
    // if (intakeState.equals(IntakeState.INTAKE) || intakeState.equals(IntakeState.BUMP)) {
    //   V1_DoomSpiralRobotState.getLedStates()
    //       .setIntakeCollecting(
    //           roller.getVoltageGoal().getSetpoint().baseUnitMagnitude()
    //               == IntakeConstants.INTAKE_VOLTAGE);
    //   V1_DoomSpiralRobotState.getLedStates().setIntakeIn(false);
    // } else {
    //   V1_DoomSpiralRobotState.getLedStates().setIntakeIn(true);
    //   V1_DoomSpiralRobotState.getLedStates().setIntakeCollecting(false);
    // }
    // V1_DoomSpiralRobotState.getLedStates()
    //     .setSpitting(
    //         roller.getVoltageGoal().getSetpoint().baseUnitMagnitude()
    //             == IntakeConstants.EXTAKE_VOLTAGE);
  }

  /**
   * Sets the voltage of the top and bottom rollers of the intake subsystem. The voltage is offset
   * by the roller voltage offset stored in the robot state.
   *
   * @param voltage the voltage to set the rollers to
   * @return a command that sets the voltage of the top and bottom rollers
   */
  public Command setRollerVoltage(double voltage) {
    return Commands.runOnce(() -> roller.setVoltageGoal(Volts.of(voltage)));
  }

  /**
   * Sets the voltage of the linkage subsystem of the intake.
   *
   * @param voltage the voltage to set the linkage to
   * @return a command that sets the voltage of the linkage
   */
  public Command setLinkageVoltage(double voltage) {
    return Commands.runOnce(() -> linkage.setVoltageGoal(Volts.of(voltage)));
  }

  public Command stopRoller() {
    return Commands.runOnce(() -> roller.setVoltageGoal(Volts.of(0)));
  }

  public Command deploy() {
    return Commands.sequence(
        Commands.runOnce(() -> intakeState = IntakeState.INTAKE),
        Commands.runOnce(() -> linkage.setPositionGoal(
            IntakeState.INTAKE.getAngle())));
  }

  public Command stow() {
    return Commands.sequence(
        Commands.runOnce(() -> intakeState = IntakeState.INTAKE),
        Commands.runOnce(() -> linkage.setPositionGoal(
            IntakeState.STOW.getAngle())));
  }

  public Command bump() {
    return Commands.sequence(
        Commands.runOnce(() -> intakeState = IntakeState.INTAKE),
        Commands.runOnce(() -> linkage.setPositionGoal(
            IntakeState.BUMP.getAngle())));
  }

  public Command agitate() {
    return Commands.parallel(
        Commands.sequence(
                Commands.runOnce(() -> agitationAngle = Rotation2d.fromDegrees(170 + 8)),
                Commands.runOnce(() -> linkage.setPositionGoal(agitationAngle)),
                linkage.waitUntilLinkageAtGoal(),
                Commands.runOnce(() -> linkage.setPositionGoal(Rotation2d.fromDegrees(98))),
                linkage.waitUntilLinkageAtGoal())
            .repeatedly(),
        setRollerVoltage(3.0));
  }

  public Command collect() {
    return Commands.parallel(deploy(), setRollerVoltage(intakeVoltage.getNewSetpoint().in(Volts)));
  }

  public Command stopCollect() {
    return Commands.sequence(
        Commands.parallel(stow(), setRollerVoltage(IntakeConstants.EXTAKE_VOLTAGE)),
        waitUntilIntakeAtGoal(),
        stopRoller());
  }

  public Command resetIntakeZero() {
    return Commands.sequence(
        Commands.runOnce(() -> linkage.setPosition(IntakeConstants.MIN_ANGLE)),
        Commands.runOnce(() -> linkage.setPositionGoal(IntakeConstants.MIN_ANGLE)));
  }

  public Transform3d getHopperWallTransform() {
    // 1. Calculate Current Pose
    final double currentY = linkage.getPosition().getSin() * IntakeConstants.PIN_LENGTH;
    final double currentX0 = linkage.getPosition().getCos() * IntakeConstants.PIN_LENGTH;
    final double currentXOff = calculateXOffset(currentY);
    Pose3d currentPose = new Pose3d(-(currentX0 + currentXOff), 0, 0, new Rotation3d(0, 0, 0));

    // 2. Calculate "Zero" Pose (at Y_MIN)
    final double zeroY = IntakeConstants.LINK_BOUNDS.MIN();
    final double zeroAngle = Math.asin(zeroY / IntakeConstants.PIN_LENGTH);
    final double zeroX0 = Math.cos(zeroAngle) * IntakeConstants.PIN_LENGTH;
    final double zeroXOff = calculateXOffset(zeroY);
    Pose3d zeroPose = new Pose3d(-(zeroX0 + zeroXOff), 0, 0, new Rotation3d(0, 0, 0));

    // 3. Return the Transform (Zero -> Current)
    return currentPose.minus(zeroPose);
  }

  /** Piecewise logic for the linkage offset */
  private double calculateXOffset(double yPos) {
    final double Y_MIN = IntakeConstants.LINK_BOUNDS.MIN();
    final double Y_PHASE_1 = IntakeConstants.LINK_BOUNDS.PHASE_1();
    final double Y_PHASE_2 = IntakeConstants.LINK_BOUNDS.PHASE_2();
    final double Y_MAX = IntakeConstants.LINK_BOUNDS.MAX();

    final double RADIUS_1 = IntakeConstants.LINK_CONST.RADIUS_1();
    final double RADIUS_2 = IntakeConstants.LINK_CONST.RADIUS_2();
    final double CENTER_OFFSET = IntakeConstants.LINK_CONST.CENTER_OFFSET();

    if (yPos <= Y_PHASE_1 && yPos >= Y_MIN) {
      return Math.sqrt(Math.pow(RADIUS_1, 2) - Math.pow(yPos, 2)) - CENTER_OFFSET;
    } else if (yPos <= Y_PHASE_2 && yPos > Y_PHASE_1) {
      return 0;
    } else if (yPos <= Y_MAX && yPos > Y_PHASE_2) {
      return Math.sqrt(Math.pow(RADIUS_2, 2) - Math.pow(yPos - Y_PHASE_2, 2)) - RADIUS_2;
    }
    return 0;
  }

  public boolean atPositionGoal() {
    return linkage.atPositionGoal();
  }

  public Command waitUntilIntakeAtGoal() {
    return Commands.waitSeconds(GompeiLib.getLoopPeriod())
        .andThen(linkage.waitUntilLinkageAtGoal());
  }

  public Command incrementStowOffset() {
    return Commands.sequence(
        Commands.runOnce(
          () -> IntakeState.INTAKE.getAngle().increment()
        ),
        stow());
  }

  public Command decrementStowOffset() {
    return Commands.sequence(
        Commands.runOnce(
          () -> IntakeState.INTAKE.getAngle().decrement()
        ),
        stow());
  }

  public Command incrementBumpOffset() {
    return Commands.sequence(
        Commands.runOnce(
          () -> IntakeState.BUMP.getAngle().increment()
        ),
        bump());
  }

  public Command decrementBumpOffset() {
    return Commands.sequence(
        Commands.runOnce(
          () -> IntakeState.BUMP.getAngle().decrement()
        ),
        bump());
  }

  public Command incrementCollectOffset() {
    return Commands.sequence(
        Commands.runOnce(
          () -> IntakeState.INTAKE.getAngle().increment()
        ),
        deploy());
  }

  public Command decrementCollectOffset() {
        return Commands.sequence(
        Commands.runOnce(
          () -> IntakeState.INTAKE.getAngle().decrement()
        ),
        deploy());
  }

  public Command increaseSpeedOffset() {
    return Commands.runOnce(() -> intakeVoltage.increment());
  }

  public Command decreaseSpeedOffset() {
    return Commands.runOnce(() -> intakeVoltage.decrement());
  }

  public Command defaultCommand() {
    Command defaultCommand =
        Commands.either(
            Commands.either(stopRoller(), Commands.none(), this::atPositionGoal),
            Commands.none(),
            () -> (intakeState.equals(IntakeState.STOW)));
    defaultCommand.addRequirements(this);
    return defaultCommand;
  }

  public Command linkageSysId() {
    return linkage.runSysId();
  }
}
