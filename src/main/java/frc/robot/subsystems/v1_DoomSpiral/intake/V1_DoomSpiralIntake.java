package frc.robot.subsystems.v1_DoomSpiral.intake;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.team190.gompeilib.subsystems.generic.roller.GenericRoller;
import edu.wpi.team190.gompeilib.subsystems.generic.roller.GenericRollerIO;
import frc.robot.subsystems.shared.fourbarlinkage.FourBarLinkage;
import frc.robot.subsystems.shared.fourbarlinkage.FourBarLinkageIO;
import frc.robot.subsystems.v1_DoomSpiral.V1_DoomSpiralRobotState;
import frc.robot.subsystems.v1_DoomSpiral.intake.V1_DoomSpiralIntakeConstants.IntakeState;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class V1_DoomSpiralIntake extends SubsystemBase {
  private final GenericRoller roller;
  @Getter private final FourBarLinkage linkage;

  @Getter private IntakeState intakeState;

  private Rotation2d agitationAngle;

  public V1_DoomSpiralIntake(GenericRollerIO rollerIO, FourBarLinkageIO linkageIO) {
    setName("Intake");
    roller =
        new GenericRoller(
            rollerIO, this, V1_DoomSpiralIntakeConstants.INTAKE_ROLLER_CONSTANTS_TOP, "1");
    linkage =
        new FourBarLinkage(linkageIO, V1_DoomSpiralIntakeConstants.LINKAGE_CONSTANTS, this, 0);

    intakeState = IntakeState.STOW;

    agitationAngle = Rotation2d.fromDegrees(150);

    setDefaultCommand(defaultCommand());
  }

  @Override
  public void periodic() {
    roller.periodic();
    linkage.periodic();

    Logger.recordOutput("Intake/Intake State", intakeState);
    Logger.recordOutput("Intake/Agitation Angle", agitationAngle);

    V1_DoomSpiralRobotState.getLedStates().setIntakeIn(false);
    if (intakeState.equals(IntakeState.INTAKE) || intakeState.equals(IntakeState.BUMP)) {
      V1_DoomSpiralRobotState.getLedStates()
          .setIntakeCollecting(
              roller.getVoltageGoalVolts().getSetpoint().baseUnitMagnitude()
                  == V1_DoomSpiralIntakeConstants.INTAKE_VOLTAGE);
      V1_DoomSpiralRobotState.getLedStates().setIntakeIn(false);
    } else {
      V1_DoomSpiralRobotState.getLedStates().setIntakeIn(true);
      V1_DoomSpiralRobotState.getLedStates().setIntakeCollecting(false);
    }
    V1_DoomSpiralRobotState.getLedStates()
        .setSpitting(
            roller.getVoltageGoalVolts().getSetpoint().baseUnitMagnitude()
                == V1_DoomSpiralIntakeConstants.EXTAKE_VOLTAGE);
  }

  /**
   * Sets the voltage of the top and bottom rollers of the intake subsystem. The voltage is offset
   * by the roller voltage offset stored in the robot state.
   *
   * @param voltage the voltage to set the rollers to
   * @return a command that sets the voltage of the top and bottom rollers
   */
  public Command setRollerVoltage(double voltage) {
    return roller.setVoltage(voltage);
  }

  /**
   * Sets the voltage of the linkage subsystem of the intake.
   *
   * @param voltage the voltage to set the linkage to
   * @return a command that sets the voltage of the linkage
   */
  public Command setLinkageVoltage(double voltage) {
    return linkage.setVoltage(voltage);
  }

  public Command stopRoller() {
    return roller.setVoltage(0);
  }

  public Command deploy() {
    return Commands.sequence(
        Commands.runOnce(() -> intakeState = IntakeState.INTAKE),
        linkage.setPositionGoal(
            IntakeState.INTAKE.getAngle(),
            (() -> V1_DoomSpiralRobotState.getIntakeOffsets().getCollectOffset())));
  }

  public Command stow() {
    return Commands.sequence(
        Commands.runOnce(() -> intakeState = IntakeState.STOW),
        linkage.setPositionGoal(
            IntakeState.STOW.getAngle(),
            (() -> V1_DoomSpiralRobotState.getIntakeOffsets().getStowOffset())));
  }

  public Command bump() {
    return Commands.sequence(
        Commands.runOnce(() -> intakeState = IntakeState.BUMP),
        linkage.setPositionGoal(
            IntakeState.BUMP.getAngle(),
            (() -> V1_DoomSpiralRobotState.getIntakeOffsets().getBumpOffset())));
  }

  public Command agitate() {
    // return Commands.parallel(
    //     Commands.sequence(
    //         Commands.runOnce(() -> agitationAngle = Rotation2d.fromDegrees(150)),
    //         Commands.sequence(
    //                 linkage.setPositionGoal(
    //                     () -> agitationAngle,
    //                     (() -> V1_DoomSpiralRobotState.getIntakeOffsets().getStowOffset())),
    //                 waitUntilIntakeAtGoal(),
    //                 Commands.runOnce(
    //                     () -> agitationAngle = agitationAngle.minus(agitationDelta.times(2))),
    //                 linkage.setPositionGoal(
    //                     () -> agitationAngle,
    //                     (() -> V1_DoomSpiralRobotState.getIntakeOffsets().getStowOffset())),
    //                 waitUntilIntakeAtGoal(),
    //                 Commands.runOnce(() -> agitationAngle = agitationAngle.plus(agitationDelta)))
    //             .repeatedly()
    //             .until(() -> agitationAngle.getRadians() <=
    // agitationDelta.times(4).getRadians())),
    //     new ContinuousConditionalCommand(
    //             setRollerVoltage(6),
    //             setRollerVoltage(-2),
    //             () -> linkage.getPosition().getDegrees() > 60)
    //         .repeatedly());
    return Commands.parallel(
        Commands.sequence(
                Commands.runOnce(() -> agitationAngle = Rotation2d.fromDegrees(170)),
                linkage.setPositionGoal(
                    () -> agitationAngle,
                    () -> V1_DoomSpiralRobotState.getIntakeOffsets().getStowOffset()),
                linkage.waitUntilLinkageAtGoal(),
                linkage.setPositionGoal(
                    () -> Rotation2d.fromDegrees(90),
                    () -> V1_DoomSpiralRobotState.getIntakeOffsets().getStowOffset()),
                linkage.waitUntilLinkageAtGoal())
            .repeatedly(),
        setRollerVoltage(3.0));
  }

  public Command toggleIntake() {
    return Commands.either(
        Commands.sequence(stow(), setRollerVoltage(V1_DoomSpiralIntakeConstants.EXTAKE_VOLTAGE)),
        Commands.sequence(deploy(), setRollerVoltage(V1_DoomSpiralIntakeConstants.INTAKE_VOLTAGE)),
        () ->
            (intakeState.equals(IntakeState.INTAKE)
                && linkage.atGoal(
                    IntakeState.INTAKE
                        .getAngle()
                        .plus(V1_DoomSpiralRobotState.getIntakeOffsets().getCollectOffset()))));
  }

  public Command collect() {
    return Commands.parallel(
        deploy(), setRollerVoltage(V1_DoomSpiralIntakeConstants.INTAKE_VOLTAGE));
  }

  public Command resetIntakeZero() {
    return Commands.sequence(
        linkage.setPosition(V1_DoomSpiralIntakeConstants.MIN_ANGLE),
        linkage.setPositionGoal(V1_DoomSpiralIntakeConstants.MIN_ANGLE, Rotation2d::new));
  }

  public Transform3d getHopperWallTransform() {
    // 1. Calculate Current Pose
    final double currentY =
        linkage.getPosition().getSin() * V1_DoomSpiralIntakeConstants.PIN_LENGTH;
    final double currentX0 =
        linkage.getPosition().getCos() * V1_DoomSpiralIntakeConstants.PIN_LENGTH;
    final double currentXOff = calculateXOffset(currentY);
    Pose3d currentPose = new Pose3d(-(currentX0 + currentXOff), 0, 0, new Rotation3d(0, 0, 0));

    // 2. Calculate "Zero" Pose (at Y_MIN)
    final double zeroY = V1_DoomSpiralIntakeConstants.LINK_BOUNDS.MIN();
    final double zeroAngle = Math.asin(zeroY / V1_DoomSpiralIntakeConstants.PIN_LENGTH);
    final double zeroX0 = Math.cos(zeroAngle) * V1_DoomSpiralIntakeConstants.PIN_LENGTH;
    final double zeroXOff = calculateXOffset(zeroY);
    Pose3d zeroPose = new Pose3d(-(zeroX0 + zeroXOff), 0, 0, new Rotation3d(0, 0, 0));

    // 3. Return the Transform (Zero -> Current)
    return currentPose.minus(zeroPose);
  }

  /** Piecewise logic for the linkage offset */
  private double calculateXOffset(double yPos) {
    final double Y_MIN = V1_DoomSpiralIntakeConstants.LINK_BOUNDS.MIN();
    final double Y_PHASE_1 = V1_DoomSpiralIntakeConstants.LINK_BOUNDS.PHASE_1();
    final double Y_PHASE_2 = V1_DoomSpiralIntakeConstants.LINK_BOUNDS.PHASE_2();
    final double Y_MAX = V1_DoomSpiralIntakeConstants.LINK_BOUNDS.MAX();

    final double RADIUS_1 = V1_DoomSpiralIntakeConstants.LINK_CONST.RADIUS_1();
    final double RADIUS_2 = V1_DoomSpiralIntakeConstants.LINK_CONST.RADIUS_2();
    final double CENTER_OFFSET = V1_DoomSpiralIntakeConstants.LINK_CONST.CENTER_OFFSET();

    if (yPos <= Y_PHASE_1 && yPos >= Y_MIN) {
      return Math.sqrt(Math.pow(RADIUS_1, 2) - Math.pow(yPos, 2)) - CENTER_OFFSET;
    } else if (yPos <= Y_PHASE_2 && yPos > Y_PHASE_1) {
      return 0;
    } else if (yPos <= Y_MAX && yPos > Y_PHASE_2) {
      return Math.sqrt(Math.pow(RADIUS_2, 2) - Math.pow(yPos - Y_PHASE_2, 2)) - RADIUS_2;
    }
    return 0;
  }

  public boolean atGoal() {
    return linkage.atGoal();
  }

  public Command waitUntilIntakeAtGoal() {
    return linkage.waitUntilLinkageAtGoal();
  }

  public Command incrementStowOffset() {
    return Commands.sequence(
        Commands.runOnce(
            () ->
                V1_DoomSpiralRobotState.getIntakeOffsets()
                    .setStowOffset(
                        V1_DoomSpiralRobotState.getIntakeOffsets()
                            .getStowOffset()
                            .plus(V1_DoomSpiralIntakeConstants.LINKAGE_ANGLE_INCREMENT))),
        stow());
  }

  public Command decrementStowOffset() {
    return Commands.sequence(
        Commands.runOnce(
            () ->
                V1_DoomSpiralRobotState.getIntakeOffsets()
                    .setStowOffset(
                        V1_DoomSpiralRobotState.getIntakeOffsets()
                            .getStowOffset()
                            .minus(V1_DoomSpiralIntakeConstants.LINKAGE_ANGLE_INCREMENT))),
        stow());
  }

  public Command incrementBumpOffset() {
    return Commands.sequence(
        Commands.runOnce(
            () ->
                V1_DoomSpiralRobotState.getIntakeOffsets()
                    .setBumpOffset(
                        V1_DoomSpiralRobotState.getIntakeOffsets()
                            .getBumpOffset()
                            .plus(V1_DoomSpiralIntakeConstants.LINKAGE_ANGLE_INCREMENT))),
        bump());
  }

  public Command decrementBumpOffset() {
    return Commands.sequence(
        Commands.runOnce(
            () ->
                V1_DoomSpiralRobotState.getIntakeOffsets()
                    .setBumpOffset(
                        V1_DoomSpiralRobotState.getIntakeOffsets()
                            .getBumpOffset()
                            .minus(V1_DoomSpiralIntakeConstants.LINKAGE_ANGLE_INCREMENT))),
        bump());
  }

  public Command incrementCollectOffset() {
    return Commands.sequence(
        Commands.runOnce(
            () ->
                V1_DoomSpiralRobotState.getIntakeOffsets()
                    .setCollectOffset(
                        V1_DoomSpiralRobotState.getIntakeOffsets()
                            .getCollectOffset()
                            .plus(V1_DoomSpiralIntakeConstants.LINKAGE_ANGLE_INCREMENT))),
        deploy());
  }

  public Command decrementCollectOffset() {
    return Commands.sequence(
        Commands.runOnce(
            () ->
                V1_DoomSpiralRobotState.getIntakeOffsets()
                    .setCollectOffset(
                        V1_DoomSpiralRobotState.getIntakeOffsets()
                            .getCollectOffset()
                            .minus(V1_DoomSpiralIntakeConstants.LINKAGE_ANGLE_INCREMENT))),
        deploy());
  }

  public Command increaseSpeedOffset() {
    return Commands.runOnce(roller::incrementVoltageOffset);
  }

  public Command decreaseSpeedOffset() {
    return Commands.runOnce(roller::decrementVoltageOffset);
  }

  public Command defaultCommand() {
    Command defaultCommand =
        Commands.either(
            Commands.either(stopRoller(), Commands.none(), this::atGoal),
            Commands.none(),
            () -> (intakeState.equals(IntakeState.STOW)));
    defaultCommand.addRequirements(this);
    return defaultCommand;
  }

  public Command linkageSysId() {
    return linkage.runSysId();
  }
}
