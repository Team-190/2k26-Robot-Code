package frc.robot.subsystems.shared.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.team190.gompeilib.core.GompeiLib;
import edu.wpi.team190.gompeilib.core.logging.Trace;
import edu.wpi.team190.gompeilib.subsystems.generic.roller.GenericRoller;
import edu.wpi.team190.gompeilib.subsystems.generic.roller.GenericRollerIO;
import frc.robot.subsystems.shared.fourbarlinkage.FourBarLinkage;
import frc.robot.subsystems.shared.fourbarlinkage.FourBarLinkageIO;
import frc.robot.subsystems.shared.intake.IntakeState;
import frc.robot.subsystems.v1_DoomSpiral.V1_DoomSpiralRobotState;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final GenericRoller roller;
  @Getter private final FourBarLinkage linkage;

  @Getter private IntakeState intakeState;
  private boolean intakeOnInIntakeState;
  private double overrideVoltage;
  private boolean overrideRoller;

  public Intake(GenericRollerIO rollerIO, FourBarLinkageIO linkageIO) {
    setName("Intake");

    intakeState = IntakeState.STOW;

    roller = new GenericRoller(rollerIO, this, IntakeConstants.INTAKE_ROLLER_CONSTANTS_TOP, "");
    linkage =
        new FourBarLinkage(
            linkageIO, IntakeConstants.LINKAGE_CONSTANTS, this, "", IntakeState.STOW.getAngle());

    intakeState = IntakeState.STOW;

    // setDefaultCommand(defaultCommand());
  }

  @Trace
  @Override
  public void periodic() {
    roller.periodic();
    linkage.periodic();

    if (overrideRoller) roller.setVoltage(Volts.of(overrideVoltage));
    else
      switch (intakeState) {
        case STOW:
          if (!linkage.atPositionGoal())
            roller.setVoltage(Volts.of(IntakeConstants.EXTAKE_VOLTAGE));
          else roller.setVoltage(Volts.of(0.0));
          break;
        case INTAKE:
        case BUMP:
          if (intakeOnInIntakeState)
            roller.setVoltage(Volts.of(IntakeConstants.INTAKE_VOLTAGE));
          else roller.setVoltage(Volts.of(0.0));
          break;
        case AGITATE:
          roller.setVoltage(Volts.of(3.0));
          break;
        default:
          break;
      }

    Logger.recordOutput("Intake/Intake State", intakeState);

    V1_DoomSpiralRobotState.getLedStates().setIntakeIn(false);
    if (intakeState.equals(IntakeState.INTAKE) || intakeState.equals(IntakeState.BUMP)) {
      V1_DoomSpiralRobotState.getLedStates()
          .setIntakeCollecting(
              roller.getVoltageGoalVolts().getNewSetpoint().baseUnitMagnitude()
                  == IntakeConstants.INTAKE_VOLTAGE);
      V1_DoomSpiralRobotState.getLedStates().setIntakeIn(false);
    } else {
      V1_DoomSpiralRobotState.getLedStates().setIntakeIn(true);
      V1_DoomSpiralRobotState.getLedStates().setIntakeCollecting(false);
    }
    V1_DoomSpiralRobotState.getLedStates()
        .setSpitting(
            roller.getVoltageGoalVolts().getSetpoint().baseUnitMagnitude()
                == IntakeConstants.EXTAKE_VOLTAGE);

    Logger.recordOutput(
        "Intake/Linkage0/Offset Degrees", linkage.getPosition().getDegrees());
    Logger.recordOutput(
        "Intake/Linkage0/Angle Degrees", linkage.getPosition().getDegrees());
    Logger.recordOutput(
        "Intake/Roller/Voltage Offset", roller.getVoltageGoalVolts().getOffset().in(Volts));
    Logger.recordOutput(
        "Intake/Roller/AppliedVolts", roller.getVoltageGoalVolts().getSetpoint().in(Volts));
    Logger.recordOutput(
        "Intake/Roller/Voltage Magnitude", roller.getVoltageGoalVolts().getNewSetpoint().in(Volts));
  }

  /**
   * Sets the voltage of the top and bottom rollers of the intake subsystem. The voltage is offset
   * by the roller voltage offset stored in the robot state.
   *
   * @param voltage the voltage to set the rollers to
   * @return a command that sets the voltage of the top and bottom rollers
   */
  public Command setOverrideRollerVoltage(double voltage) {
    return Commands.runOnce(
        () -> {
          overrideVoltage = voltage;
          overrideRoller = true;
        });
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
    return Commands.runOnce(
        () -> {
          intakeOnInIntakeState = false;
          overrideRoller = false;
        });
  }

  public Command deploy() {
    return Commands.sequence(
        Commands.runOnce(
            () -> {
              intakeState = IntakeState.INTAKE;
              intakeOnInIntakeState = true;
              linkage.setPositionGoal(IntakeState.INTAKE.getAngle());
            }));
  }

  public Command stow() {
    return Commands.runOnce(
        () -> {
          intakeState = IntakeState.STOW;
          intakeOnInIntakeState = true;
          linkage.setPositionGoal(IntakeState.STOW.getAngle());
        });
  }

  public Command bump() {
    return Commands.runOnce(
        () -> {
          intakeState = IntakeState.BUMP;
          intakeOnInIntakeState = true;
          linkage.setPositionGoal(IntakeState.BUMP.getAngle());
        });
  }

  public Command agitate() {
    return Commands.sequence(
            Commands.runOnce(
                () -> {
                  intakeState = IntakeState.AGITATE;
                  linkage.setPositionGoal(IntakeState.AGITATE.getAngle());
                  linkage.setPositionGoal(IntakeState.AGITATE.getAngle());
                }),
            linkage.waitUntilLinkageAtGoal(),
            Commands.runOnce(
                () -> {
                  linkage.setPositionGoal(IntakeState.AGITATE.getAngle());
                  linkage.setPositionGoal(Rotation2d.fromDegrees(90 + 8.0));
                }),
            linkage.waitUntilLinkageAtGoal())
        .repeatedly();
  }

  public Command collect() {
    return deploy();
  }

  public Command stopCollect() {
    return stow();
  }

  public Command resetIntakeZero() {
    return Commands.runOnce(
        () -> {
          linkage.setPosition(IntakeConstants.MIN_ANGLE);
          linkage.setPositionGoal(IntakeConstants.MIN_ANGLE);
        });
  }

  public Transform3d getHopperWallTransform() {
    // 1. Calculate Current Pose
    final double currentY = linkage.getPosition().getSin() * IntakeConstants.LINK_LENGTHS.AB();
    final double currentX0 = linkage.getPosition().getCos() * IntakeConstants.LINK_LENGTHS.BC();
    final double currentXOff = calculateXOffset(currentY);
    Pose3d currentPose = new Pose3d(-(currentX0 + currentXOff), 0, 0, new Rotation3d(0, 0, 0));

    // 2. Calculate "Zero" Pose (at Y_MIN)
    final double zeroY = IntakeConstants.LINK_BOUNDS.MIN();
    final double zeroAngle = Math.asin(zeroY / IntakeConstants.LINK_LENGTHS.CD());
    final double zeroX0 = Math.cos(zeroAngle) * IntakeConstants.LINK_LENGTHS.DA();
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

  public boolean atGoal() {
    return linkage.atPositionGoal();
  }

  public Command waitUntilIntakeAtGoal() {
    return Commands.waitSeconds(GompeiLib.getLoopPeriod())
        .andThen(linkage.waitUntilLinkageAtGoal());
  }

  // public Command incrementStowOffset() {
  //   return Commands.sequence(
  //       Commands.runOnce(() -> IntakeState.STOW.getAngle().increment()), stow());
  // }

  // public Command decrementStowOffset() {
  //   return Commands.sequence(
  //       Commands.runOnce(() -> IntakeState.STOW.getAngle().decrement()), stow());
  // }

  // public Command incrementBumpOffset() {
  //   return Commands.sequence(
  //       Commands.runOnce(() -> IntakeState.BUMP.getAngle().increment()), bump());
  // }

  // public Command decrementBumpOffset() {
  //   return Commands.sequence(
  //       Commands.runOnce(() -> IntakeState.BUMP.getAngle().decrement()), bump());
  // }

  // public Command incrementCollectOffset() {
  //   return Commands.sequence(
  //       Commands.runOnce(() -> IntakeState.INTAKE.getAngle().increment()), deploy());
  // }

  // public Command decrementCollectOffset() {
  //   return Commands.sequence(
  //       Commands.runOnce(() -> IntakeState.INTAKE.getAngle().decrement()), deploy());
  // }

  public Command increaseSpeedOffset() {
    return Commands.runOnce(roller.getVoltageGoalVolts()::increment);
  }

  public Command decreaseSpeedOffset() {
    return Commands.runOnce(roller.getVoltageGoalVolts()::decrement);
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
