package frc.robot.subsystems.v2_Delta.clopper;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.team190.gompeilib.core.utility.Setpoint;
import edu.wpi.team190.gompeilib.subsystems.generic.roller.GenericRoller;
import edu.wpi.team190.gompeilib.subsystems.generic.roller.GenericRollerIO;
import frc.robot.util.command.ContinuousConditionalCommand;
import org.littletonrobotics.junction.Logger;

public class V2_DeltaClopper extends SubsystemBase {
  private final GenericRoller rollerFloor;
  private final GenericRoller ballTunnel;
  private final GenericRoller ballsToWall;

  private boolean overrideBallTunnel;
  private boolean overrideRollerFloor;
  private boolean overrideBallsToWall;

  private final Setpoint<VoltageUnit> ballTunnelSetpoint, ballTunnelOverrideSetpoint;
  private final Setpoint<VoltageUnit> rollerFloorSetpoint, rollerFloorOverrideSetpoint;
  private final Setpoint<VoltageUnit> ballsToWallSetpoint, ballsToWallsverrideSetpoint;

  private final Trigger ballToWallCurrentTrigger;
  private final Trigger rollerFloorCurrentTrigger;
  private boolean shouldReverse;

  public V2_DeltaClopper(
      GenericRollerIO rollerFloorIO, GenericRollerIO ballTunnelIO, GenericRollerIO ballsToWallIO) {
    setName("Hopper");

    rollerFloorSetpoint =
        new Setpoint<>(
            Volts.of(0),
            V2_DeltaClopperConstants.ROLLER_FLOOR_CONSTANTS.voltageOffsetStep,
            Volts.of(-12),
            Volts.of(12));
    rollerFloorOverrideSetpoint =
        new Setpoint<>(
            Volts.of(0),
            V2_DeltaClopperConstants.ROLLER_FLOOR_CONSTANTS.voltageOffsetStep,
            Volts.of(-12),
            Volts.of(12));

    ballTunnelSetpoint =
        new Setpoint<>(
            Volts.of(0),
            V2_DeltaClopperConstants.BALL_TUNNEL_CONSTANTS.voltageOffsetStep,
            Volts.of(-12),
            Volts.of(12));
    ballTunnelOverrideSetpoint =
        new Setpoint<>(
            Volts.of(0),
            V2_DeltaClopperConstants.BALL_TUNNEL_CONSTANTS.voltageOffsetStep,
            Volts.of(-12),
            Volts.of(12));

    ballsToWallSetpoint =
        new Setpoint<>(
            Volts.of(0),
            V2_DeltaClopperConstants.BALLS_TO_THE_WALL_CONSTANTS.voltageOffsetStep,
            Volts.of(-12),
            Volts.of(12));
    ballsToWallsverrideSetpoint =
        new Setpoint<>(
            Volts.of(0),
            V2_DeltaClopperConstants.BALLS_TO_THE_WALL_CONSTANTS.voltageOffsetStep,
            Volts.of(-12),
            Volts.of(12));

    rollerFloor =
        new GenericRoller(
            rollerFloorIO,
            this,
            V2_DeltaClopperConstants.ROLLER_FLOOR_CONSTANTS,
            " Floor",
            rollerFloorSetpoint);
    ballTunnel =
        new GenericRoller(
            ballTunnelIO,
            this,
            V2_DeltaClopperConstants.BALL_TUNNEL_CONSTANTS,
            " Tunnel",
            ballTunnelSetpoint);
    ballsToWall =
        new GenericRoller(
            ballsToWallIO,
            this,
            V2_DeltaClopperConstants.BALLS_TO_THE_WALL_CONSTANTS,
            " Side",
            ballsToWallSetpoint);

    overrideBallTunnel = false;
    overrideRollerFloor = false;
    overrideBallsToWall = false;

    shouldReverse = false;
    ballToWallCurrentTrigger =
        new Trigger(
                () ->
                    ballsToWall.getTorqueCurrent().length > 0
                        && ballsToWall.getTorqueCurrent()[0]
                            >= V2_DeltaClopperConstants.BALLS_TO_WALL_CURRENT_THRESHOLD
                        && Math.abs(ballTunnel.getVoltageGoal().getSetpoint().in(Volts)) > 0)
            .debounce(0.1)
            .debounce(3, Debouncer.DebounceType.kFalling);
    ballToWallCurrentTrigger.onTrue(
        Commands.run(() -> shouldReverse = true)
            .withTimeout(1)
            .andThen(Commands.runOnce(() -> shouldReverse = false)));
    rollerFloorCurrentTrigger =
        new Trigger(
                () ->
                    rollerFloor.getTorqueCurrent().length > 0
                        && rollerFloor.getTorqueCurrent()[0] >= 25
                        && rollerFloor.getVoltageGoal().getSetpoint().in(Volts) > 0)
            .debounce(0.125);
  }

  @Override
  public void periodic() {
    if (!overrideBallTunnel) {
      ballTunnel.setVoltageGoal(ballTunnelSetpoint);
    }
    if (!overrideRollerFloor) {
      rollerFloor.setVoltageGoal(rollerFloorSetpoint);
    }
    if (!overrideBallsToWall) {
      ballsToWall.setVoltageGoal(ballsToWallSetpoint);
      if (shouldReverse) {
        ballsToWall.setVoltageGoal(V2_DeltaClopperConstants.BALLS_TO_THE_WALL_REVERSE_VOLTAGE);
      } else if (Math.abs(ballTunnel.getVoltageGoal().getSetpoint().in(Volts)) > 0) {
        ballsToWall.setVoltageGoal(V2_DeltaClopperConstants.BALLS_TO_THE_WALL_FORWARD_VOLTAGE);
      } else ballsToWall.setVoltageGoal(Volts.of(0));
    }
    rollerFloor.periodic();
    ballTunnel.periodic();
    ballsToWall.periodic();

    Logger.recordOutput(
        "Elastic/Hopper/RollerFloor/Voltage Magnitude",
        String.format("%.1f", Math.abs(rollerFloor.getVoltageGoal().getSetpoint().in(Volts))));
    Logger.recordOutput(
        "Elastic/Hopper/RollerFloor/Voltage Offset",
        String.format("%.1f", rollerFloor.getVoltageGoal().getOffset().in(Volts)));

    Logger.recordOutput(
        "Elastic/Hopper/BallTunnel/Voltage Magnitude",
        String.format("%.1f", Math.abs(ballTunnel.getVoltageGoal().getSetpoint().in(Volts))));
    Logger.recordOutput(
        "Elastic/Hopper/BallTunnel/Voltage Offset",
        String.format("%.1f", ballTunnel.getVoltageGoal().getOffset().in(Volts)));
  }

  public Command setRollerFloorVoltage(Voltage voltage) {
    return Commands.runOnce(() -> rollerFloorSetpoint.setSetpoint(voltage));
  }

  public Command feedShooterRollerFloor() {
    return new ContinuousConditionalCommand(
        Commands.sequence(
                setRollerFloorVoltage(Volts.of(-12)),
                Commands.waitSeconds(.125),
                setRollerFloorVoltage(V2_DeltaClopperConstants.ROLLER_FLOOR_FEED_VOLTAGE))
            .alongWith(Commands.idle()),
        setRollerFloorVoltage(V2_DeltaClopperConstants.ROLLER_FLOOR_FEED_VOLTAGE)
            .alongWith(Commands.idle()),
        rollerFloorCurrentTrigger);
  }

  public Command setOverrideRollerFloorVoltage(Voltage voltage) {
    return Commands.startEnd(
        () -> {
          overrideRollerFloor = true;
          rollerFloor.setVoltageGoal(rollerFloorOverrideSetpoint);
          rollerFloor.setVoltageGoal(voltage);
        },
        () -> {
          overrideRollerFloor = false;
        },
        this);
  }

  public Command incrementRollerFloorVelocity() {
    return Commands.runOnce(rollerFloorSetpoint::increment);
  }

  public Command decrementRollerFloorVelocity() {
    return Commands.runOnce(rollerFloorSetpoint::decrement);
  }

  public Command stopRollerFloor() {
    return setRollerFloorVoltage(Volts.of(0.0));
  }

  public Command setBallTunnelVoltage(Voltage voltage) {
    return Commands.runOnce(() -> ballTunnelSetpoint.setSetpoint(voltage));
  }

  public Command setOverrideBallTunnelVoltage(Voltage voltage) {
    return Commands.startEnd(
        () -> {
          overrideBallTunnel = true;
          ballTunnel.setVoltageGoal(ballTunnelOverrideSetpoint);
          ballTunnel.setVoltageGoal(voltage);
        },
        () -> {
          overrideBallTunnel = false;
        },
        this);
  }

  public Command setOverrideBallsToWallVoltage(Voltage voltage) {
    return this.startEnd(
        () -> {
          overrideBallsToWall = true;
          ballsToWall.setVoltageGoal(ballsToWallsverrideSetpoint);
          ballsToWall.setVoltageGoal(voltage);
        },
        () -> {
          overrideBallsToWall = false;
        });
  }

  public Command incrementBallsToWallVelocity() {
    return Commands.runOnce(ballsToWallSetpoint::increment);
  }

  public Command decrementBallsToWallVelocity() {
    return Commands.runOnce(ballsToWallSetpoint::decrement);
  }

  public Command incrementBallTunnelVelocity() {
    return Commands.runOnce(ballTunnelSetpoint::increment);
  }

  public Command decrementBallTunnelVelocity() {
    return Commands.runOnce(ballTunnelSetpoint::decrement);
  }

  public Command feedShooterBallTunnel() {
    return setBallTunnelVoltage(V2_DeltaClopperConstants.BALL_TUNNEL_FEED_VOLTAGE);
  }

  public Command stopBallTunnel() {
    return setBallTunnelVoltage(Volts.of(0.0));
  }

  public Command intake() {
    return Commands.parallel(feedShooterRollerFloor(), feedShooterBallTunnel());
  }
}
