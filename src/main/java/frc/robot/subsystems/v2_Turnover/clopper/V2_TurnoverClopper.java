package frc.robot.subsystems.v2_Turnover.clopper;

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
import org.littletonrobotics.junction.Logger;

public class V2_TurnoverClopper extends SubsystemBase {
  private final GenericRoller rollerFloor;
  private final GenericRoller ballTunnelTop;
  private final GenericRoller ballTunnelBottom;
  private final GenericRoller ballsToWall;

  private boolean overrideTopBallTunnel;
  private boolean overrideBottomBallTunnel;
  private boolean overrideRollerFloor;
  private boolean overrideBallsToWall;

  private final Setpoint<VoltageUnit> ballTunnelTopSetpoint, ballTunnelBottomSetpoint, ballTunnelTopOverrideSetpoint, ballTunnelBottomOverrideSetpoint;
  private final Setpoint<VoltageUnit> rollerFloorSetpoint, rollerFloorOverrideSetpoint;
  private final Setpoint<VoltageUnit> ballsToWallSetpoint, ballsToWallsverrideSetpoint;

  private final Trigger ballToWallCurrentTrigger;
  private final Trigger rollerFloorCurrentTrigger;
  private boolean shouldReverse;

  public V2_TurnoverClopper(
      GenericRollerIO rollerFloorIO, GenericRollerIO ballTunnelTopIO, GenericRollerIO ballTunnelBottomIO, GenericRollerIO ballsToWallIO) {
    setName("Hopper");

    rollerFloorSetpoint =
        new Setpoint<>(
            Volts.of(0),
            V2_TurnoverClopperConstants.ROLLER_FLOOR_CONSTANTS.voltageOffsetStep,
            Volts.of(-12),
            Volts.of(12));
    rollerFloorOverrideSetpoint =
        new Setpoint<>(
            Volts.of(0),
            V2_TurnoverClopperConstants.ROLLER_FLOOR_CONSTANTS.voltageOffsetStep,
            Volts.of(-12),
            Volts.of(12));

    ballTunnelTopSetpoint =
        new Setpoint<>(
            Volts.of(0),
            V2_TurnoverClopperConstants.BALL_TUNNEL_TOP_CONSTANTS.voltageOffsetStep,
            Volts.of(-12),
            Volts.of(12));

    ballTunnelBottomSetpoint =
        new Setpoint<>(
            Volts.of(0),
            V2_TurnoverClopperConstants.BALL_TUNNEL_BOTTOM_CONSTANTS.voltageOffsetStep,
            Volts.of(-12),
            Volts.of(12));
    
    ballTunnelTopOverrideSetpoint =
        new Setpoint<>(
            Volts.of(0),
            V2_TurnoverClopperConstants.BALL_TUNNEL_TOP_CONSTANTS.voltageOffsetStep,
            Volts.of(-12),
            Volts.of(12));

    ballTunnelBottomOverrideSetpoint =
        new Setpoint<>(
            Volts.of(0),
            V2_TurnoverClopperConstants.BALL_TUNNEL_BOTTOM_CONSTANTS.voltageOffsetStep,
            Volts.of(-12),
            Volts.of(12));

    ballsToWallSetpoint =
        new Setpoint<>(
            Volts.of(0),
            V2_TurnoverClopperConstants.BALLS_TO_THE_WALL_CONSTANTS.voltageOffsetStep,
            Volts.of(-12),
            Volts.of(12));
    ballsToWallsverrideSetpoint =
        new Setpoint<>(
            Volts.of(0),
            V2_TurnoverClopperConstants.BALLS_TO_THE_WALL_CONSTANTS.voltageOffsetStep,
            Volts.of(-12),
            Volts.of(12));

    rollerFloor =
        new GenericRoller(
            rollerFloorIO,
            this,
            V2_TurnoverClopperConstants.ROLLER_FLOOR_CONSTANTS,
            " Floor",
            rollerFloorSetpoint);
    ballTunnelTop =
        new GenericRoller(
            ballTunnelTopIO,
            this,
            V2_TurnoverClopperConstants.BALL_TUNNEL_TOP_CONSTANTS,
            " Tunnel",
            ballTunnelTopSetpoint);
    ballTunnelBottom =
        new GenericRoller(
            ballTunnelBottomIO,
            this,
            V2_TurnoverClopperConstants.BALL_TUNNEL_BOTTOM_CONSTANTS,
            " Tunnel",
            ballTunnelBottomSetpoint);
    ballsToWall =
        new GenericRoller(
            ballsToWallIO,
            this,
            V2_TurnoverClopperConstants.BALLS_TO_THE_WALL_CONSTANTS,
            " Side",
            ballsToWallSetpoint);

    overrideTopBallTunnel = false;
    overrideBottomBallTunnel = false;
    overrideRollerFloor = false;
    overrideBallsToWall = false;

    shouldReverse = false;
    ballToWallCurrentTrigger =
        new Trigger(
                () ->
                    ballsToWall.getTorqueCurrent().length > 0
                        && ballsToWall.getTorqueCurrent()[0]
                            >= V2_TurnoverClopperConstants.BALLS_TO_WALL_CURRENT_THRESHOLD
                        && Math.abs(ballTunnelTop.getVoltageGoal().getSetpoint().in(Volts)) > 0
                        && Math.abs(ballTunnelBottom.getVoltageGoal().getSetpoint().in(Volts)) > 0)

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
    if (!overrideTopBallTunnel) {
      ballTunnelTop.setVoltageGoal(ballTunnelTopSetpoint);
    }
    if (!overrideBottomBallTunnel) {
      ballTunnelBottom.setVoltageGoal(ballTunnelBottomSetpoint);
    }
    if (!overrideRollerFloor) {
      rollerFloor.setVoltageGoal(rollerFloorSetpoint);
    }
    if (!overrideBallsToWall) {
      ballsToWall.setVoltageGoal(ballsToWallSetpoint);
      if (shouldReverse) {
        ballsToWall.setVoltageGoal(V2_TurnoverClopperConstants.BALLS_TO_THE_WALL_REVERSE_VOLTAGE);
      } else if (Math.abs(ballTunnelTop.getVoltageGoal().getSetpoint().in(Volts)) > 0 && Math.abs(ballTunnelBottom.getVoltageGoal().getSetpoint().in(Volts)) > 0) {
        ballsToWall.setVoltageGoal(V2_TurnoverClopperConstants.BALLS_TO_THE_WALL_FORWARD_VOLTAGE);
      } else ballsToWall.setVoltageGoal(Volts.of(0));
    }
    rollerFloor.periodic();
    ballTunnelTop.periodic();
    ballTunnelBottom.periodic();
    ballsToWall.periodic();

    Logger.recordOutput(
        "Elastic/Hopper/RollerFloor/Voltage Magnitude",
        String.format("%.1f", Math.abs(rollerFloor.getVoltageGoal().getSetpoint().in(Volts))));
    Logger.recordOutput(
        "Elastic/Hopper/RollerFloor/Voltage Offset",
        String.format("%.1f", rollerFloor.getVoltageGoal().getOffset().in(Volts)));

    Logger.recordOutput(
        "Elastic/Hopper/BallTunnelTop/Voltage Magnitude",
        String.format("%.1f", Math.abs(ballTunnelTop.getVoltageGoal().getSetpoint().in(Volts))));
    Logger.recordOutput(
        "Elastic/Hopper/BallTunnelTop/Voltage Offset",
        String.format("%.1f", ballTunnelTop.getVoltageGoal().getOffset().in(Volts)));

    Logger.recordOutput(
        "Elastic/Hopper/BallTunnelBottom/Voltage Magnitude",
        String.format("%.1f", Math.abs(ballTunnelBottom.getVoltageGoal().getSetpoint().in(Volts))));
    Logger.recordOutput(
        "Elastic/Hopper/BallTunnelBottom/Voltage Offset",
        String.format("%.1f", ballTunnelBottom.getVoltageGoal().getOffset().in(Volts)));
  }


  public Command setRollerFloorVoltage(Voltage voltage) {
    return Commands.runOnce(() -> rollerFloorSetpoint.setSetpoint(voltage));
  }

  public Command feedShooterRollerFloor() {
    return setRollerFloorVoltage(V2_TurnoverClopperConstants.ROLLER_FLOOR_FEED_VOLTAGE);
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
    return Commands.runOnce(() -> {ballTunnelTopSetpoint.setSetpoint(voltage); ballTunnelBottomSetpoint.setSetpoint(voltage);});
  }

  public Command setBallTunnelVoltage(Voltage voltage, Voltage voltage2) {
    return Commands.runOnce(() -> {ballTunnelTopSetpoint.setSetpoint(voltage); ballTunnelBottomSetpoint.setSetpoint(voltage2);});
  }

  public Command setOverrideBallTunnelVoltage(Voltage voltage) {
    return Commands.startEnd(
        () -> {
          overrideTopBallTunnel = true;
          overrideBottomBallTunnel = true;
          ballTunnelTop.setVoltageGoal(ballTunnelTopOverrideSetpoint);
          ballTunnelTop.setVoltageGoal(voltage);
          ballTunnelBottom.setVoltageGoal(ballTunnelBottomOverrideSetpoint);
          ballTunnelBottom.setVoltageGoal(voltage);
        },
        () -> {
          overrideTopBallTunnel = false;
          overrideBottomBallTunnel = false;
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
    return Commands.runOnce(() -> {ballTunnelTopSetpoint.increment(); ballTunnelBottomSetpoint.increment();});
  }

  public Command decrementBallTunnelVelocity() {
    return Commands.runOnce(() -> {ballTunnelTopSetpoint.decrement(); ballTunnelBottomSetpoint.decrement();});
  }

  public Command feedShooterBallTunnel() {
    return setBallTunnelVoltage(V2_TurnoverClopperConstants.BALL_TUNNEL_FEED_VOLTAGE);
  }

  public Command stopBallTunnel() {
    return setBallTunnelVoltage(Volts.of(0.0));
  }

  public Command marcusCommand() {
    return setBallTunnelVoltage(Volts.of(V2_TurnoverClopperConstants.BALL_TUNNEL_FEED_VOLTAGE.in(Volts)), Volts.of(-V2_TurnoverClopperConstants.ROLLER_FLOOR_FEED_VOLTAGE.in(Volts)));
  }

  public Command intake() {
    return Commands.parallel(feedShooterRollerFloor(), feedShooterBallTunnel());
  }
}
