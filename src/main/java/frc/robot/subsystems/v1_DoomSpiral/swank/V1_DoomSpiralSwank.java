package frc.robot.subsystems.v1_DoomSpiral.swank;

import static frc.robot.subsystems.v1_DoomSpiral.swank.V1_DoomSpiralSwankConstants.SWANK_VOLTAGE;
import static frc.robot.subsystems.v1_DoomSpiral.swank.V1_DoomSpiralSwankState.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldConstants;
import frc.robot.subsystems.v1_DoomSpiral.V1_DoomSpiralRobotState;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class V1_DoomSpiralSwank extends SubsystemBase {
  private final V1_DoomSpiralSwankIO io;
  private final V1_DoomSpiralSwankIOInputsAutoLogged inputs;

  private double voltageGoal;

  @AutoLogOutput(key = "Swank/Current State")
  private V1_DoomSpiralSwankState currentState;

  public V1_DoomSpiralSwank(V1_DoomSpiralSwankIO io) {
    this.io = io;
    this.inputs = new V1_DoomSpiralSwankIOInputsAutoLogged();

    voltageGoal = 0;
    currentState = IDLE;

    setDefaultCommand(runOnce(() -> currentState = OPEN_LOOP_AUTO_ENABLE));
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Swank", inputs);

    switch (currentState) {
      case OPEN_LOOP_VOLTAGE_CONTROL:
        io.setVoltage(voltageGoal);
        break;
      case OPEN_LOOP_AUTO_ENABLE:
        if (checkBumpPosition() && checkBumpAngle()) {
          io.setVoltage(getSwankVoltage());
        }
        break;
      case IDLE:
        break;
    }
  }

  public Command setVoltage(double voltage) {
    return run(
        () -> {
          currentState = OPEN_LOOP_VOLTAGE_CONTROL;
          voltageGoal = voltage;
        });
  }

  public Command stop() {
    return run(
        () -> {
          currentState = IDLE;
          voltageGoal = 0;
        });
  }

  @AutoLogOutput(key = "Swank/BumpAngle")
  public boolean checkBumpAngle() {

    boolean frontCheck =
        Math.abs(
                V1_DoomSpiralRobotState.getHeading()
                    .relativeTo(new Rotation2d(Math.PI / 2))
                    .getRadians())
            < Math.PI / 6;
    boolean backCheck =
        Math.abs(
                V1_DoomSpiralRobotState.getHeading()
                    .relativeTo(new Rotation2d(-Math.PI / 2))
                    .getRadians())
            < Math.PI / 6;

    return (frontCheck && backCheck);
  }

  @AutoLogOutput(key = "Swank/BumpPosition")
  public boolean checkBumpPosition() {
    boolean withinLeftBumpStart =
        V1_DoomSpiralRobotState.getGlobalPose().getY()
            < FieldConstants.LinesHorizontal.leftBumpStart;
    boolean withinLeftBumpEnd =
        V1_DoomSpiralRobotState.getGlobalPose().getY() > FieldConstants.LinesHorizontal.leftBumpEnd;
    boolean withinRightBumpStart =
        V1_DoomSpiralRobotState.getGlobalPose().getY()
            < FieldConstants.LinesHorizontal.rightBumpStart;
    boolean withinRightBumpEnd =
        V1_DoomSpiralRobotState.getGlobalPose().getY()
            > FieldConstants.LinesHorizontal.rightBumpEnd;
    boolean withinRange =
        (Math.abs(
                    V1_DoomSpiralRobotState.getGlobalPose().getX()
                        - FieldConstants.LinesVertical.hubCenter)
                < 1
            || Math.abs(
                    V1_DoomSpiralRobotState.getGlobalPose().getY()
                        - FieldConstants.LinesVertical.oppHubCenter)
                < 1);

    return ((withinLeftBumpStart && withinLeftBumpEnd)
        || (withinRightBumpStart && withinRightBumpEnd) && withinRange);
  }

  public double getSwankVoltage() {
    boolean isTurnedLeft = V1_DoomSpiralRobotState.getHeading().getRadians() > Math.PI / 2;
    boolean onOpponentSide =
        V1_DoomSpiralRobotState.getGlobalPose().getX() > FieldConstants.LinesVertical.center;
    boolean isRobotSideLeft =
        V1_DoomSpiralRobotState.getGlobalPose().getX() < FieldConstants.LinesVertical.hubCenter;
    if (onOpponentSide) {
      isTurnedLeft = !isTurnedLeft;
      isRobotSideLeft =
          V1_DoomSpiralRobotState.getGlobalPose().getX()
              < FieldConstants.LinesVertical.oppHubCenter;
    }
    if (isRobotSideLeft ^ isTurnedLeft) {
      return -SWANK_VOLTAGE;
    } else {
      return SWANK_VOLTAGE;
    }
  }
}
