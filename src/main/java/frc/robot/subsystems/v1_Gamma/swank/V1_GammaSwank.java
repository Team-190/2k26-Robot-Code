package frc.robot.subsystems.v1_Gamma.swank;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldConstants;
import frc.robot.subsystems.v1_Gamma.V1_GammaRobotState;
import org.littletonrobotics.junction.Logger;

public class V1_GammaSwank extends SubsystemBase {
  private final V1_GammaSwankIO io;
  private final V1_GammaSwankIOInputsAutoLogged inputs;

  private double voltageGoal;

  public V1_GammaSwank(V1_GammaSwankIO io) {
    this.io = io;
    this.inputs = new V1_GammaSwankIOInputsAutoLogged();
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Swank", inputs);
    io.setVoltage(voltageGoal);

    if (checkBumpPosition() && checkBumpAngle()) {
      io.setVoltage(getSwankVoltage());
    }
  }

  public Command setVoltage(double voltage) {
    return runOnce(() -> voltageGoal = voltage);
  }

  public Command stop() {
    return runOnce(() -> voltageGoal = 0);
  }

  public boolean checkBumpAngle() {

    boolean frontCheck =
        Math.abs(
                V1_GammaRobotState.getHeading()
                    .relativeTo(new Rotation2d(Math.PI / 2))
                    .getRadians())
            < Math.PI / 6;
    boolean backCheck =
        Math.abs(
                V1_GammaRobotState.getHeading()
                    .relativeTo(new Rotation2d(-Math.PI / 2))
                    .getRadians())
            < Math.PI / 6;

    return (frontCheck && backCheck);
  }

  public boolean checkBumpPosition() {
    boolean withinLeftBumpStart =
        V1_GammaRobotState.getGlobalPose().getY() < FieldConstants.LinesHorizontal.leftBumpStart;
    boolean withinLeftBumpEnd =
        V1_GammaRobotState.getGlobalPose().getY() > FieldConstants.LinesHorizontal.leftBumpEnd;
    boolean withinRightBumpStart =
        V1_GammaRobotState.getGlobalPose().getY() < FieldConstants.LinesHorizontal.rightBumpStart;
    boolean withinRightBumpEnd =
        V1_GammaRobotState.getGlobalPose().getY() > FieldConstants.LinesHorizontal.rightBumpEnd;
    boolean withinRange =
        (Math.abs(
                    V1_GammaRobotState.getGlobalPose().getX()
                        - FieldConstants.LinesVertical.hubCenter)
                < 1
            || Math.abs(
                    V1_GammaRobotState.getGlobalPose().getY()
                        - FieldConstants.LinesVertical.oppHubCenter)
                < 1);

    return ((withinLeftBumpStart && withinLeftBumpEnd)
        || (withinRightBumpStart && withinRightBumpEnd) && withinRange);
  }

  public double getSwankVoltage() {
    boolean isTurnedLeft = V1_GammaRobotState.getHeading().getRadians() > Math.PI / 2;
    boolean onOpponentSide =
        V1_GammaRobotState.getGlobalPose().getX() > FieldConstants.LinesVertical.center;
    boolean isRobotSideLeft =
        V1_GammaRobotState.getGlobalPose().getX() < FieldConstants.LinesVertical.hubCenter;
    if (onOpponentSide) {
      isTurnedLeft = !isTurnedLeft;
      isRobotSideLeft =
          V1_GammaRobotState.getGlobalPose().getX() < FieldConstants.LinesVertical.oppHubCenter;
    }
    if (isRobotSideLeft ^ isTurnedLeft) {
      return -12.0;
    } else {
      return 12.0;
    }
  }
}
