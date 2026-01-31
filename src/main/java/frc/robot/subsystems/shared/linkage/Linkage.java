package frc.robot.subsystems.shared.linkage;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.team190.gompeilib.core.GompeiLib;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class Linkage {
  public final LinkageIO io;
  public final String aKitTopic;
  private final LinkageIOInputsAutoLogged inputs;

  private LinkageState currentState;
  private LinkageState.Output currentOutput;

  private final SysIdRoutine characterizationRoutine;

  public Linkage(LinkageIO io, Subsystem subsystem, int index) {

    inputs = new LinkageIOInputsAutoLogged();
    this.io = io;

    aKitTopic = subsystem.getName() + "/Linkage" + index;

    characterizationRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(0.5).per(Seconds),
                Volts.of(3.5),
                Seconds.of(10),
                (state) -> Logger.recordOutput(aKitTopic + "/sysIDState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> io.setVoltage(voltage.in(Volts)), null, subsystem));
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(aKitTopic, inputs);

    Logger.recordOutput(aKitTopic + "/At Goal", atGoal());

    switch (currentState) {
      case OPEN_LOOP_VOLTAGE_CONTROL -> {
        io.setVoltage(currentOutput.volts());
      }
      case CLOSED_LOOP_POSITION_CONTROL -> {
        io.setPositionGoal(currentOutput.position());
      }
      default -> {}
    }
  }

  /**
   * Sets the voltage being passed into the linkage subsystem.
   *
   * @param volts the voltage passed into the linkage.
   * @return A command that sets the specified voltage.
   */
  public Command setVoltage(double volts) {
    return Commands.runOnce(
        () -> {
          currentState = LinkageState.OPEN_LOOP_VOLTAGE_CONTROL;
          currentState.set(volts);
        });
  }

  public Command setPositionGoal(Rotation2d position) {
    return Commands.runOnce(
        () -> {
          currentState = LinkageState.CLOSED_LOOP_POSITION_CONTROL;
          currentState.set(position);
        });
  }

  /**
   * Checks if the linkage is at the goal position.
   *
   * @return If the linkage is within tolerance of the goal (true) or not (false).
   */
  public boolean atGoal() {
    return io.atGoal();
  }

  /**
   * Waits until linkage at goal/in tolerance.
   *
   * @return A command that waits until the linkage is at the goal.
   */
  public Command waitUntilLinkageAtGoal() {
    return Commands.waitSeconds(GompeiLib.getLoopPeriod())
        .andThen(Commands.waitUntil(this::atGoal));
  }

  /**
   * Updates the PID values for the linkage.
   *
   * @param kp the proportional gain
   * @param kd the derivative gain
   */
  public void setPID(double kp, double kd) {
    io.setPID(kp, 0.0, kd);
  }

  /**
   * Updates the profile constraints.
   *
   * @param maxVelocityRadiansPerSecond Maximum velocity (rad/sec)
   * @param maxAccelerationRadiansPerSecondSquared Maximum acceleration (rad/sec^2)
   * @param goalToleranceRadians Tolerance (rad)
   */
  public void setProfile(
      double maxVelocityRadiansPerSecond,
      double maxAccelerationRadiansPerSecondSquared,
      double goalToleranceRadians) {
    io.setProfile(
        maxVelocityRadiansPerSecond, maxAccelerationRadiansPerSecondSquared, goalToleranceRadians);
  }

  // 23.5 degrees from deployed is when you start running the intake
  // TODO: Implement in autos and in deployment of intake ELLIOT HELPPPPPPPPPPP MAKE AN ISSUE
  public List<Pose3d> getLinkagePoses() { // TODO: Work in progress

    final double AB = LinkageConstants.LINK_LENGTHS.AB();
    final double BC = LinkageConstants.LINK_LENGTHS.BC();
    ;
    final double CD = LinkageConstants.LINK_LENGTHS.CD();
    ;
    final double DA = LinkageConstants.LINK_LENGTHS.DA();
    ;

    final Rotation2d aAngleFromHorizontal = inputs.position;
    final Rotation2d dAngleFromHorizontal = LinkageConstants.INTAKE_ANGLE_OFFSET;

    final Rotation2d angleA = aAngleFromHorizontal.minus(dAngleFromHorizontal);

    final double BD =
        Math.sqrt(Math.pow(AB, 2) + Math.pow(DA, 2) - 2 * AB * DA * Math.cos(angleA.getRadians()));

    final Rotation2d angleC =
        new Rotation2d(
            Math.acos(Math.pow(BD, 2) - Math.pow(BC, 2) - Math.pow(CD, 2) / (-2 * BC * CD)));

    final Rotation2d angleD =
        new Rotation2d(
            Math.asin(Math.sin(angleA.getRadians()) * AB / BD)
                + Math.asin(Math.sin(angleC.getRadians()) * BC / BD));

    final Rotation2d angleB =
        new Rotation2d(
            Math.asin(Math.sin(angleA.getRadians()) * DA / BD)
                + Math.asin(Math.sin(angleC.getRadians()) * CD / BD));

    final Translation3d pointA = LinkageConstants.LINKAGE_OFFSET;
    final Translation3d pointD =
        pointA.plus(
            new Translation3d(
                DA * Math.cos(LinkageConstants.INTAKE_ANGLE_OFFSET.getRadians()),
                0,
                DA * Math.sin(LinkageConstants.INTAKE_ANGLE_OFFSET.getRadians())));

    final Translation3d pointB =
        pointA.plus(
            new Translation3d(
                AB * Math.cos(aAngleFromHorizontal.getRadians()),
                0,
                AB * Math.sin(aAngleFromHorizontal.getRadians())));
    final Translation3d pointC =
        pointB.plus(
            new Translation3d(
                BC * Math.cos((aAngleFromHorizontal.getRadians() + angleB.getRadians()) - Math.PI),
                0,
                BC * Math.sin((angleA.getRadians() + angleB.getRadians()) - Math.PI)));

    final Pose3d poseA =
        new Pose3d(pointA, new Rotation3d(0, aAngleFromHorizontal.getRadians(), 0));
    final Pose3d poseB = new Pose3d(pointB, new Rotation3d(0, Math.PI - angleB.getRadians(), 0));
    final Pose3d poseC = new Pose3d(pointC, new Rotation3d(0, Math.PI - angleC.getRadians(), 0));
    final Pose3d poseD = new Pose3d(pointD, new Rotation3d(0, Math.PI - angleD.getRadians(), 0));

    return List.of(poseA, poseB, poseC, poseD);
  }

  public Pose3d getHopperWallPose() {

    final double yPos = Math.sin(inputs.position.getRadians()) * LinkageConstants.PIN_LENGTH;
    final double x0 = Math.cos(inputs.position.getRadians()) * LinkageConstants.PIN_LENGTH;

    double xOff = 0;

    final double Y_MIN = LinkageConstants.LINK_BOUNDS.MIN(); // 0.810921
    final double Y_PHASE_1 = LinkageConstants.LINK_BOUNDS.PHASE_1(); // 2.86545
    final double Y_PHASE_2 = LinkageConstants.LINK_BOUNDS.PHASE_2(); // 4.752162
    final double Y_MAX = LinkageConstants.LINK_BOUNDS.MAX(); // 6.46545

    final double RADIUS_1 = LinkageConstants.LINK_CONST.RADIUS_1();
    final double RADIUS_2 = LinkageConstants.LINK_CONST.RADIUS_2();
    final double CENTER_OFFSET = LinkageConstants.LINK_CONST.CENTER_OFFSET();

    if (yPos <= Y_PHASE_1 && yPos > Y_MIN) {
      xOff = Math.sqrt(Math.pow(RADIUS_1, 2) - Math.pow(yPos, 2)) - CENTER_OFFSET;
    } else if (yPos <= Y_PHASE_2 && yPos > Y_PHASE_1) {
      xOff = 0;
    } else if (yPos <= Y_MAX && yPos > Y_PHASE_2) {
      xOff = Math.sqrt(Math.pow(RADIUS_2, 2) - Math.pow(yPos - Y_PHASE_2, 2)) - RADIUS_2;
    }

    return new Pose3d(x0 + xOff, 0, yPos, new Rotation3d(0, 0, 0));
  }

  /**
   * Runs the system ID
   *
   * @return runs a sysID charecterization routine command
   */
  public Command runSysId() {
    return Commands.sequence(
        Commands.runOnce(() -> currentState = LinkageState.IDLE),
        characterizationRoutine.quasistatic(Direction.kForward),
        Commands.waitSeconds(3),
        characterizationRoutine.quasistatic(Direction.kReverse),
        Commands.waitSeconds(3),
        characterizationRoutine.dynamic(Direction.kForward),
        Commands.waitSeconds(3),
        characterizationRoutine.dynamic(Direction.kReverse));
  }
}
