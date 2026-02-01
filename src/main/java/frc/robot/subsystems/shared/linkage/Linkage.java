package frc.robot.subsystems.shared.linkage;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
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

    currentState = LinkageState.CLOSED_LOOP_POSITION_CONTROL;
    currentOutput = currentState.set(Rotation2d.fromDegrees(0));
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
          currentOutput = currentState.set(volts);
        });
  }

  public Command setPositionGoal(Rotation2d position) {
    return Commands.runOnce(
        () -> {
          currentState = LinkageState.CLOSED_LOOP_POSITION_CONTROL;
          currentOutput = currentState.set(position);
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

  public List<Pose3d> getLinkagePoses() {

    double L1 = LinkageConstants.LINK_LENGTHS.AB();
    double L2 = LinkageConstants.LINK_LENGTHS.BC();
    double L3 = LinkageConstants.LINK_LENGTHS.CD();
    double L4 = LinkageConstants.LINK_LENGTHS.DA();

    double theta1 = Units.degreesToRadians(-30.96);
    double theta2 = inputs.position.getRadians();

    double k1 = L4 / L1;
    // double k2 = L4 / L3;
    // double k3 =
    //     (Math.pow(L1, 2) - Math.pow(L2, 2) + Math.pow(L3, 2) + Math.pow(L4, 2)) / (2 * L1 * L3);
    double k4 = L4 / L2;
    double k5 =
        (-Math.pow(L1, 2) - Math.pow(L2, 2) + Math.pow(L3, 2) - Math.pow(L4, 2)) / (2 * L1 * L2);

    // double A = Math.cos(theta2) - k1 - k2 * Math.cos(theta2) + k3;
    // double B = -2 * Math.sin(theta2);
    // double C = k1 - (k2 + 1) * Math.cos(theta2) + k3;
    double D = Math.cos(theta2) - k1 + k4 * Math.cos(theta2) + k5;
    double E = -2 * Math.sin(theta2);
    double F = k1 + (k4 - 1) * Math.cos(theta2) + k5;

    double theta3_num = -E - Math.sqrt(Math.pow(E, 2) - (4 * D * F));
    double theta3_den = 2 * D;
    double theta3 = 2 * Math.atan2(theta3_num, theta3_den);

    // double theta4_num = -B - Math.sqrt(Math.pow(B, 2) - (4 * A * C));
    // double theta4_den = 2 * A;
    // double theta4 = 2 * Math.atan2(theta4_num, theta4_den);

    Translation3d point1 = new Translation3d();

    double x2 = L1 * Math.cos(theta2);
    double y2 = L1 * Math.sin(theta2);

    Translation3d point2 = new Translation3d(x2, 0, y2);

    double x3 = L1 * Math.cos(theta2) + L2 * Math.cos(theta3);
    double y3 = L1 * Math.sin(theta2) + L2 * Math.sin(theta3);

    Translation3d point3 = new Translation3d(x3, 0, y3);

    double x4 = L4 * Math.cos(theta1);
    double y4 = L4 * Math.sin(theta1);

    double theta5 = Math.atan2(y4 - y3, x4 - x3);

    Translation3d point4 = new Translation3d(x4, 0, y4);

    Pose3d pose1 = new Pose3d(point1, new Rotation3d(0, -theta2, 0));
    Pose3d pose2 = new Pose3d(point2, new Rotation3d(0, -theta3, 0));
    Pose3d pose3 = new Pose3d(point3, new Rotation3d(0, -theta5, 0));
    Pose3d pose4 = new Pose3d(point4, new Rotation3d(0, Math.PI - theta1, 0));

    return List.of(pose1, pose2, pose3, pose4);
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

    return new Pose3d(x0 + xOff, 0, 0, new Rotation3d(0, 0, 0));
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
