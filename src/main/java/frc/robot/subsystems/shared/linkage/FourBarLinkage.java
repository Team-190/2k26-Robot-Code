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
import java.util.List;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class FourBarLinkage {
  public final FourBarLinkageIO io;
  public final String aKitTopic;
  private final FourBarLinkageIOInputsAutoLogged inputs;

  private FourBarLinkageState currentState;
  private FourBarLinkageState.Output currentOutput;
  private final FourBarLinkageConstants constants;

  private final SysIdRoutine characterizationRoutine;
  private final LoggedMechanism2d mechanism2d;
  private final LoggedMechanismRoot2d root2d;
  private final LoggedMechanismLigament2d link1;
  private final LoggedMechanismLigament2d link2;
  private final LoggedMechanismLigament2d link3;
  private final LoggedMechanismLigament2d link4;

  /**
   * Creates a linkage object with four bars. Handles the calculations of position in 3D space.
   *
   * @param io
   * @param constants
   * @param subsystem
   * @param index
   */
  public FourBarLinkage(
      FourBarLinkageIO io, FourBarLinkageConstants constants, Subsystem subsystem, int index) {

    inputs = new FourBarLinkageIOInputsAutoLogged();
    this.io = io;
    this.constants = constants;
    this.mechanism2d =
        new LoggedMechanism2d(constants.LINKAGE_OFFSET.getX(), constants.LINKAGE_OFFSET.getZ());
    aKitTopic = subsystem.getName() + "/Linkage" + index;

    this.root2d = mechanism2d.getRoot("Linkage", 0.5, 0.5);

    this.link1 =
        root2d.append(new LoggedMechanismLigament2d("Link1", constants.LINK_LENGTHS.AB(), 0));
    this.link2 =
        link1.append(new LoggedMechanismLigament2d("Link2", constants.LINK_LENGTHS.BC(), 0));
    this.link3 =
        link2.append(new LoggedMechanismLigament2d("Link3", constants.LINK_LENGTHS.CD(), 0));
    this.link4 =
        link3.append(new LoggedMechanismLigament2d("Link4", constants.LINK_LENGTHS.DA(), 0));

    characterizationRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(0.5).per(Seconds),
                Volts.of(3.5),
                Seconds.of(10),
                (state) -> Logger.recordOutput(aKitTopic + "/sysIDState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> io.setVoltage(voltage.in(Volts)), null, subsystem));

    currentState = FourBarLinkageState.IDLE;
    currentOutput = currentState.set(0);
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

    List<Pose3d> currentPoses = getLinkagePoses();

    Rotation2d angleA = Rotation2d.fromRadians(currentPoses.get(0).getRotation().getY());
    Rotation2d angleB = Rotation2d.fromRadians(currentPoses.get(1).getRotation().getY());
    Rotation2d angleC = Rotation2d.fromRadians(currentPoses.get(2).getRotation().getY());
    Rotation2d angleD = Rotation2d.fromRadians(currentPoses.get(3).getRotation().getY());

    link1.setAngle(angleA);
    link2.setAngle(angleB.minus(angleA));
    link3.setAngle(new Rotation2d(2*Math.PI).plus(angleB).plus(angleC).times(-1));
    link4.setAngle(new Rotation2d(2*Math.PI).minus(angleD.plus(angleC)));

    Logger.recordOutput(aKitTopic + "LinkageMechanism", mechanism2d);
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
          currentState = FourBarLinkageState.OPEN_LOOP_VOLTAGE_CONTROL;
          currentOutput = currentState.set(volts);
        });
  }

  /**
   * Set the goal for the linkage.
   *
   * @param position
   * @return
   */
  public Command setPositionGoal(Rotation2d position) {
    return Commands.runOnce(
        () -> {
          currentState = FourBarLinkageState.CLOSED_LOOP_POSITION_CONTROL;
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
    return Commands.waitUntil(this::atGoal);
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
    io.setProfile(maxVelocityRadiansPerSecond, maxAccelerationRadiansPerSecondSquared);
  }

  public List<Pose3d> getLinkagePoses() {

    double L1 = constants.LINK_LENGTHS.AB();
    double L2 = constants.LINK_LENGTHS.BC();
    double L3 = constants.LINK_LENGTHS.CD();
    double L4 = constants.LINK_LENGTHS.DA();

    double theta1 = constants.INTAKE_ANGLE_OFFSET.getRadians();
    double theta2 = inputs.position.getRadians();

    double k1 = L4 / L1;
    double k4 = L4 / L2;
    double k5 =
        (-Math.pow(L1, 2) - Math.pow(L2, 2) + Math.pow(L3, 2) - Math.pow(L4, 2)) / (2 * L1 * L2);

    double D = Math.cos(theta2) - k1 + k4 * Math.cos(theta2) + k5;
    double E = -2 * Math.sin(theta2);
    double F = k1 + (k4 - 1) * Math.cos(theta2) + k5;

    double theta3_num = -E - Math.sqrt(Math.pow(E, 2) - (4 * D * F));
    double theta3_den = 2 * D;
    double theta3 = 2 * Math.atan2(theta3_num, theta3_den);

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

    final double yPos = inputs.position.getSin() * constants.PIN_LENGTH;
    final double x0 = inputs.position.getCos() * constants.PIN_LENGTH;

    double xOff = 0;

    final double Y_MIN = constants.LINK_BOUNDS.MIN(); // 0.810921
    final double Y_PHASE_1 = constants.LINK_BOUNDS.PHASE_1(); // 2.86545
    final double Y_PHASE_2 = constants.LINK_BOUNDS.PHASE_2(); // 4.752162
    final double Y_MAX = constants.LINK_BOUNDS.MAX(); // 6.46545

    final double RADIUS_1 = constants.LINK_CONST.RADIUS_1();
    final double RADIUS_2 = constants.LINK_CONST.RADIUS_2();
    final double CENTER_OFFSET = constants.LINK_CONST.CENTER_OFFSET();

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
        Commands.runOnce(() -> currentState = FourBarLinkageState.IDLE),
        characterizationRoutine.quasistatic(Direction.kForward),
        Commands.waitSeconds(3),
        characterizationRoutine.quasistatic(Direction.kReverse),
        Commands.waitSeconds(3),
        characterizationRoutine.dynamic(Direction.kForward),
        Commands.waitSeconds(3),
        characterizationRoutine.dynamic(Direction.kReverse));
  }
}
