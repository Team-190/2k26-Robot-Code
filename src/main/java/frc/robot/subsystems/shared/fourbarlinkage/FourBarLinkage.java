package frc.robot.subsystems.shared.fourbarlinkage;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.team190.gompeilib.core.utility.Setpoint;
import edu.wpi.team190.gompeilib.core.utility.control.Gains;
import edu.wpi.team190.gompeilib.core.utility.control.constraints.AngularPositionConstraints;
import frc.robot.subsystems.shared.fourbarlinkage.FourBarLinkageConstants.LinkageState;
import java.util.List;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class FourBarLinkage {
  private final FourBarLinkageIO io;
  private final String aKitTopic;
  private final FourBarLinkageIOInputsAutoLogged inputs;

  private FourBarLinkageState currentState;

  private Setpoint<VoltageUnit> voltageGoal;
  private Setpoint<AngleUnit> positionGoal;

  private final FourBarLinkageConstants constants;

  private final SysIdRoutine characterizationRoutine;
  private final LoggedMechanism2d mechanism2d;
  private final LoggedMechanismRoot2d root2d;
  private final LoggedMechanismLigament2d crank;
  private final LoggedMechanismLigament2d coupler;
  private final LoggedMechanismLigament2d follower;
  private final LoggedMechanismLigament2d ground;

  /**
   * Creates a linkage object with four bars. Handles the calculations of position in 3D space.
   *
   * @param io the IO object for the linkage. Either {@link
   *     frc.robot.subsystems.shared.fourbarlinkage.FourBarLinkageIOSim} or {@link
   *     frc.robot.subsystems.shared.fourbarlinkage.FourBarLinkageIOTalonFX}.
   * @param constants The constants file for FourBarLinkage.{@link
   *     frc.robot.subsystems.shared.fourbarlinkage.FourBarLinkageConstants}
   * @param subsystem The subsystem this linkage belongs to.
   * @param name The name of multiple linkages in the same subsystem.
   */
  public FourBarLinkage(
      FourBarLinkageIO io,
      FourBarLinkageConstants constants,
      Subsystem subsystem,
      String name,
      Setpoint<AngleUnit> positionGoal,
      Setpoint<VoltageUnit> voltageGoal) {

    inputs = new FourBarLinkageIOInputsAutoLogged();
    this.io = io;
    this.constants = constants;
    this.mechanism2d =
        new LoggedMechanism2d(constants.linkageOffset.getX(), constants.linkageOffset.getZ());
    aKitTopic = subsystem.getName() + "/Linkage" + name;

    this.root2d = mechanism2d.getRoot("Linkage", 0.5, 0.5);

    this.crank =
        root2d.append(new LoggedMechanismLigament2d("Crank", constants.linkLengths.AB(), 0));
    this.coupler =
        crank.append(new LoggedMechanismLigament2d("Coupler", constants.linkLengths.BC(), 0));
    this.follower =
        coupler.append(new LoggedMechanismLigament2d("Follower", constants.linkLengths.CD(), 0));
    this.ground =
        follower.append(new LoggedMechanismLigament2d("Ground", constants.linkLengths.DA(), 0));

    characterizationRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(0.5).per(Seconds),
                Volts.of(3.5),
                Seconds.of(10),
                (state) -> Logger.recordOutput(aKitTopic + "/sysIDState", state.toString())),
            new SysIdRoutine.Mechanism(io::setVoltageGoal, null, subsystem));

    currentState = FourBarLinkageState.IDLE;

    this.voltageGoal = voltageGoal;
    this.positionGoal = positionGoal;
  }

  public FourBarLinkage(
      FourBarLinkageIO io, FourBarLinkageConstants constants, Subsystem subsystem, String name) {
    this(
        io,
        constants,
        subsystem,
        name,
        new Setpoint<>(
            constants.minAngle.getMeasure(),
            constants.positionOffsetStep.getMeasure(),
            constants.minAngle.getMeasure(),
            constants.maxAngle.getMeasure()),
        new Setpoint<>(Volts.of(0), constants.voltageOffsetStep, Volts.of(-12), Volts.of(12)));
  }

  public FourBarLinkage(
      FourBarLinkageIO io,
      FourBarLinkageConstants constants,
      Subsystem subsystem,
      String name,
      Setpoint<AngleUnit> positionGoal) {
    this(
        io,
        constants,
        subsystem,
        name,
        positionGoal,
        new Setpoint<>(Volts.of(0), constants.voltageOffsetStep, Volts.of(-12), Volts.of(12)));
  }

  public void periodic() {
    io.updateInputs(inputs);

    Logger.processInputs(aKitTopic, inputs);

    Logger.recordOutput(aKitTopic + "/Position Goal", positionGoal.getSetpoint());
    Logger.recordOutput(aKitTopic + "/Voltage Goal", voltageGoal.getSetpoint());
    Logger.recordOutput(aKitTopic + "/Position Offset", positionGoal.getOffset());
    Logger.recordOutput(aKitTopic + "/Voltage Offset", voltageGoal.getOffset());
    Logger.recordOutput(aKitTopic + "/At Position Goal", atPositionGoal());
    Logger.recordOutput(aKitTopic + "/At Voltage Goal", atVoltageGoal());
    Logger.recordOutput(aKitTopic + "/State", currentState);

    switch (currentState) {
      case OPEN_LOOP_VOLTAGE_CONTROL -> {
        io.setVoltageGoal((Voltage) voltageGoal.getNewSetpoint());
      }
      case CLOSED_LOOP_POSITION_CONTROL ->
          io.setPositionGoal(new Rotation2d(positionGoal.getNewSetpoint().baseUnitMagnitude()));
      default -> {}
    }

    List<Rotation2d> currentPoses = getLinkagePoses().stream().map(LinkageState::rotation).toList();

    Rotation2d crankAngle = currentPoses.get(0);
    Rotation2d couplerAngle = currentPoses.get(1);
    Rotation2d followerAngle = currentPoses.get(2);
    Rotation2d groundAngle = currentPoses.get(3);

    crank.setAngle(crankAngle.unaryMinus());
    coupler.setAngle(couplerAngle.minus(crankAngle).unaryMinus());
    follower.setAngle(followerAngle.minus(couplerAngle).unaryMinus());
    ground.setAngle(groundAngle.minus(followerAngle).unaryMinus());

    Logger.recordOutput(aKitTopic + "/LinkageMechanism", mechanism2d);
  }

  /**
   * Gets the current position of the linkage.
   *
   * @return The current position of the linkage as a Rotation2d.
   */
  public Rotation2d getPosition() {
    return inputs.position;
  }

  public Command setIdle() {
    return Commands.runOnce(
        () -> {
          currentState = FourBarLinkageState.IDLE;
        });
  }

  /**
   * Sets the voltage being passed into the linkage subsystem.
   *
   * @param voltage the voltage passed into the linkage.
   * @return A command that sets the specified voltage.
   */
  public void setVoltageGoal(Voltage voltage) {
    currentState = FourBarLinkageState.OPEN_LOOP_VOLTAGE_CONTROL;
    voltageGoal.setSetpoint(voltage);
  }

  /**
   * Set the goal for the linkage.
   *
   * @param position the goal
   * @return A command to set the goal to the specified value.
   */
  public void setPositionGoal(Rotation2d position) {
    currentState = FourBarLinkageState.CLOSED_LOOP_POSITION_CONTROL;
    positionGoal.setSetpoint(position.getMeasure());
  }

  public void setVoltageGoal(Setpoint<VoltageUnit> voltageGoal) {
    currentState = FourBarLinkageState.OPEN_LOOP_VOLTAGE_CONTROL;
    this.voltageGoal = voltageGoal;
  }

  public void setPositionGoal(Setpoint<AngleUnit> positionGoal) {
    currentState = FourBarLinkageState.CLOSED_LOOP_POSITION_CONTROL;
    this.positionGoal = positionGoal;
  }

  public void setPosition(Rotation2d position) {
    io.setPosition(position);
  }

  /**
   * Checks if the linkage is at the goal position.
   *
   * @return If the linkage is within tolerance of the goal (true) or not (false).
   */
  public boolean atPositionGoal() {
    return io.atPositionGoal(new Rotation2d(positionGoal.getNewSetpoint().baseUnitMagnitude()));
  }

  /**
   * Checks if the linkage is at the goal position.
   *
   * @param position The state to check goal against.
   * @return If the linkage is within tolerance of the goal (true) or not (false).
   */
  public boolean atPositionGoal(Rotation2d positionReference) {
    return io.atPositionGoal(positionReference);
  }

  public boolean atVoltageGoal() {
    return io.atVoltageGoal((Voltage) voltageGoal.getNewSetpoint());
  }

  public boolean atVoltageGoal(Voltage voltageReference) {
    return io.atVoltageGoal(voltageReference);
  }

  /**
   * Waits until linkage at goal/in tolerance.
   *
   * @return A command that waits until the linkage is at the goal.
   */
  public Command waitUntilLinkageAtGoal() {
    return Commands.waitUntil(this::atPositionGoal);
  }

  /**
   * Updates the PID values for the linkage.
   *
   * @param gains The proportional gain, derivative gain, and feedforward gains.
   */
  public void setGains(Gains gains) {
    io.setGains(gains);
  }

  /**
   * Updates the profile constraints.
   *
   * @param constraints the new profile constraints to set.
   */
  public void setProfile(AngularPositionConstraints constraints) {
    io.setProfile(constraints);
  }

  /**
   * Gets the poses of each linkage in 3D space.
   *
   * @return List of LinkageStates representing the 3d poses and 2d rotations of each link.
   */
  public List<LinkageState> getLinkagePoses() {
    double crankLength = constants.linkLengths.AB();
    double couplerLength = constants.linkLengths.BC();
    double followerLength = constants.linkLengths.CD();
    double groundLength = constants.linkLengths.DA();

    double theta1 = constants.intakeAngleOffset.getRadians();
    double theta2 = -inputs.position.minus(constants.zeroOffset).getRadians() - theta1;

    double k1 = groundLength / crankLength;
    double k4 = groundLength / couplerLength;
    double k5 =
        (-Math.pow(crankLength, 2)
                - Math.pow(couplerLength, 2)
                + Math.pow(followerLength, 2)
                - Math.pow(groundLength, 2))
            / (2 * crankLength * couplerLength);

    double D = Math.cos(theta2) - k1 + k4 * Math.cos(theta2) + k5;
    double E = -2 * Math.sin(theta2);
    double F = k1 + (k4 - 1) * Math.cos(theta2) + k5;

    double theta3_num = -E - Math.sqrt(Math.pow(E, 2) - (4 * D * F));
    double theta3_den = 2 * D;
    double theta3 = 2 * Math.atan2(theta3_num, theta3_den);

    Translation3d point1 = new Translation3d(0, 0, 0);

    double x2 = crankLength * Math.cos(theta2 + theta1);
    double z2 = crankLength * Math.sin(theta2 + theta1);
    Translation3d point2 = new Translation3d(x2, 0, z2);

    double x3 = x2 + couplerLength * Math.cos(theta3 + theta1);
    double z3 = z2 + couplerLength * Math.sin(theta3 + theta1);
    Translation3d point3 = new Translation3d(x3, 0, z3);

    double x4 = groundLength * Math.cos(theta1);
    double z4 = groundLength * Math.sin(theta1);
    Translation3d point4 = new Translation3d(x4, 0, z4);

    double theta5 = Math.atan2(z4 - z3, x4 - x3);

    Pose3d crankPose = new Pose3d(point1, new Rotation3d(0, -(theta2 + theta1), 0));
    Pose3d couplerPose = new Pose3d(point2, new Rotation3d(0, -(theta3 + theta1), 0));
    Pose3d followerPose = new Pose3d(point3, new Rotation3d(0, -theta5, 0));
    Pose3d groundPose = new Pose3d(point4, new Rotation3d(0, -(Math.PI + theta1), 0));

    Rotation2d crankAngle = Rotation2d.fromRadians(-(theta2 + theta1));
    Rotation2d couplerAngle = Rotation2d.fromRadians(-(theta3 + theta1));
    Rotation2d followerAngle = Rotation2d.fromRadians(-theta5);
    Rotation2d groundAngle = Rotation2d.fromRadians(-(Math.PI + theta1));

    LinkageState link1 = new LinkageState(crankPose, crankAngle);
    LinkageState link2 = new LinkageState(couplerPose, couplerAngle);
    LinkageState link3 = new LinkageState(followerPose, followerAngle);
    LinkageState link4 = new LinkageState(groundPose, groundAngle);

    return List.of(link1, link2, link3, link4);
  }

  /**
   * Runs the system ID
   *
   * @return runs a sysID charecterization routine command
   */
  public Command runSysId() {
    return Commands.sequence(
        Commands.runOnce(() -> currentState = FourBarLinkageState.IDLE),
        Commands.print("Sys Id being run"),
        characterizationRoutine
            .quasistatic(Direction.kForward)
            .until(() -> atPositionGoal(constants.maxAngle)),
        Commands.waitSeconds(3),
        characterizationRoutine
            .quasistatic(Direction.kReverse)
            .until(() -> atPositionGoal(constants.minAngle)),
        Commands.waitSeconds(3),
        characterizationRoutine
            .dynamic(Direction.kForward)
            .until(() -> atPositionGoal(constants.maxAngle)),
        Commands.waitSeconds(3),
        characterizationRoutine
            .dynamic(Direction.kReverse)
            .until(() -> atPositionGoal(constants.minAngle)));
  }
}
