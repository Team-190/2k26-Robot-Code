package frc.robot.subsystems.v2_Delta;

import frc.robot.Constants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class V2_DeltaIntakeIOSim implements V2_DeltaIntakeIO {
    public final ElevatorSim extensionSim;
    public final DCMotorSim rollerSim;
    
    private final ProfiledPIDController extensionController;
    private SimpleMotorFeedforward extensionFeedforward;

    private double extensionAppliedVolts;
    private double rollerAppliedVolts;
    private boolean extensionClosedLoop;

    public V2_DeltaIntakeIOSim() {
        extensionSim =
        new ElevatorSim(
            LinearSystemId.createElevatorSystem(
                V2_DeltaIntakeConstants.EXTENSION_PARAMS.MOTOR(),
                V2_DeltaIntakeConstants.EXTENSION_PARAMS.MASS_KG(),
                V2_DeltaIntakeConstants.EXTENSION_PARAMS.PITCH_DIAMETER(),
                V2_DeltaIntakeConstants.EXTENSION_MOTOR_GEAR_RATIO),
            V2_DeltaIntakeConstants.EXTENSION_PARAMS.MOTOR(),
            V2_DeltaIntakeConstants.EXTENSION_PARAMS.MIN_EXTENSION(),
            V2_DeltaIntakeConstants.EXTENSION_PARAMS.MAX_EXTENSION(),
            false,
            V2_DeltaIntakeConstants.EXTENSION_PARAMS.MIN_EXTENSION());
    rollerSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                V2_DeltaIntakeConstants.ROLLER_PARAMS.MOTOR(),
                V2_DeltaIntakeConstants.ROLLER_PARAMS.MOMENT_OF_INERTIA(),
                V2_DeltaIntakeConstants.ROLLER_MOTOR_GEAR_RATIO),
            V2_DeltaIntakeConstants.ROLLER_PARAMS.MOTOR());

    extensionController =
        new ProfiledPIDController(
            V2_DeltaIntakeConstants.EXTENSION_MOTOR_GAINS.kP().get(),
            0.0,
            V2_DeltaIntakeConstants.EXTENSION_MOTOR_GAINS.kD().get(),
            new TrapezoidProfile.Constraints(
                V2_DeltaIntakeConstants.EXTENSION_MOTOR_CONSTRAINTS.MAX_VELOCITY().get(),
                V2_DeltaIntakeConstants.EXTENSION_MOTOR_CONSTRAINTS.MAX_ACCELERATION().get()));
    extensionFeedforward =
        new SimpleMotorFeedforward(
            V2_DeltaIntakeConstants.EXTENSION_MOTOR_GAINS.kS().get(),
            V2_DeltaIntakeConstants.EXTENSION_MOTOR_GAINS.kV().get(),
            V2_DeltaIntakeConstants.EXTENSION_MOTOR_GAINS.kA().get());

    extensionAppliedVolts = 0.0;
    rollerAppliedVolts = 0.0;
    extensionClosedLoop = true;

    extensionController.enableContinuousInput(-Math.PI, Math.PI);

    }

    @Override
    public void updateInputs(V2_DeltaIntakeIOInputs inputs) {
    if (extensionClosedLoop) {
      extensionAppliedVolts =
          extensionController.calculate(extensionSim.getPositionMeters())
              + extensionFeedforward.calculate(extensionController.getSetpoint().position);
    }

    extensionAppliedVolts = MathUtil.clamp(extensionAppliedVolts, -12.0, 12.0);
    rollerAppliedVolts = MathUtil.clamp(rollerAppliedVolts, -12.0, 12.0);

    extensionSim.setInputVoltage(extensionAppliedVolts);
    rollerSim.setInputVoltage(rollerAppliedVolts);
    extensionSim.update(Constants.LOOP_PERIOD_SECONDS);
    rollerSim.update(Constants.LOOP_PERIOD_SECONDS);

    inputs.extensionPositionMeters = extensionSim.getPositionMeters();
    inputs.extensionVelocityMetersPerSecond = extensionSim.getVelocityMetersPerSecond();
    inputs.extensionAppliedVolts = extensionAppliedVolts;
    inputs.extensionSupplyCurrentAmps = extensionSim.getCurrentDrawAmps();
    inputs.extensionGoal =
        (extensionController.getGoal().position)
            / (2 * Math.PI)
            * V2_DeltaIntakeConstants.EXTENSION_MOTOR_GEAR_RATIO
            * V2_DeltaIntakeConstants.EXTENSION_MOTOR_METERS_PER_REV;
    inputs.extensionPositionSetpoint =
        (extensionController.getSetpoint().position)
            / (2 * Math.PI)
            * V2_DeltaIntakeConstants.EXTENSION_MOTOR_GEAR_RATIO
            * V2_DeltaIntakeConstants.EXTENSION_MOTOR_METERS_PER_REV;
    inputs.extensionPositionError =
        (extensionController.getPositionError())
            / (2 * Math.PI)
            * V2_DeltaIntakeConstants.EXTENSION_MOTOR_GEAR_RATIO
            * V2_DeltaIntakeConstants.EXTENSION_MOTOR_METERS_PER_REV;

    inputs.rollerPosition = Rotation2d.fromRadians(rollerSim.getAngularPositionRad());
    inputs.rollerVelocityRadiansPerSecond = rollerSim.getAngularVelocityRadPerSec();
    inputs.rollerAppliedVolts = rollerAppliedVolts;
    inputs.rollerSupplyCurrentAmps = rollerSim.getCurrentDrawAmps();
  }

  @Override
  public void setExtensionVoltage(double volts) {
    extensionClosedLoop = false;
    extensionAppliedVolts = volts;
  }

  @Override
  public void setRollerVoltage(double volts) {
    rollerAppliedVolts = volts;
  }

  @Override
  public void setExtensionGoal(double position) {
    extensionClosedLoop = true;
    extensionController.setGoal(position);
  }

  @Override
  public void stopRoller() {
    rollerAppliedVolts = 0.0;
  }

  @Override
  public boolean atExtensionPositionGoal() {
    return extensionController.atGoal();
  }

  @Override
  public void updateGains(double kP, double kD, double kS, double kV, double kA) {
    extensionController.setP(kP);
    extensionController.setD(kD);
    extensionFeedforward = new SimpleMotorFeedforward(kS, kV, kA);
  }

  @Override
  public void updateConstraints(double maxAcceleration, double maxVelocity) {
    extensionController.setConstraints(
        new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration));
  }
}
