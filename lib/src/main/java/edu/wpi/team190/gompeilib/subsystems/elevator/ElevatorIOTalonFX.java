package edu.wpi.team190.gompeilib.subsystems.elevator;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.units.measure.*;
import edu.wpi.team190.gompeilib.core.GompeiLib;
import edu.wpi.team190.gompeilib.core.utility.control.Gains;
import edu.wpi.team190.gompeilib.core.utility.control.constraints.LinearConstraints;
import edu.wpi.team190.gompeilib.core.utility.phoenix.GainSlot;
import edu.wpi.team190.gompeilib.core.utility.phoenix.PhoenixUtil;
import java.util.ArrayList;

public class ElevatorIOTalonFX implements ElevatorIO {

  // Core hardware components
  protected final TalonFX talonFX;
  public final TalonFX[] followTalonFX;

  // Configuration
  public final TalonFXConfiguration config;
  protected final ElevatorConstants constants;

  // Sensor inputs
  private StatusSignal<Angle> positionRotations;
  private StatusSignal<AngularVelocity> velocityRotationsPerSecond;
  private StatusSignal<AngularAcceleration> accelerationRotationsPerSecondPerSecond;
  private ArrayList<StatusSignal<Voltage>> appliedVolts;
  private ArrayList<StatusSignal<Current>> supplyCurrentAmps;
  private ArrayList<StatusSignal<Current>> torqueCurrentAmps;
  private ArrayList<StatusSignal<Temperature>> temperatureCelsius;

  public Distance positionGoalMeters;

  private StatusSignal<Double> positionSetpointRotations;
  private StatusSignal<Double> positionErrorRotations;

  private StatusSignal<?>[] statusSignals;

  private MotionMagicVoltage positionVoltageRequest;
  private VoltageOut voltageRequest;

  public ElevatorIOTalonFX(ElevatorConstants constants) {

    this.constants = constants;

    // Create lead motor
    talonFX = new TalonFX(constants.leaderCANID, constants.canBus);

    // Create follower motor array (define length)
    followTalonFX = new TalonFX[constants.elevatorParameters.NUM_MOTORS() - 1];

    config = new TalonFXConfiguration();
    config.Slot0.withKP(constants.slot0Gains.kP().get())
        .withKD(constants.slot0Gains.kD().get())
        .withKS(constants.slot0Gains.kS().get())
        .withKV(constants.slot0Gains.kV().get())
        .withKA(constants.slot0Gains.kA().get())
        .withKG(constants.slot0Gains.kG().get())
        .withGravityType(GravityTypeValue.Elevator_Static);

    config.Slot1.withKP(constants.slot1Gains.kP().get())
        .withKD(constants.slot1Gains.kD().get())
        .withKS(constants.slot1Gains.kS().get())
        .withKV(constants.slot1Gains.kV().get())
        .withKA(constants.slot1Gains.kA().get())
        .withKG(constants.slot1Gains.kG().get())
        .withGravityType(GravityTypeValue.Elevator_Static);

    config.Slot2.withKP(constants.slot2Gains.kP().get())
        .withKD(constants.slot2Gains.kD().get())
        .withKS(constants.slot2Gains.kS().get())
        .withKV(constants.slot2Gains.kV().get())
        .withKA(constants.slot2Gains.kA().get())
        .withKG(constants.slot2Gains.kG().get())
        .withGravityType(GravityTypeValue.Elevator_Static);

    config.CurrentLimits.withSupplyCurrentLimit(constants.elevatorSupplyCurrentLimit)
        .withSupplyCurrentLimitEnable(true)
        .withStatorCurrentLimit(constants.elevatorStatorCurrentLimit)
        .withStatorCurrentLimitEnable(true);

    config.Feedback.SensorToMechanismRatio =
        constants.elevatorGearRatio / (2 * Math.PI * constants.drumRadius);

    config.SoftwareLimitSwitch.withForwardSoftLimitThreshold(
            constants.elevatorParameters.MAX_HEIGHT().in(Meters))
        .withForwardSoftLimitEnable(true)
        .withReverseSoftLimitThreshold(constants.elevatorParameters.MIN_HEIGHT().in(Meters))
        .withReverseSoftLimitEnable(true);

    config.MotionMagic.withMotionMagicAcceleration(
            constants.constraints.maxAcceleration().get().in(MetersPerSecondPerSecond))
        .withMotionMagicCruiseVelocity(
            constants.constraints.maxVelocity().get().in(MetersPerSecond));

    PhoenixUtil.tryUntilOk(5, () -> talonFX.getConfigurator().apply(config));

    final int[] indexHolder = {0}; // mutable index for array insertion

    constants.alignedFollowerCANIDs.forEach(
        id -> {
          TalonFX follower = new TalonFX(id, talonFX.getNetwork());
          followTalonFX[indexHolder[0]++] = follower;

          PhoenixUtil.tryUntilOk(5, () -> follower.getConfigurator().apply(config, 0.25));

          follower.setControl(new Follower(talonFX.getDeviceID(), MotorAlignmentValue.Aligned));
        });

    constants.opposedFollowerCANIDs.forEach(
        id -> {
          TalonFX follower = new TalonFX(id, talonFX.getNetwork());
          followTalonFX[indexHolder[0]++] = follower;

          PhoenixUtil.tryUntilOk(5, () -> follower.getConfigurator().apply(config, 0.25));

          follower.setControl(new Follower(talonFX.getDeviceID(), MotorAlignmentValue.Opposed));
        });

    appliedVolts = new ArrayList<>();
    supplyCurrentAmps = new ArrayList<>();
    torqueCurrentAmps = new ArrayList<>();
    temperatureCelsius = new ArrayList<>();

    positionRotations = talonFX.getPosition();
    velocityRotationsPerSecond = talonFX.getVelocity();
    accelerationRotationsPerSecondPerSecond = talonFX.getAcceleration();
    appliedVolts.add(talonFX.getMotorVoltage());
    supplyCurrentAmps.add(talonFX.getSupplyCurrent());
    torqueCurrentAmps.add(talonFX.getTorqueCurrent());
    temperatureCelsius.add(talonFX.getDeviceTemp());
    positionGoalMeters = Meters.of(0.0);
    positionSetpointRotations = talonFX.getClosedLoopReference();
    positionErrorRotations = talonFX.getClosedLoopError();

    for (TalonFX follower : followTalonFX) {
      appliedVolts.add(follower.getMotorVoltage());
      supplyCurrentAmps.add(follower.getSupplyCurrent());
      torqueCurrentAmps.add(follower.getTorqueCurrent());
      temperatureCelsius.add(follower.getDeviceTemp());
    }

    var signalsList = new ArrayList<StatusSignal<?>>();

    signalsList.add(positionRotations);
    signalsList.add(velocityRotationsPerSecond);
    signalsList.add(accelerationRotationsPerSecondPerSecond);
    signalsList.add(positionSetpointRotations);
    signalsList.add(positionErrorRotations);
    signalsList.addAll(appliedVolts);
    signalsList.addAll(supplyCurrentAmps);
    signalsList.addAll(torqueCurrentAmps);
    signalsList.addAll(temperatureCelsius);

    statusSignals = new StatusSignal[signalsList.size()];

    for (int i = 0; i < signalsList.size(); i++) {
      statusSignals[i] = signalsList.get(i);
    }

    BaseStatusSignal.setUpdateFrequencyForAll(1 / GompeiLib.getLoopPeriod(), statusSignals);

    talonFX.optimizeBusUtilization();
    for (TalonFX follower : followTalonFX) {
      follower.optimizeBusUtilization();
    }

    positionVoltageRequest = new MotionMagicVoltage(0.0);
    voltageRequest = new VoltageOut(0.0);

    PhoenixUtil.registerSignals(constants.canBus.isNetworkFD(), statusSignals);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {

    // CTRE status signals are natively in rotations, but setting sensor to mechanism ratio
    // including the circumference of the drum allows us to transform into meters directly from
    // status signal object
    inputs.position = Meters.of(positionRotations.getValueAsDouble());
    inputs.velocity = MetersPerSecond.of(velocityRotationsPerSecond.getValueAsDouble());
    inputs.acceleration =
        MetersPerSecondPerSecond.of(accelerationRotationsPerSecondPerSecond.getValueAsDouble());

    inputs.appliedVolts = new double[appliedVolts.size()];
    inputs.supplyCurrentAmps = new double[supplyCurrentAmps.size()];
    inputs.torqueCurrentAmps = new double[torqueCurrentAmps.size()];
    inputs.temperatureCelsius = new double[temperatureCelsius.size()];

    for (int i = 0; i <= followTalonFX.length; i++) {
      inputs.appliedVolts[i] = appliedVolts.get(i).getValueAsDouble();
      inputs.supplyCurrentAmps[i] = supplyCurrentAmps.get(i).getValueAsDouble();
      inputs.torqueCurrentAmps[i] = torqueCurrentAmps.get(i).getValueAsDouble();
      inputs.temperatureCelsius[i] = temperatureCelsius.get(i).getValueAsDouble();
    }
    inputs.positionGoalMeters = positionGoalMeters;
    inputs.positionSetpointMeters = Meters.of(positionSetpointRotations.getValueAsDouble());
    inputs.positionErrorMeters = Meters.of(positionErrorRotations.getValueAsDouble());

    inputs.gainSlot = GainSlot.integerToGainSlot(talonFX.getClosedLoopSlot().getValue());
  }

  @Override
  public void setVoltageGoal(Voltage voltageGoal) {
    talonFX.setControl(voltageRequest.withOutput(voltageGoal).withEnableFOC(true));
  }

  @Override
  public void setPositionGoal(Distance positionGoal) {
    positionGoalMeters = positionGoal;
    talonFX.setControl(positionVoltageRequest.withPosition(positionGoal.in(Meters)));
  }

  @Override
  public boolean atVoltageGoal(Voltage voltageReference) {
    return appliedVolts.get(0).getValue().isNear(voltageReference, Millivolts.of(500));
  }

  @Override
  public boolean atPositionGoal(Distance positionReference) {
    return Math.abs(positionRotations.getValueAsDouble() - positionReference.in(Meters))
        <= constants.constraints.goalTolerance().get(Meters);
  }

  @Override
  public void setPosition(Distance position) {
    talonFX.setPosition(position.in(Meters));
  }

  @Override
  public void setGainSlot(GainSlot slot) {
    switch (slot) {
      case ONE:
        talonFX.setControl(positionVoltageRequest.withSlot(1));
        break;
      case TWO:
        talonFX.setControl(positionVoltageRequest.withSlot(2));
        break;
      default:
        talonFX.setControl(positionVoltageRequest.withSlot(0));
        break;
    }
  }

  @Override
  public void updateGains(Gains gains, GainSlot gainSlot) {
    switch (gainSlot) {
      case ZERO:
        config.Slot0.withKP(gains.kP().get())
            .withKD(gains.kD().get())
            .withKS(gains.kS().get())
            .withKV(gains.kV().get())
            .withKA(gains.kA().get())
            .withKG(gains.kG().get());
        break;
      case ONE:
        config.Slot1.withKP(gains.kP().get())
            .withKD(gains.kD().get())
            .withKS(gains.kS().get())
            .withKV(gains.kV().get())
            .withKA(gains.kA().get())
            .withKG(gains.kG().get());
        break;
      case TWO:
      default:
        config.Slot2.withKP(gains.kP().get())
            .withKD(gains.kD().get())
            .withKS(gains.kS().get())
            .withKV(gains.kV().get())
            .withKA(gains.kA().get())
            .withKG(gains.kG().get());
        break;
    }
    PhoenixUtil.tryUntilOk(5, () -> talonFX.getConfigurator().apply(config, 0.25));
    for (TalonFX follower : followTalonFX) {
      PhoenixUtil.tryUntilOk(5, () -> follower.getConfigurator().apply(config, 0.25));
    }
  }

  @Override
  public void updateConstraints(LinearConstraints constraints) {
    config.MotionMagic.withMotionMagicAcceleration(
            constraints.maxAcceleration().get().in(MetersPerSecondPerSecond))
        .withMotionMagicCruiseVelocity(constraints.maxVelocity().get().in(MetersPerSecond));
    PhoenixUtil.tryUntilOk(5, () -> talonFX.getConfigurator().apply(config, 0.25));
    for (TalonFX follower : followTalonFX) {
      PhoenixUtil.tryUntilOk(5, () -> follower.getConfigurator().apply(config, 0.25));
    }
  }
}
