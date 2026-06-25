package edu.wpi.team190.gompeilib.subsystems.arm;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.team190.gompeilib.core.GompeiLib;
import edu.wpi.team190.gompeilib.core.utility.control.Gains;
import edu.wpi.team190.gompeilib.core.utility.control.constraints.AngularPositionConstraints;
import edu.wpi.team190.gompeilib.core.utility.phoenix.GainSlot;
import edu.wpi.team190.gompeilib.core.utility.phoenix.PhoenixUtil;
import java.util.ArrayList;

public class ArmIOTalonFX implements ArmIO {
  protected final TalonFX talonFX;
  private final TalonFX[] followTalonFX;

  private final StatusSignal<Angle> positionRotations;
  private final StatusSignal<AngularVelocity> velocityRotationsPerSecond;
  private ArrayList<StatusSignal<Voltage>> appliedVolts;
  private ArrayList<StatusSignal<Current>> supplyCurrentAmps;
  private ArrayList<StatusSignal<Current>> torqueCurrentAmps;
  private ArrayList<StatusSignal<Temperature>> temperatureCelsius;
  private final StatusSignal<Double> positionSetpointRotations;
  private final StatusSignal<Double> positionErrorRotations;

  private StatusSignal<?>[] statusSignals;

  private final VoltageOut voltageRequest;
  private final MotionMagicVoltage positionVoltageRequest;

  private final TalonFXConfiguration config;

  protected final ArmConstants constants;

  public ArmIOTalonFX(ArmConstants constants) {
    talonFX = new TalonFX(constants.armCANID, constants.canBus);
    followTalonFX = new TalonFX[constants.armParameters.numMotors() - 1];

    config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.CurrentLimits.SupplyCurrentLimit = constants.currentLimits.supplyCurrentLimit().in(Amps);
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = constants.currentLimits.statorCurrentLimit().in(Amps);
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.Feedback.SensorToMechanismRatio = constants.armParameters.gearRatio();
    config.MotorOutput.Inverted = constants.invertedValue;

    config.Slot0.withKP(constants.slot0Gains.kP().get())
        .withKD(constants.slot0Gains.kD().get())
        .withKS(constants.slot0Gains.kS().get())
        .withKV(constants.slot0Gains.kV().get())
        .withKA(constants.slot0Gains.kA().get())
        .withKG(constants.slot0Gains.kG().get())
        .withGravityType(GravityTypeValue.Arm_Cosine);

    config.Slot1.withKP(constants.slot1Gains.kP().get())
        .withKD(constants.slot1Gains.kD().get())
        .withKS(constants.slot1Gains.kS().get())
        .withKV(constants.slot1Gains.kV().get())
        .withKA(constants.slot1Gains.kA().get())
        .withKG(constants.slot1Gains.kG().get())
        .withGravityType(GravityTypeValue.Arm_Cosine);

    config.Slot2.withKP(constants.slot2Gains.kP().get())
        .withKD(constants.slot2Gains.kD().get())
        .withKS(constants.slot2Gains.kS().get())
        .withKV(constants.slot2Gains.kV().get())
        .withKA(constants.slot2Gains.kA().get())
        .withKG(constants.slot2Gains.kG().get())
        .withGravityType(GravityTypeValue.Arm_Cosine);

    config.MotorOutput.Inverted = constants.invertedValue;
    config.ClosedLoopGeneral.ContinuousWrap = constants.armParameters.continuousOutput();
    config.MotionMagic =
        new MotionMagicConfigs()
            .withMotionMagicAcceleration(
                constants.constraints.maxAcceleration().get().in(RotationsPerSecondPerSecond))
            .withMotionMagicCruiseVelocity(
                constants.constraints.maxVelocity().get().in(RotationsPerSecond));

    PhoenixUtil.tryUntilOk(5, () -> talonFX.getConfigurator().apply(config, 0.25));

    appliedVolts = new ArrayList<>();
    supplyCurrentAmps = new ArrayList<>();
    torqueCurrentAmps = new ArrayList<>();
    temperatureCelsius = new ArrayList<>();

    positionRotations = talonFX.getPosition();
    velocityRotationsPerSecond = talonFX.getVelocity();
    appliedVolts.add(talonFX.getMotorVoltage());
    supplyCurrentAmps.add(talonFX.getSupplyCurrent());
    torqueCurrentAmps.add(talonFX.getTorqueCurrent());
    temperatureCelsius.add(talonFX.getDeviceTemp());

    positionSetpointRotations = talonFX.getClosedLoopReference();
    positionErrorRotations = talonFX.getClosedLoopError();

    for (int i = 0; i < constants.armParameters.numMotors() - 1; i++) {
      appliedVolts.add(followTalonFX[i].getMotorVoltage());
      supplyCurrentAmps.add(followTalonFX[i].getSupplyCurrent());
      torqueCurrentAmps.add(followTalonFX[i].getTorqueCurrent());
      temperatureCelsius.add(followTalonFX[i].getDeviceTemp());
    }

    voltageRequest = new VoltageOut(0).withEnableFOC(constants.enableFOC);
    positionVoltageRequest = new MotionMagicVoltage(0).withEnableFOC(constants.enableFOC);

    var signalsList = new ArrayList<StatusSignal<?>>();

    signalsList.add(positionRotations);
    signalsList.add(velocityRotationsPerSecond);
    signalsList.addAll(appliedVolts);
    signalsList.addAll(supplyCurrentAmps);
    signalsList.addAll(torqueCurrentAmps);
    signalsList.addAll(temperatureCelsius);
    signalsList.add(positionSetpointRotations);
    signalsList.add(positionErrorRotations);

    statusSignals = new StatusSignal[signalsList.size()];

    for (int i = 0; i < signalsList.size(); i++) {
      statusSignals[i] = signalsList.get(i);
    }

    BaseStatusSignal.setUpdateFrequencyForAll(1 / GompeiLib.getLoopPeriod(), statusSignals);

    talonFX.optimizeBusUtilization();

    PhoenixUtil.registerSignals(constants.canBus.isNetworkFD(), statusSignals);

    talonFX.setPosition(0);

    this.constants = constants;
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {

    inputs.position = new Rotation2d(positionRotations.getValue());
    inputs.velocity = velocityRotationsPerSecond.getValue();

    inputs.appliedVolts = new double[constants.armParameters.numMotors()];
    inputs.supplyCurrentAmps = new double[constants.armParameters.numMotors()];
    inputs.torqueCurrentAmps = new double[constants.armParameters.numMotors()];
    inputs.temperatureCelsius = new double[constants.armParameters.numMotors()];

    for (int i = 0; i < constants.armParameters.numMotors(); i++) {
      inputs.appliedVolts[i] = appliedVolts.get(i).getValueAsDouble();
      inputs.supplyCurrentAmps[i] = supplyCurrentAmps.get(i).getValueAsDouble();
      inputs.torqueCurrentAmps[i] = torqueCurrentAmps.get(i).getValueAsDouble();
      inputs.temperatureCelsius[i] = temperatureCelsius.get(i).getValueAsDouble();
    }

    inputs.positionGoal = new Rotation2d(positionVoltageRequest.getPositionMeasure());
    inputs.positionSetpoint =
        Rotation2d.fromRotations(positionSetpointRotations.getValueAsDouble());
    inputs.positionError = Rotation2d.fromRotations(positionErrorRotations.getValueAsDouble());

    inputs.gainSlot = GainSlot.integerToGainSlot(talonFX.getClosedLoopSlot().getValue());
  }

  @Override
  public void setVoltageGoal(Voltage voltageGoal) {
    talonFX.setControl(voltageRequest.withOutput(voltageGoal));
  }

  @Override
  public void setPositionGoal(Rotation2d positionGoal) {
    talonFX.setControl(positionVoltageRequest.withPosition(positionGoal.getRotations()));
  }

  @Override
  public boolean atVoltageGoal(Voltage voltageReference) {
    return appliedVolts.get(0).getValue().isNear(voltageReference, Millivolts.of(500));
  }

  @Override
  public boolean atPositionGoal(Rotation2d positionReference) {
    return Math.abs(positionRotations.getValueAsDouble() - positionReference.getRotations())
        < constants.constraints.goalTolerance().get().in(Rotations);
  }

  @Override
  public void setPosition(Rotation2d position) {
    talonFX.setPosition(position.getRotations());
  }

  @Override
  public void setGainSlot(GainSlot gainSlot) {
    switch (gainSlot) {
      case ONE:
        talonFX.setControl(positionVoltageRequest.withSlot(1));
        break;
      case TWO:
        talonFX.setControl(positionVoltageRequest.withSlot(2));
        break;
      default:
        talonFX.setControl(positionVoltageRequest.withSlot(0));
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
    PhoenixUtil.tryUntilOk(5, () -> talonFX.getConfigurator().apply(config));

    for (int i = 0; i < constants.armParameters.numMotors() - 1; i++) {
      int finalI = i;
      PhoenixUtil.tryUntilOk(5, () -> followTalonFX[finalI].getConfigurator().apply(config));
    }
  }

  @Override
  public void updateConstraints(AngularPositionConstraints constraints) {
    config.MotionMagic =
        new MotionMagicConfigs()
            .withMotionMagicAcceleration(
                constraints.maxAcceleration().get(RotationsPerSecondPerSecond))
            .withMotionMagicCruiseVelocity(constraints.maxVelocity().get(RotationsPerSecond));
    PhoenixUtil.tryUntilOk(5, () -> talonFX.getConfigurator().apply(config, 0.25));

    for (int i = 0; i < constants.armParameters.numMotors() - 1; i++) {
      int finalI = i;
      PhoenixUtil.tryUntilOk(5, () -> followTalonFX[finalI].getConfigurator().apply(config, 0.25));
    }
  }
}
