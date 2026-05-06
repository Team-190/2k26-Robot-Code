package edu.wpi.team190.gompeilib.subsystems.generic.flywheel;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.*;
import edu.wpi.team190.gompeilib.core.GompeiLib;
import edu.wpi.team190.gompeilib.core.utility.control.Gains;
import edu.wpi.team190.gompeilib.core.utility.control.constraints.AngularVelocityConstraints;
import edu.wpi.team190.gompeilib.core.utility.phoenix.GainSlot;
import edu.wpi.team190.gompeilib.core.utility.phoenix.PhoenixUtil;
import java.util.ArrayList;

public class GenericFlywheelIOTalonFX implements GenericFlywheelIO {
  protected final TalonFX talonFX;
  private final TalonFX[] followerTalonFX;

  private final StatusSignal<Angle> positionRotations;
  private final StatusSignal<AngularVelocity> velocityRotationsPerSecond;
  private final ArrayList<StatusSignal<Voltage>> appliedVolts;
  private final ArrayList<StatusSignal<Current>> supplyCurrentAmps;
  private final ArrayList<StatusSignal<Current>> torqueCurrentAmps;
  private final ArrayList<StatusSignal<Temperature>> temperatureCelsius;
  private AngularVelocity velocityGoal;
  private final StatusSignal<Double> velocitySetpointRotationsPerSecond;
  private final StatusSignal<Double> velocityErrorRotationsPerSecond;

  private StatusSignal<?>[] statusSignals;

  private final TalonFXConfiguration talonFXConfiguration;

  private final NeutralOut neutralControlRequest;
  private final VoltageOut voltageControlRequest;
  private final TorqueCurrentFOC torqueCurrentFOCRequest;
  private final VelocityVoltage velocityControlRequest;
  private final VelocityTorqueCurrentFOC velocityTorqueCurrentRequest;

  protected GenericFlywheelConstants constants;

  public GenericFlywheelIOTalonFX(GenericFlywheelConstants constants) {
    talonFX = new TalonFX(constants.leaderCANID, constants.canBus);
    followerTalonFX =
        new TalonFX
            [constants.alignedFollowerCANIDs.size() + constants.opposedFollowerCANIDs.size()];

    talonFXConfiguration = new TalonFXConfiguration();

    talonFXConfiguration.MotorOutput.withInverted(constants.leaderInversion);

    talonFXConfiguration
        .CurrentLimits
        .withSupplyCurrentLimit(constants.currentLimit.supplyCurrentLimit())
        .withSupplyCurrentLimitEnable(true)
        .withStatorCurrentLimit(constants.currentLimit.statorCurrentLimit())
        .withStatorCurrentLimitEnable(true);
    talonFXConfiguration.MotorOutput.withNeutralMode(NeutralModeValue.Coast);
    talonFXConfiguration
        .Slot0
        .withKP(constants.voltageGains.kP().getAsDouble())
        .withKD(constants.voltageGains.kD().getAsDouble())
        .withKS(constants.voltageGains.kS().getAsDouble())
        .withKV(constants.voltageGains.kV().getAsDouble())
        .withKA(constants.voltageGains.kA().getAsDouble());

    talonFXConfiguration
        .Slot1
        .withKP(constants.torqueGains.kP().getAsDouble())
        .withKD(constants.torqueGains.kD().getAsDouble())
        .withKS(constants.torqueGains.kS().getAsDouble())
        .withKV(constants.torqueGains.kV().getAsDouble())
        .withKA(constants.torqueGains.kA().getAsDouble());

    talonFXConfiguration.Feedback.SensorToMechanismRatio = constants.gearRatio;

    talonFXConfiguration.MotionMagic =
        new MotionMagicConfigs()
            .withMotionMagicAcceleration(
                (AngularAcceleration) constants.constraints.maxAcceleration().get())
            .withMotionMagicCruiseVelocity(
                (AngularVelocity) constants.constraints.maxVelocity().get());

    PhoenixUtil.tryUntilOk(5, () -> talonFX.getConfigurator().apply(talonFXConfiguration, 0.25));

    final int[] indexHolder = {0}; // mutable index for array insertion

    constants.alignedFollowerCANIDs.forEach(
        id -> {
          TalonFX follower = new TalonFX(id, talonFX.getNetwork());
          followerTalonFX[indexHolder[0]++] = follower;

          PhoenixUtil.tryUntilOk(
              5, () -> follower.getConfigurator().apply(talonFXConfiguration, 0.25));

          follower.setControl(new Follower(talonFX.getDeviceID(), MotorAlignmentValue.Aligned));
        });

    constants.opposedFollowerCANIDs.forEach(
        id -> {
          TalonFX follower = new TalonFX(id, talonFX.getNetwork());
          followerTalonFX[indexHolder[0]++] = follower;

          PhoenixUtil.tryUntilOk(
              5, () -> follower.getConfigurator().apply(talonFXConfiguration, 0.25));

          follower.setControl(new Follower(talonFX.getDeviceID(), MotorAlignmentValue.Opposed));
        });

    positionRotations = talonFX.getPosition();
    velocityRotationsPerSecond = talonFX.getVelocity();

    appliedVolts = new ArrayList<>();
    supplyCurrentAmps = new ArrayList<>();
    torqueCurrentAmps = new ArrayList<>();
    temperatureCelsius = new ArrayList<>();

    appliedVolts.add(talonFX.getMotorVoltage());
    supplyCurrentAmps.add(talonFX.getSupplyCurrent());
    torqueCurrentAmps.add(talonFX.getTorqueCurrent());
    temperatureCelsius.add(talonFX.getDeviceTemp());

    for (TalonFX follower : followerTalonFX) {
      appliedVolts.add(follower.getMotorVoltage());
      supplyCurrentAmps.add(follower.getSupplyCurrent());
      torqueCurrentAmps.add(follower.getTorqueCurrent());
      temperatureCelsius.add(follower.getDeviceTemp());
    }

    velocityGoal = RotationsPerSecond.of(0.0);

    velocitySetpointRotationsPerSecond = talonFX.getClosedLoopReference();
    velocityErrorRotationsPerSecond = talonFX.getClosedLoopError();

    var signalsList = new ArrayList<StatusSignal<?>>();

    signalsList.add(positionRotations);
    signalsList.add(velocityRotationsPerSecond);
    signalsList.addAll(appliedVolts);
    signalsList.addAll(supplyCurrentAmps);
    signalsList.addAll(torqueCurrentAmps);
    signalsList.addAll(temperatureCelsius);

    statusSignals = new StatusSignal[signalsList.size()];

    for (int i = 0; i < signalsList.size(); i++) {
      statusSignals[i] = signalsList.get(i);
    }

    BaseStatusSignal.setUpdateFrequencyForAll(1 / GompeiLib.getLoopPeriod(), statusSignals);

    PhoenixUtil.registerSignals(constants.canBus.isNetworkFD(), statusSignals);

    talonFX.optimizeBusUtilization();
    for (TalonFX follower : followerTalonFX) {
      follower.optimizeBusUtilization();
    }

    neutralControlRequest = new NeutralOut();
    voltageControlRequest = new VoltageOut(0.0);
    torqueCurrentFOCRequest = new TorqueCurrentFOC(0.0);

    velocityControlRequest = new VelocityVoltage(0).withSlot(0);
    velocityTorqueCurrentRequest = new VelocityTorqueCurrentFOC(0.0).withSlot(1);

    this.constants = constants;
  }

  @Override
  public void updateInputs(GenericFlywheelIOInputs inputs) {
    inputs.position = Rotation2d.fromRotations(positionRotations.getValueAsDouble());
    inputs.velocity = velocityRotationsPerSecond.getValue();

    inputs.appliedVolts = new double[appliedVolts.size()];
    inputs.supplyCurrentAmps = new double[supplyCurrentAmps.size()];
    inputs.torqueCurrentAmps = new double[torqueCurrentAmps.size()];
    inputs.temperatureCelsius = new double[temperatureCelsius.size()];

    for (int i = 0; i <= followerTalonFX.length; i++) {
      inputs.appliedVolts[i] = appliedVolts.get(i).getValueAsDouble();
      inputs.supplyCurrentAmps[i] = supplyCurrentAmps.get(i).getValueAsDouble();
      inputs.torqueCurrentAmps[i] = torqueCurrentAmps.get(i).getValueAsDouble();
      inputs.temperatureCelsius[i] = temperatureCelsius.get(i).getValueAsDouble();
    }

    inputs.velocityGoal = velocityGoal;
    inputs.velocitySetpoint =
        RotationsPerSecond.of(velocitySetpointRotationsPerSecond.getValueAsDouble());
    inputs.velocityError = RotationsPerSecond.of(velocityRotationsPerSecond.getValueAsDouble());

    inputs.gainSlot = GainSlot.integerToGainSlot(talonFX.getClosedLoopSlot().getValue());
  }

  @Override
  public void setVoltageGoal(Voltage voltageGoal) {
    talonFX.setControl(
        voltageControlRequest.withOutput(voltageGoal).withEnableFOC(constants.enableFOC));
  }

  @Override
  public void setCurrentGoal(Current currentGoal) {
    talonFX.setControl(torqueCurrentFOCRequest.withOutput(currentGoal));
  }

  @Override
  public void setNeutralControl() {
    talonFX.setControl(neutralControlRequest);
  }

  @Override
  public void setVelocityGoal(AngularVelocity velocityGoal) {
    this.velocityGoal = velocityGoal;
    talonFX.setControl(
        velocityControlRequest.withVelocity(this.velocityGoal).withEnableFOC(constants.enableFOC));
  }

  @Override
  public void setVelocityGoal(AngularVelocity velocityGoal, Current currentFeedforward) {
    this.velocityGoal = velocityGoal;
    talonFX.setControl(
        velocityTorqueCurrentRequest
            .withVelocity(this.velocityGoal)
            .withFeedForward(currentFeedforward));
  }

  @Override
  public boolean atVoltageGoal(Voltage voltageReference) {
    return appliedVolts.get(0).isNear(voltageReference, Millivolts.of(500));
  }

  @Override
  public boolean atCurrentGoal(Current currentReference) {
    return torqueCurrentAmps.get(0).isNear(currentReference, Milliamps.of(500));
  }

  @Override
  public boolean atVelocityGoal(AngularVelocity velocityReference) {
    return velocityRotationsPerSecond.isNear(
        velocityReference,
        RotationsPerSecond.of(constants.constraints.goalTolerance().get(RotationsPerSecond)));
  }

  @Override
  public void updateGains(Gains gains, GainSlot gainSlot) {
    switch (gainSlot) {
      case ONE ->
          talonFXConfiguration
              .Slot1
              .withKP(gains.kP().get())
              .withKI(gains.kI().get())
              .withKD(gains.kD().get())
              .withKS(gains.kS().get())
              .withKV(gains.kV().get())
              .withKA(gains.kA().get())
              .withKG(gains.kG().get());
      default ->
          talonFXConfiguration
              .Slot0
              .withKP(gains.kP().get())
              .withKI(gains.kI().get())
              .withKD(gains.kD().get())
              .withKS(gains.kS().get())
              .withKV(gains.kV().get())
              .withKA(gains.kA().get())
              .withKG(gains.kG().get());
    }
    PhoenixUtil.tryUntilOk(5, () -> talonFX.getConfigurator().apply(talonFXConfiguration));
    for (TalonFX follower : followerTalonFX) {
      PhoenixUtil.tryUntilOk(5, () -> follower.getConfigurator().apply(talonFXConfiguration));
    }
  }

  @Override
  public void updateConstraints(AngularVelocityConstraints constraints) {
    talonFXConfiguration
        .MotionMagic
        .withMotionMagicAcceleration(constraints.maxAcceleration().get(RotationsPerSecondPerSecond))
        .withMotionMagicCruiseVelocity(constraints.maxVelocity().get(RotationsPerSecond));
    PhoenixUtil.tryUntilOk(5, () -> talonFX.getConfigurator().apply(talonFXConfiguration));
    for (TalonFX follower : followerTalonFX) {
      PhoenixUtil.tryUntilOk(5, () -> follower.getConfigurator().apply(talonFXConfiguration));
    }
  }
}
