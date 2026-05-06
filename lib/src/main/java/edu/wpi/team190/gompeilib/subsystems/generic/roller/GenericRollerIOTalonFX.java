package edu.wpi.team190.gompeilib.subsystems.generic.roller;

import static edu.wpi.first.units.Units.Millivolts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.*;
import edu.wpi.team190.gompeilib.core.GompeiLib;
import edu.wpi.team190.gompeilib.core.utility.phoenix.PhoenixUtil;
import java.util.ArrayList;

public class GenericRollerIOTalonFX implements GenericRollerIO {
  protected final TalonFX talonFX;
  private final TalonFX[] followerTalonFX;

  private final StatusSignal<Angle> positionRotations;
  private final StatusSignal<AngularVelocity> velocityRotationsPerSecond;
  private final ArrayList<StatusSignal<Voltage>> appliedVolts;
  private final ArrayList<StatusSignal<Current>> supplyCurrentAmps;
  private final ArrayList<StatusSignal<Current>> torqueCurrentAmps;
  private final ArrayList<StatusSignal<Temperature>> temperatureCelsius;

  private StatusSignal<?>[] statusSignals;

  private final TalonFXConfiguration talonFXConfiguration;

  private final VoltageOut voltageRequest;

  protected GenericRollerConstants constants;

  public GenericRollerIOTalonFX(GenericRollerConstants constants) {
    talonFX = new TalonFX(constants.leaderCANID, constants.canBus);
    followerTalonFX =
        new TalonFX
            [constants.alignedFollowerCANIDs.size() + constants.opposedFollowerCANIDs.size()];

    talonFXConfiguration = new TalonFXConfiguration();

    talonFXConfiguration.MotorOutput.withInverted(constants.leaderInvertedValue);
    talonFXConfiguration.MotorOutput.NeutralMode = constants.neutralMode;

    talonFXConfiguration
        .CurrentLimits
        .withSupplyCurrentLimit(constants.currentLimits.supplyCurrentLimit())
        .withSupplyCurrentLimitEnable(true)
        .withStatorCurrentLimit(constants.currentLimits.statorCurrentLimit())
        .withStatorCurrentLimitEnable(true);
    talonFXConfiguration.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.25;

    talonFXConfiguration.Feedback.SensorToMechanismRatio = constants.rollerMotorGearRatio;

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

    voltageRequest = new VoltageOut(0.0).withEnableFOC(constants.enableFOC);

    this.constants = constants;
  }

  @Override
  public void updateInputs(GenericRollerIOInputs inputs) {
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
  }

  @Override
  public void setVoltageGoal(Voltage voltageGoal) {
    talonFX.setControl(voltageRequest.withOutput(voltageGoal));
  }

  @Override
  public boolean atVoltageGoal(Voltage referenceVoltage) {
    return appliedVolts.get(0).isNear(referenceVoltage, Millivolts.of(500));
  }
}
