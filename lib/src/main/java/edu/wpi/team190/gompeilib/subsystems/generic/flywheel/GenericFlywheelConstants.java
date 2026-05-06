package edu.wpi.team190.gompeilib.subsystems.generic.flywheel;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.team190.gompeilib.core.utility.control.CurrentLimits;
import edu.wpi.team190.gompeilib.core.utility.control.Gains;
import edu.wpi.team190.gompeilib.core.utility.control.constraints.AngularVelocityConstraints;
import java.util.Set;
import lombok.Builder;
import lombok.NonNull;
import lombok.Singular;

@Builder(setterPrefix = "with")
public class GenericFlywheelConstants {

  @NonNull public final Integer leaderCANID;
  @NonNull public final InvertedValue leaderInversion;

  @NonNull public final CANBus canBus;
  @NonNull public final Boolean enableFOC;

  @NonNull public final CurrentLimits currentLimit;
  @NonNull public final Double momentOfInertia;
  @NonNull public final Double gearRatio;

  @NonNull public final DCMotor motorConfig;

  @NonNull public final Gains voltageGains;
  @NonNull public final Gains torqueGains;
  @NonNull public final AngularVelocityConstraints constraints;

  @Singular(value = "alignedFollowerCANID")
  @NonNull
  public final Set<Integer> alignedFollowerCANIDs;

  @Singular(value = "opposedFollowerCANID")
  @NonNull
  public final Set<Integer> opposedFollowerCANIDs;

  @NonNull public final AngularVelocity velocityOffsetStep;

  @NonNull public final Voltage voltageOffsetStep;
}
