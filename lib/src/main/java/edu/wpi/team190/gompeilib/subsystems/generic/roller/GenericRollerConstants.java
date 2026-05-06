package edu.wpi.team190.gompeilib.subsystems.generic.roller;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.team190.gompeilib.core.utility.control.CurrentLimits;
import java.util.Set;
import lombok.Builder;
import lombok.NonNull;
import lombok.Singular;

@Builder(setterPrefix = "with")
public class GenericRollerConstants {
  @NonNull public final Integer leaderCANID;
  @NonNull public final InvertedValue leaderInvertedValue;

  @Singular(value = "alignedFollowerCANID")
  @NonNull
  public final Set<Integer> alignedFollowerCANIDs;

  @Singular(value = "opposedFollowerCANID")
  @NonNull
  public final Set<Integer> opposedFollowerCANIDs;

  @NonNull public final CurrentLimits currentLimits;
  @NonNull public final DCMotor rollerGearbox;
  @NonNull public final Double rollerMotorGearRatio;
  @NonNull public final MomentOfInertia momentOfInertia;
  @NonNull public final NeutralModeValue neutralMode;
  @NonNull public final CANBus canBus;

  @NonNull public final Boolean enableFOC;

  @NonNull public final Voltage voltageOffsetStep;
}
