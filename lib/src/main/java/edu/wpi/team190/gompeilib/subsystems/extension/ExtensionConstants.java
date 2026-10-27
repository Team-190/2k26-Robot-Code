package edu.wpi.team190.gompeilib.subsystems.extension;

import static org.wpilib.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.team190.gompeilib.core.utility.control.Gains;
import edu.wpi.team190.gompeilib.core.utility.control.constraints.LinearConstraints;
import java.util.Set;
import lombok.Builder;
import lombok.NonNull;
import lombok.Singular;
import org.wpilib.math.system.DCMotor;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.Voltage;

@Builder(setterPrefix = "with")
public class ExtensionConstants {
  @NonNull public final Integer leaderCANID;
  @NonNull public final InvertedValue leaderInvertedValue;
  @NonNull public final CANBus canBus;
  @NonNull public final Double extensionGearRatio;
  @NonNull public final Double drumRadius;

  @NonNull public final Double extensionSupplyCurrentLimit;
  @NonNull public final Double extensionStatorCurrentLimit;

  @Builder.Default public final Voltage zeroVoltage = Volts.of(1.0);
  @Builder.Default public final Current zeroCurrentThreshold = Amps.of(-20.0);
  @Builder.Default public final Current zeroCurrentEpsilon = Amps.of(5.0);

  @NonNull public final ExtensionParameters extensionParameters;
  @NonNull public final Gains slot0Gains;
  @Builder.Default public final Gains slot1Gains = Gains.builder().build();
  @Builder.Default public final Gains slot2Gains = Gains.builder().build();
  @NonNull public final LinearConstraints constraints;

  @NonNull public final Boolean verticalGravity;

  @Singular(value = "alignedFollowerCANID")
  @NonNull
  public final Set<Integer> alignedFollowerCANIDs;

  @Singular(value = "opposedFollowerCANID")
  @NonNull
  public final Set<Integer> opposedFollowerCANIDs;

  @NonNull public final Voltage voltageOffsetStep;
  @NonNull public final Distance heightOffsetStep;

  @Builder(setterPrefix = "with")
  public record ExtensionParameters(
      @NonNull DCMotor EXTENSION_MOTOR_CONFIG,
      @NonNull Double CARRIAGE_MASS_KG,
      @NonNull Distance MIN_LENGTH,
      @NonNull Distance MAX_LENGTH,
      @NonNull Integer NUM_MOTORS) {}
}
