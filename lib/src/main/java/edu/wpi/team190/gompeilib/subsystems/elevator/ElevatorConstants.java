package edu.wpi.team190.gompeilib.subsystems.elevator;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.team190.gompeilib.core.utility.control.Gains;
import edu.wpi.team190.gompeilib.core.utility.control.constraints.LinearConstraints;
import java.util.Set;
import lombok.Builder;
import lombok.NonNull;
import lombok.Singular;

@Builder(setterPrefix = "with")
public class ElevatorConstants {
  @NonNull public final Integer leaderCANID;
  @NonNull public final CANBus canBus = new CANBus();
  @NonNull public final Double elevatorGearRatio;
  @NonNull public final Double drumRadius;

  @NonNull public final Double elevatorSupplyCurrentLimit;
  @NonNull public final Double elevatorStatorCurrentLimit;

  @NonNull public final ElevatorParameters elevatorParameters;
  @NonNull public final Gains slot0Gains;
  @Builder.Default public final Gains slot1Gains = Gains.builder().build();
  @Builder.Default public final Gains slot2Gains = Gains.builder().build();
  @NonNull public final LinearConstraints constraints;

  @Singular(value = "alignedFollowerCANID")
  @NonNull
  public final Set<Integer> alignedFollowerCANIDs;

  @Singular(value = "opposedFollowerCANID")
  @NonNull
  public final Set<Integer> opposedFollowerCANIDs;

  @NonNull public final Voltage voltageOffsetStep;
  @NonNull public final Distance heightOffsetStep;

  @Builder(setterPrefix = "with")
  public record ElevatorParameters(
      @NonNull DCMotor ELEVATOR_MOTOR_CONFIG,
      @NonNull Double CARRIAGE_MASS_KG,
      @NonNull Distance MIN_HEIGHT,
      @NonNull Distance MAX_HEIGHT,
      @NonNull Integer NUM_MOTORS) {}
}
