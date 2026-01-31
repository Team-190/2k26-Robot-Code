package frc.robot.subsystems.v1_Gamma.spindexer;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.ChassisReference;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.team190.gompeilib.subsystems.generic.roller.GenericRollerConstants;

public class V1_GammaSpindexerConstants {
  public static final int MOTOR_CAN_ID;
  public static final double GEAR_RATIO;
  public static final double STATOR_CURRENT_LIMIT;
  public static final double SUPPLY_CURRENT_LIMIT;
  public static final double MOMENT_OF_INERTIA;
  public static final DCMotor MOTOR_CONFIG;
  public static final InvertedValue SPINDEXER_INVERTED_VALUE;
  public static final ChassisReference SPINDEXER_ORIENTATION;
  public static final boolean IS_CAN_FD;

  static {
    MOTOR_CAN_ID = 0;
    GEAR_RATIO = 0;
    STATOR_CURRENT_LIMIT = 0;
    SUPPLY_CURRENT_LIMIT = 0;
    MOTOR_CONFIG = DCMotor.getKrakenX60Foc(1); // TODO: Replace with actual motor.
    MOMENT_OF_INERTIA = 0;
    SPINDEXER_INVERTED_VALUE = InvertedValue.Clockwise_Positive; // TODO: Replace with actual value.
    SPINDEXER_ORIENTATION =
        ChassisReference.CounterClockwise_Positive; // TODO: Replace with actual value
    IS_CAN_FD = false;
  }

  public final int FEEDER_CAN_ID = 0;
  public final double FEEDER_SUPPLY_CURRENT_LIMIT = 0;
  public final DCMotor FEEDER_GEARBOX =
      DCMotor.getKrakenX60Foc(1); // TODO: Replace with actual motor.
  public final double FEEDER_MOTOR_GEAR_RATIO = 1;
  public final MomentOfInertia FEEDER_MOMENT_OF_INERTIA = Units.KilogramSquareMeters.of(67);
  public final boolean FEEDER_ON_CANIVORE = false;

  GenericRollerConstants feederConstants =
      new GenericRollerConstants(
          FEEDER_CAN_ID,
          FEEDER_SUPPLY_CURRENT_LIMIT,
          FEEDER_GEARBOX,
          FEEDER_MOTOR_GEAR_RATIO,
          FEEDER_MOMENT_OF_INERTIA,
          FEEDER_ON_CANIVORE);
}
