package frc.robot.subsystems.v1_DoomSpiral.spindexer;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.ChassisReference;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.team190.gompeilib.subsystems.generic.roller.GenericRollerConstants;

public class V1_DoomSpiralSpindexerConstants {
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
    GEAR_RATIO = 1.0;
    STATOR_CURRENT_LIMIT = 0;
    SUPPLY_CURRENT_LIMIT = 0;
    MOTOR_CONFIG = DCMotor.getKrakenX60Foc(1);
    MOMENT_OF_INERTIA = 0.004;
    SPINDEXER_INVERTED_VALUE = InvertedValue.Clockwise_Positive;
    SPINDEXER_ORIENTATION = ChassisReference.CounterClockwise_Positive;
    IS_CAN_FD = false;
  }

  public static final int KICKER_CAN_ID = 40;
  public static final double KICKER_CURRENT_LIMIT = 30.0;
  public static final DCMotor KICKER_GEARBOX = DCMotor.getKrakenX44(KICKER_CAN_ID);
  public static final double KICKER_GEAR_RATIO = 1.0;
  public static final MomentOfInertia KICKER_MOMENT_OF_INERTIA = Units.KilogramSquareMeters.of(10);
  public static final boolean KICKER_ON_CANIVORE = false;

  public static final GenericRollerConstants KICKER_ROLLER_CONSTANTS =
      new GenericRollerConstants(
          KICKER_CAN_ID,
          KICKER_CURRENT_LIMIT,
          KICKER_GEARBOX,
          KICKER_GEAR_RATIO,
          KICKER_MOMENT_OF_INERTIA,
          KICKER_ON_CANIVORE);

  public static final int FEEDER_CAN_ID = 41;
  public static final double FEEDER_SUPPLY_CURRENT_LIMIT = 30.0;
  public static final DCMotor FEEDER_GEARBOX = DCMotor.getKrakenX60Foc(1);
  public static final double FEEDER_MOTOR_GEAR_RATIO = 1;
  public static final MomentOfInertia FEEDER_MOMENT_OF_INERTIA =
      Units.KilogramSquareMeters.of(0.004);
  public static final boolean FEEDER_ON_CANIVORE = false;

  public static final GenericRollerConstants FEEDER_ROLLER_CONSTANTS =
      new GenericRollerConstants(
          FEEDER_CAN_ID,
          FEEDER_SUPPLY_CURRENT_LIMIT,
          FEEDER_GEARBOX,
          FEEDER_MOTOR_GEAR_RATIO,
          FEEDER_MOMENT_OF_INERTIA,
          FEEDER_ON_CANIVORE);
}
