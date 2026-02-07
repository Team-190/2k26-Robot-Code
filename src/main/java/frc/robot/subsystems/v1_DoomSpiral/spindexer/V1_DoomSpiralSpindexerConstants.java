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
  public static final CANBus CAN_LOOP;

  static {
    MOTOR_CAN_ID = 40;
    GEAR_RATIO = 0;
    STATOR_CURRENT_LIMIT = 0;
    SUPPLY_CURRENT_LIMIT = 0;
    MOTOR_CONFIG = DCMotor.getKrakenX60Foc(1);
    MOMENT_OF_INERTIA = 0.0271553222;
    SPINDEXER_INVERTED_VALUE = InvertedValue.Clockwise_Positive;
    SPINDEXER_ORIENTATION = ChassisReference.CounterClockwise_Positive;
    CAN_LOOP = new CANBus();
  }

  public static final int KICKER_CAN_ID = 41;
  public static final double KICKER_CURRENT_LIMIT = 30.0;
  public static final DCMotor KICKER_GEARBOX =
      DCMotor.getKrakenX44Foc(1); // I hope we dont need 41 motors
  public static final double KICKER_GEAR_RATIO = 1.0;
  public static final MomentOfInertia KICKER_MOMENT_OF_INERTIA =
      Units.KilogramSquareMeters.of(0.0000559571);

  public static final GenericRollerConstants KICKER_ROLLER_CONSTANTS =
      GenericRollerConstants.builder()
          .ROLLER_CAN_ID(KICKER_CAN_ID)
          .SUPPLY_CURRENT_LIMIT(KICKER_CURRENT_LIMIT)
          .ROLLER_GEARBOX(KICKER_GEARBOX)
          .ROLLER_MOTOR_GEAR_RATIO(KICKER_GEAR_RATIO)
          .MOMENT_OF_INERTIA(KICKER_MOMENT_OF_INERTIA)
          .build();

  public static final int FEEDER_CAN_ID = 42;
  public static final double FEEDER_SUPPLY_CURRENT_LIMIT = 30.0;
  public static final DCMotor FEEDER_GEARBOX = DCMotor.getKrakenX60Foc(1);
  public static final double FEEDER_MOTOR_GEAR_RATIO = 1;
  public static final MomentOfInertia FEEDER_MOMENT_OF_INERTIA =
      Units.KilogramSquareMeters.of(0.0001710116);

  public static final GenericRollerConstants FEEDER_CONSTANTS =
      GenericRollerConstants.builder()
          .ROLLER_CAN_ID(FEEDER_CAN_ID)
          .SUPPLY_CURRENT_LIMIT(FEEDER_SUPPLY_CURRENT_LIMIT)
          .ROLLER_GEARBOX(FEEDER_GEARBOX)
          .ROLLER_MOTOR_GEAR_RATIO(FEEDER_MOTOR_GEAR_RATIO)
          .MOMENT_OF_INERTIA(FEEDER_MOMENT_OF_INERTIA)
          .build();
}
