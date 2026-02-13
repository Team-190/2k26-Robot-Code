package frc.robot.subsystems.v1_DoomSpiral.spindexer;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.ChassisReference;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.team190.gompeilib.subsystems.generic.roller.GenericRollerConstants;

public class V1_DoomSpiralSpindexerConstants {
  public static final int SPINDEXER_MOTOR_CAN_ID;
  public static final double SPINDEXER_GEAR_RATIO;
  public static final double SPINDEXER_STATOR_CURRENT_LIMIT;
  public static final double SPINDEXER_SUPPLY_CURRENT_LIMIT;
  public static final double SPINDEXER_MOMENT_OF_INERTIA;
  public static final DCMotor SPINDEXER_MOTOR_CONFIG;
  public static final InvertedValue SPINDEXER_INVERTED_VALUE;
  public static final ChassisReference SPINDEXER_ORIENTATION;
  public static final CANBus SPINDEXER_CAN_BUS;
  public static final NeutralModeValue SPINDEXER_NEUTRAL_MODE;

  public static final double SPINDEXER_VOLTAGE;
  public static final double SPINDEXER_SLOW_VOLTAGE;
  public static final double SPINDEXER_INCREMENT_VOLTAGE;

  static {
    SPINDEXER_MOTOR_CAN_ID = 40;
    SPINDEXER_GEAR_RATIO = 1.0;
    SPINDEXER_STATOR_CURRENT_LIMIT = 40.0;
    SPINDEXER_SUPPLY_CURRENT_LIMIT = 40.0;
    SPINDEXER_MOTOR_CONFIG = DCMotor.getKrakenX60Foc(1);
    SPINDEXER_MOMENT_OF_INERTIA = 0.0271553222;
    SPINDEXER_INVERTED_VALUE = InvertedValue.Clockwise_Positive;
    SPINDEXER_ORIENTATION = ChassisReference.CounterClockwise_Positive;
    SPINDEXER_CAN_BUS = CANBus.roboRIO();
    SPINDEXER_NEUTRAL_MODE = NeutralModeValue.Coast;

    SPINDEXER_VOLTAGE = 12.0;
    SPINDEXER_SLOW_VOLTAGE = 2.0;
    SPINDEXER_INCREMENT_VOLTAGE = 0.25;
  }

  public static final GenericRollerConstants KICKER_ROLLER_CONSTANTS =
      GenericRollerConstants.builder()
          .withRollerCANID(41)
          .withSupplyCurrentLimit(30.0)
          .withRollerGearbox(DCMotor.getKrakenX44(1))
          .withRollerMotorGearRatio(1.0)
          .withNeutralMode(NeutralModeValue.Coast)
          .withMomentOfInertia(Units.KilogramSquareMeters.of(0.0000559571))
          .withCanBus(CANBus.roboRIO())
          .build();

  public static final GenericRollerConstants FEEDER_ROLLER_CONSTANTS =
      GenericRollerConstants.builder()
          .withRollerCANID(42)
          .withSupplyCurrentLimit(30.0)
          .withRollerGearbox(DCMotor.getKrakenX60Foc(1))
          .withRollerMotorGearRatio(1.0)
          .withNeutralMode(NeutralModeValue.Brake)
          .withMomentOfInertia(Units.KilogramSquareMeters.of(0.0001710116))
          .withCanBus(CANBus.roboRIO())
          .build();
}
