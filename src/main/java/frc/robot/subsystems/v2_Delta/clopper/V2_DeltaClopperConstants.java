package frc.robot.subsystems.v2_Delta.clopper;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.team190.gompeilib.core.utility.control.CurrentLimits;
import edu.wpi.team190.gompeilib.subsystems.generic.roller.GenericRollerConstants;
import frc.robot.RobotConfig;

public class V2_DeltaClopperConstants {
  public static final Voltage ROLLER_FLOOR_FEED_VOLTAGE;
  public static final Voltage BALL_TUNNEL_FEED_VOLTAGE;
  public static final Voltage ROLLER_FLOOR_FEED_VOLTAGE_SLOW;
  public static final Voltage OUTTAKE_VOLTAGE;

  public static final GenericRollerConstants ROLLER_FLOOR_CONSTANTS;
  public static final GenericRollerConstants BALL_TUNNEL_CONSTANTS;

  public static final int BEAM_BREAK_ID;

  static {
    switch (RobotConfig.ROBOT) {
      case V2_DELTA:
      case V2_DELTA_SIM:
      default:
        ROLLER_FLOOR_FEED_VOLTAGE = Volts.of(9.0);
        BALL_TUNNEL_FEED_VOLTAGE = Volts.of(11.0);
        ROLLER_FLOOR_FEED_VOLTAGE_SLOW = Volts.of(5.0);
        OUTTAKE_VOLTAGE = Volts.of(-10);
        BEAM_BREAK_ID = 0;

        ROLLER_FLOOR_CONSTANTS =
            GenericRollerConstants.builder()
                .withLeaderCANID(32)
                .withCurrentLimits(
                    CurrentLimits.builder()
                        .withSupplyCurrentLimit(Amps.of(40.0))
                        .withStatorCurrentLimit(Amps.of(80.0))
                        .build())
                .withNeutralMode(NeutralModeValue.Coast)
                .withRollerGearbox(DCMotor.getKrakenX60Foc(1))
                .withRollerMotorGearRatio(42.0)
                .withLeaderInvertedValue(InvertedValue.Clockwise_Positive)
                .withMomentOfInertia(Units.KilogramSquareMeters.of(0.0001))
                .withVoltageOffsetStep(Volts.of(0.20))
                .withCanBus(CANBus.roboRIO())
                .withEnableFOC(false)
                .build();
        BALL_TUNNEL_CONSTANTS =
            GenericRollerConstants.builder()
                .withLeaderCANID(31) // Bottom Ball Tunnel
                .withCurrentLimits(
                    CurrentLimits.builder()
                        .withSupplyCurrentLimit(Amps.of(40.0))
                        .withStatorCurrentLimit(Amps.of(80.0))
                        .build())
                .withNeutralMode(NeutralModeValue.Coast)
                .withRollerGearbox(DCMotor.getKrakenX60Foc(1))
                .withRollerMotorGearRatio(42.0)
                .withLeaderInvertedValue(InvertedValue.Clockwise_Positive)
                .withAlignedFollowerCANID(30) // Top Ball Tunnel
                .withMomentOfInertia(Units.KilogramSquareMeters.of(0.0001))
                .withVoltageOffsetStep(Volts.of(0.20))
                .withCanBus(CANBus.roboRIO())
                .withEnableFOC(false)
                .build();
        break;
    }
  }
}
