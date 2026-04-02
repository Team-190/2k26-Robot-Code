package frc.robot.subsystems.v2_Delta.hopper;

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

public class V2_DeltaHopperConstants {
  public static final Voltage ROLLER_FLOOR_FEED_VOLTAGE;
  public static final Voltage BALL_TUNNEL_FEED_VOLTAGE;
  public static final Voltage OUTTAKE_VOLTAGE;

  public static final GenericRollerConstants ROLLERFLOOR_CONSTANTS;
  public static final GenericRollerConstants BALLTUNNEL_CONSTANTS;

  static {
    switch (RobotConfig.ROBOT) {
      case V2_DELTA:
      case V2_DELTA_SIM:
      default:
        ROLLER_FLOOR_FEED_VOLTAGE = Volts.of(11.0);
        BALL_TUNNEL_FEED_VOLTAGE = Volts.of(11.0);
        OUTTAKE_VOLTAGE = Volts.of(-11.0);

        ROLLERFLOOR_CONSTANTS =
            GenericRollerConstants.builder()
                .withLeaderCANID(50)
                .withCurrentLimits(
                    CurrentLimits.builder()
                        .withSupplyCurrentLimit(Amps.of(20.0))
                        .withStatorCurrentLimit(Amps.of(30.0))
                        .build())
                .withNeutralMode(NeutralModeValue.Coast)
                .withRollerGearbox(DCMotor.getKrakenX60Foc(1))
                .withRollerMotorGearRatio(9.0 / 4.0)
                .withLeaderInvertedValue(InvertedValue.Clockwise_Positive)
                .withOpposedFollowerCANID(51)
                .withMomentOfInertia(Units.KilogramSquareMeters.of(0.0001))
                .withVoltageOffsetStep(Volts.of(0.20))
                .withCanBus(CANBus.roboRIO())
                .build();
        BALLTUNNEL_CONSTANTS =
            GenericRollerConstants.builder()
                .withLeaderCANID(52)
                .withCurrentLimits(
                    CurrentLimits.builder()
                        .withSupplyCurrentLimit(Amps.of(20.0))
                        .withStatorCurrentLimit(Amps.of(30.0))
                        .build())
                .withNeutralMode(NeutralModeValue.Coast)
                .withRollerGearbox(DCMotor.getKrakenX60Foc(1))
                .withRollerMotorGearRatio(9.0 / 4.0)
                .withLeaderInvertedValue(InvertedValue.Clockwise_Positive)
                .withOpposedFollowerCANID(53)
                .withMomentOfInertia(Units.KilogramSquareMeters.of(0.0001))
                .withVoltageOffsetStep(Volts.of(0.20))
                .withCanBus(CANBus.roboRIO())
                .build();
        break;
    }
  }
}
