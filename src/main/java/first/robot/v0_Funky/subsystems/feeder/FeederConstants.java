package first.robot.v0_Funky.subsystems.feeder;

import static org.wpilib.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.team190.gompeilib.core.utility.control.CurrentLimits;
import edu.wpi.team190.gompeilib.subsystems.generic.roller.GenericRollerConstants;
import org.wpilib.math.system.DCMotor;
import org.wpilib.units.measure.MomentOfInertia;

public class FeederConstants {
  public static final GenericRollerConstants FEEDER_CONSTANTS =
      GenericRollerConstants.builder()
          .withLeaderCANID(20)
          .withCanBus(new CANBus())
          .withCurrentLimits(
              CurrentLimits.builder()
                  .withSupplyCurrentLimit(Amps.of(20.0))
                  .withStatorCurrentLimit(Amps.of(20.0))
                  .build())
          .withRollerGearbox(DCMotor.getKrakenX60Foc(1))
          .withRollerMotorGearRatio(1.0 / 2.0)
          .withNeutralMode(NeutralModeValue.Brake)
          .withLeaderInvertedValue(InvertedValue.CounterClockwise_Positive)
          .withMomentOfInertia(MomentOfInertia.ofBaseUnits(0.004, KilogramSquareMeters))
          .withVoltageOffsetStep(Volts.of(1))
          .withEnableFOC(false)
          .build();
}
