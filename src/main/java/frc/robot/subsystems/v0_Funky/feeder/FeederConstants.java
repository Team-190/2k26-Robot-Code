package frc.robot.subsystems.v0_Funky.feeder;

import static edu.wpi.first.units.Units.KilogramSquareMeters;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.team190.gompeilib.subsystems.generic.roller.GenericRollerConstants;

public class FeederConstants {
  public static final GenericRollerConstants FEEDER_CONSTANTS =
      GenericRollerConstants.builder()
          .withRollerCANID(20)
          .withSupplyCurrentLimit(40.0)
          .withRollerGearbox(DCMotor.getKrakenX60Foc(1))
          .withRollerMotorGearRatio(1.0 / 2.0)
          .withNeutralMode(NeutralModeValue.Brake)
          .withMomentOfInertia(MomentOfInertia.ofBaseUnits(0.004, KilogramSquareMeters))
          .build();
}
