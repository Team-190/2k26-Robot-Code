package frc.robot.subsystems.feeder;

import static edu.wpi.first.units.Units.KilogramSquareMeters;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.team190.gompeilib.subsystems.generic.roller.GenericRollerConstants;

public class FeederConstants {
  public static final GenericRollerConstants FEEDER_CONSTANTS =
      new GenericRollerConstants(
          42,
          40.0,
          DCMotor.getKrakenX60Foc(1),
          1.0,
          MomentOfInertia.ofBaseUnits(0.004, KilogramSquareMeters),
          false);
}
