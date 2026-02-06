package frc.robot.subsystems.v0_Funky.feeder;

import static edu.wpi.first.units.Units.KilogramSquareMeters;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.team190.gompeilib.subsystems.generic.roller.GenericRollerConstants;

public class FeederConstants {
  public static final GenericRollerConstants FEEDER_CONSTANTS =
      GenericRollerConstants.builder()
          .ROLLER_CAN_ID(20)
          .SUPPLY_CURRENT_LIMIT(40.0)
          .ROLLER_GEARBOX(DCMotor.getKrakenX60Foc(1))
          .ROLLER_MOTOR_GEAR_RATIO(1.0 / 2.0)
          .MOMENT_OF_INERTIA(MomentOfInertia.ofBaseUnits(0.004, KilogramSquareMeters))
          .build();
}
