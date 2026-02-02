package frc.robot.subsystems.v1_Gamma.intake;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.team190.gompeilib.subsystems.generic.roller.GenericRollerConstants;

public class V1_GammaIntakeConstants {
  public static final int CAN_IDS_TOP = 20;
  public static final boolean ON_CANIVORE_TOP = true;
  public static final double CURRENT_LIMIT_TOP = 30.0;
  public static final MomentOfInertia MOMENT_OF_INERTIA_TOP = Units.KilogramSquareMeters.of(67);
  public static final double GEAR_RATIO_TOP = 1.0;
  public static final DCMotor GEARBOX_TOP = new DCMotor(0, 0, 0, 0, 0, 0);

  public static final GenericRollerConstants INTAKE_ROLLER_CONSTANTS_TOP =
      new GenericRollerConstants(
          CAN_IDS_TOP,
          CURRENT_LIMIT_TOP,
          GEARBOX_TOP,
          GEAR_RATIO_TOP,
          MOMENT_OF_INERTIA_TOP,
          ON_CANIVORE_TOP);

  public static final int CAN_IDS_BOTTOM = 21;
  public static final boolean ON_CANIVORE_BOTTOM = true;
  public static final double CURRENT_LIMIT_BOTTOM = 30.0;
  public static final MomentOfInertia MOMENT_OF_INERTIA_BOTTOM = Units.KilogramSquareMeters.of(67);
  public static final double GEAR_RATIO_BOTTOM = 1.0;
  public static final DCMotor GEARBOX_BOTTOM = new DCMotor(0, 0, 0, 0, 0, 0);

  public static final GenericRollerConstants INTAKE_ROLLER_CONSTANTS_BOTTOM =
      new GenericRollerConstants(
          CAN_IDS_BOTTOM,
          CURRENT_LIMIT_BOTTOM,
          GEARBOX_BOTTOM,
          GEAR_RATIO_BOTTOM,
          MOMENT_OF_INERTIA_BOTTOM,
          ON_CANIVORE_BOTTOM);
}
