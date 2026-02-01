package frc.robot.subsystems.v1_Gamma.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.team190.gompeilib.subsystems.generic.roller.GenericRollerConstants;

public class V1_GammaIntakeConstants {
  public static final int CAN_IDS_TOP = 16;
  public static final boolean ON_CANIVORE_TOP = true;
  public static final double CURRENT_LIMIT_TOP = 30.0;
  public static final MomentOfInertia MOMENT_OF_INERTIA_TOP = Units.KilogramSquareMeters.of(67);
  public static final double GEAR_RATIO_TOP = 1.0;
  public static final DCMotor GEARBOX_TOP = new DCMotor(0, 0, 0, 0, 0, 0);
  public static final Rotation2d STOW_ANGLE =
      Rotation2d.fromDegrees(90); // TODO: Put in actual value.
  public static final Rotation2d DEPLOY_ANGLE =
      Rotation2d.fromDegrees(-90); // TODO: Put in actual value.
  public static final GenericRollerConstants INTAKE_ROLLER_CONSTANTS_TOP =
      new GenericRollerConstants(
          CAN_IDS_TOP,
          CURRENT_LIMIT_TOP,
          GEARBOX_TOP,
          GEAR_RATIO_TOP,
          MOMENT_OF_INERTIA_TOP,
          ON_CANIVORE_TOP);

  public static final int CAN_IDS_BOTTOM = 17;
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

  public static final Translation3d INTAKE_GLOBAL_OFFSET =
      new Translation3d(1, 1, 1); // TODO: Put in actual offset.
}
