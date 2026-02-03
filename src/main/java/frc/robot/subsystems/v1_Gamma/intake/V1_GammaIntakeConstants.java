package frc.robot.subsystems.v1_Gamma.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.team190.gompeilib.core.utility.LoggedTunableNumber;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveDriveConstants.DriveConfig;
import edu.wpi.team190.gompeilib.subsystems.generic.roller.GenericRollerConstants;
import frc.robot.subsystems.shared.linkage.FourBarLinkageConstants;
import frc.robot.subsystems.shared.linkage.FourBarLinkageConstants.Constraints;
import frc.robot.subsystems.shared.linkage.FourBarLinkageConstants.Gains;
import frc.robot.subsystems.shared.linkage.FourBarLinkageConstants.LinkBounds;
import frc.robot.subsystems.shared.linkage.FourBarLinkageConstants.LinkConstants;
import frc.robot.subsystems.shared.linkage.FourBarLinkageConstants.LinkLengths;
import frc.robot.subsystems.v1_Gamma.V1_GammaConstants;

public class V1_GammaIntakeConstants {
  public static final int CAN_IDS_TOP = 16;
  public static final boolean ON_CANIVORE_TOP = true;
  public static final double CURRENT_LIMIT_TOP = 30.0;
  public static final MomentOfInertia MOMENT_OF_INERTIA_TOP = Units.KilogramSquareMeters.of(.004);
  public static final double GEAR_RATIO_TOP = 1.0;
  public static final DCMotor GEARBOX_TOP = DCMotor.getKrakenX44Foc(1);
  public static final Rotation2d STOW_ANGLE = Rotation2d.fromDegrees(9);
  public static final Rotation2d DEPLOY_ANGLE = Rotation2d.fromDegrees(170);
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
  public static final MomentOfInertia MOMENT_OF_INERTIA_BOTTOM =
      Units.KilogramSquareMeters.of(.004);
  public static final double GEAR_RATIO_BOTTOM = 1.0;
  public static final DCMotor GEARBOX_BOTTOM = DCMotor.getKrakenX44Foc(1);

  public static final GenericRollerConstants INTAKE_ROLLER_CONSTANTS_BOTTOM =
      new GenericRollerConstants(
          CAN_IDS_BOTTOM,
          CURRENT_LIMIT_BOTTOM,
          GEARBOX_BOTTOM,
          GEAR_RATIO_BOTTOM,
          MOMENT_OF_INERTIA_BOTTOM,
          ON_CANIVORE_BOTTOM);

  public static final Translation3d LINKAGE_OFFSET =
      new Translation3d(1, 1, 1); // TODO: Put in actual offset.

  public static final boolean IS_CAN_FD = false;
  public static final int MOTOR_CAN_ID = 8; // TODO: set correct ID
  public static final int CAN_CODER_CAN_ID = 9; // TODO: set correct ID
  public static final int GEAR_RATIO = 1;
  public static final int SUPPLY_CURRENT_LIMIT = 40; // TODO: Set actual value.
  public static final int STATOR_CURRENT_LIMIT = 40; // TODO: Set actual value.
  public static final double MOMENT_OF_INERTIA = 0.004;
  public static final DCMotor MOTOR_CONFIG = DCMotor.getKrakenX60Foc(1);
  public static final double LENGTH_METERS = 0.3;
  public static final double MIN_ANGLE = Rotation2d.fromDegrees(9).getRadians();
  public static final double MAX_ANGLE = Rotation2d.fromDegrees(170).getRadians();
  public static final Rotation2d INTAKE_ANGLE_OFFSET =
      Rotation2d.fromDegrees(-30.9603232217); // TODO: Figure out the actual angle offset between
  // points A and D on the intake.

  public static final boolean LINKAGE_ENABLE_FOC = false;

  public static final double PIN_LENGTH =
      Units.Inches.of(6.092560)
          .in(Units.Meters); // TODO: set to distance of pin from point of rotation.

  public static final Gains GAINS =
      new Gains(
          new LoggedTunableNumber("Linkage/KP", 1.0),
          new LoggedTunableNumber("Linkage/KD", 0.1),
          new LoggedTunableNumber("Linkage/KS", 0.0),
          new LoggedTunableNumber("Linkage/KV", 0.0),
          new LoggedTunableNumber("Linkage/KA", 0.0));
  public static final Constraints CONSTRAINTS =
      new Constraints(
          new LoggedTunableNumber("Linkage/Max Velocity", 1.0),
          new LoggedTunableNumber("Linkage/Max Acceleration", 1.0),
          new LoggedTunableNumber(
              "Linkage/Goal Tolerance", Rotation2d.fromDegrees(1.0).getRadians()));

  public static final DriveConfig DRIVE_CONFIG = V1_GammaConstants.DRIVE_CONFIG;

  public static final LinkLengths LINK_LENGTHS =
      new LinkLengths(
          Units.Inches.of(6.950079).in(Units.Meters),
          Units.Inches.of(8.809879).in(Units.Meters),
          Units.Inches.of(8.284456).in(Units.Meters),
          Units.Inches.of(6.4213032).in(Units.Meters));

  public static final LinkBounds LINK_BOUNDS =
      new LinkBounds(
          Units.Inches.of(0.810921).in(Units.Meters),
          Units.Inches.of(2.86545).in(Units.Meters),
          Units.Inches.of(4.752162).in(Units.Meters),
          Units.Inches.of(6.46545).in(Units.Meters));

  public static final LinkConstants LINK_CONST =
      new LinkConstants(
          Units.Inches.of(6.092560).in(Units.Meters),
          Units.Inches.of(2.446682).in(Units.Meters),
          Units.Inches.of(5.376661).in(Units.Meters));

  public static final FourBarLinkageConstants LINKAGE_CONSTANTS =
      new FourBarLinkageConstants(
          IS_CAN_FD,
          MOTOR_CAN_ID,
          CAN_CODER_CAN_ID,
          INTAKE_ANGLE_OFFSET,
          GEAR_RATIO,
          SUPPLY_CURRENT_LIMIT,
          STATOR_CURRENT_LIMIT,
          MOMENT_OF_INERTIA,
          MOTOR_CONFIG,
          LENGTH_METERS,
          MIN_ANGLE,
          MAX_ANGLE,
          GAINS,
          CONSTRAINTS,
          DRIVE_CONFIG,
          LINK_LENGTHS,
          LINK_BOUNDS,
          LINK_CONST,
          PIN_LENGTH,
          LINKAGE_OFFSET,
          LINKAGE_ENABLE_FOC);
}
