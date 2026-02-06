package frc.robot.subsystems.v1_Gamma.intake;

import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.team190.gompeilib.core.utility.LoggedTunableNumber;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveDriveConstants.DriveConfig;
import edu.wpi.team190.gompeilib.subsystems.generic.roller.GenericRollerConstants;
import frc.robot.subsystems.shared.fourbarlinkage.FourBarLinkageConstants;
import frc.robot.subsystems.shared.fourbarlinkage.FourBarLinkageConstants.Constraints;
import frc.robot.subsystems.shared.fourbarlinkage.FourBarLinkageConstants.Gains;
import frc.robot.subsystems.shared.fourbarlinkage.FourBarLinkageConstants.LinkBounds;
import frc.robot.subsystems.shared.fourbarlinkage.FourBarLinkageConstants.LinkConstants;
import frc.robot.subsystems.shared.fourbarlinkage.FourBarLinkageConstants.LinkLengths;
import frc.robot.subsystems.v1_Gamma.V1_GammaConstants;

public class V1_GammaIntakeConstants {
  public static final int CAN_ID_TOP = 20;
  public static final double CURRENT_LIMIT_TOP = 40.0;
  public static final MomentOfInertia MOMENT_OF_INERTIA_TOP = Units.KilogramSquareMeters.of(0.0004);
  public static final double GEAR_RATIO_TOP = 2.67; // might be 8/3
  public static final DCMotor GEARBOX_TOP = DCMotor.getKrakenX60(1);

  public static final Rotation2d STOW_ANGLE = Rotation2d.fromDegrees(9);
  public static final Rotation2d DEPLOY_ANGLE = Rotation2d.fromDegrees(170);
  public static final GenericRollerConstants INTAKE_ROLLER_CONSTANTS_TOP =
      GenericRollerConstants.builder()
          .ROLLER_CAN_ID(CAN_ID_TOP)
          .SUPPLY_CURRENT_LIMIT(CURRENT_LIMIT_TOP)
          .ROLLER_GEARBOX(GEARBOX_TOP)
          .ROLLER_MOTOR_GEAR_RATIO(GEAR_RATIO_TOP)
          .MOMENT_OF_INERTIA(MOMENT_OF_INERTIA_TOP)
          .build();

  public static final int CAN_ID_BOTTOM = 21;
  public static final double CURRENT_LIMIT_BOTTOM = 40.0;
  public static final MomentOfInertia MOMENT_OF_INERTIA_BOTTOM =
      Units.KilogramSquareMeters.of(0.0004);
  public static final double GEAR_RATIO_BOTTOM = 2.67;
  public static final DCMotor GEARBOX_BOTTOM = DCMotor.getKrakenX60(1);

  public static final GenericRollerConstants INTAKE_ROLLER_CONSTANTS_BOTTOM =
      GenericRollerConstants.builder()
          .ROLLER_CAN_ID(CAN_ID_BOTTOM)
          .SUPPLY_CURRENT_LIMIT(CURRENT_LIMIT_BOTTOM)
          .ROLLER_GEARBOX(GEARBOX_BOTTOM)
          .ROLLER_MOTOR_GEAR_RATIO(GEAR_RATIO_BOTTOM)
          .MOMENT_OF_INERTIA(MOMENT_OF_INERTIA_BOTTOM)
          .build();

  public static final Translation3d LINKAGE_OFFSET = new Translation3d(0.381, 0.141, 0.276);

  public static final boolean IS_CAN_FD = false;
  public static final int MOTOR_CAN_ID = 22;
  public static final int CAN_CODER_CAN_ID = 23;

  public static final SensorDirectionValue CANCODER_SENSOR_DIRECTION =
      SensorDirectionValue.CounterClockwise_Positive; // TODO: set correct direction
  public static final int GEAR_RATIO = 1;
  public static final int SUPPLY_CURRENT_LIMIT = 40;
  public static final int STATOR_CURRENT_LIMIT = 40;

  public static final double MOMENT_OF_INERTIA = 0.004;
  public static final DCMotor MOTOR_CONFIG = DCMotor.getKrakenX60Foc(1);
  public static final Rotation2d INTAKE_ANGLE_OFFSET = Rotation2d.fromDegrees(-30.9603232217);

  public static final Rotation2d ZERO_OFFSET = Rotation2d.kPi;
  public static final Rotation2d MIN_ANGLE = Rotation2d.fromDegrees(9);
  public static final Rotation2d MAX_ANGLE = Rotation2d.fromDegrees(170);
  // points A and D on the intake.

  public static final boolean LINKAGE_ENABLE_FOC = false;

  public static final double PIN_LENGTH = Units.Inches.of(6.125).in(Units.Meters);

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
      FourBarLinkageConstants.builder()
          .CANCODER_SENSOR_DIRECTION(CANCODER_SENSOR_DIRECTION)
          .CAN_CODER_CAN_ID(CAN_CODER_CAN_ID)
          .CAN_CODER_CAN_ID(CAN_CODER_CAN_ID)
          .CONSTRAINTS(CONSTRAINTS)
          .GAINS(GAINS)
          .GEAR_RATIO(GEAR_RATIO)
          .INTAKE_ANGLE_OFFSET(INTAKE_ANGLE_OFFSET)
          .LINKAGE_OFFSET(LINKAGE_OFFSET)
          .LINK_BOUNDS(LINK_BOUNDS)
          .LINK_CONST(LINK_CONST)
          .LINK_LENGTHS(LINK_LENGTHS)
          .MAX_ANGLE(MAX_ANGLE)
          .MIN_ANGLE(MIN_ANGLE)
          .MOMENT_OF_INERTIA(MOMENT_OF_INERTIA)
          .MOTOR_CAN_ID(MOTOR_CAN_ID)
          .MOTOR_CONFIG(MOTOR_CONFIG)
          .PIN_LENGTH(PIN_LENGTH)
          .STATOR_CURRENT_LIMIT(STATOR_CURRENT_LIMIT)
          .SUPPLY_CURRENT_LIMIT(SUPPLY_CURRENT_LIMIT)
          .ZERO_OFFSET(ZERO_OFFSET)
          .build();
}
