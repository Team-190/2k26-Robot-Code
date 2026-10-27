package first.robot.shared.subsystems.intake;

import static org.wpilib.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.team190.gompeilib.core.utility.Setpoint;
import edu.wpi.team190.gompeilib.core.utility.control.CurrentLimits;
import edu.wpi.team190.gompeilib.core.utility.control.Gains;
import edu.wpi.team190.gompeilib.core.utility.control.constraints.AngularPositionConstraints;
import edu.wpi.team190.gompeilib.subsystems.generic.roller.GenericRollerConstants;
import first.robot.RobotConfig;
import first.robot.shared.subsystems.fourbarlinkage.FourBarLinkageConstants;
import first.robot.shared.subsystems.fourbarlinkage.FourBarLinkageConstants.LinkBounds;
import first.robot.shared.subsystems.fourbarlinkage.FourBarLinkageConstants.LinkConstants;
import first.robot.shared.subsystems.fourbarlinkage.FourBarLinkageConstants.LinkLengths;
import java.util.Map;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.math.system.DCMotor;
import org.wpilib.units.AngleUnit;
import org.wpilib.units.Units;

public class IntakeConstants {
  public static final double INTAKE_VOLTAGE;
  public static final double EXTAKE_VOLTAGE;

  public static final Rotation2d LINKAGE_ANGLE_INCREMENT;
  public static final double LINKAGE_SLOW_VOLTAGE;

  public static final GenericRollerConstants INTAKE_ROLLER_CONSTANTS;

  public static final Translation3d LINKAGE_OFFSET;

  public static final int MOTOR_CAN_ID;

  public static final double GEAR_RATIO;
  public static final int SUPPLY_CURRENT_LIMIT;
  public static final int STATOR_CURRENT_LIMIT;

  public static final double MOMENT_OF_INERTIA;
  public static final DCMotor MOTOR_CONFIG;
  public static final Rotation2d INTAKE_ANGLE_OFFSET;

  public static final Rotation2d ZERO_OFFSET;
  public static final Rotation2d MIN_ANGLE;
  public static final Rotation2d MAX_ANGLE;

  public static final double PIN_LENGTH;

  public static final Gains GAINS;
  public static final AngularPositionConstraints CONSTRAINTS;

  public static final LinkLengths LINK_LENGTHS;

  public static final LinkBounds LINK_BOUNDS;

  public static final LinkConstants LINK_CONST;

  public static final FourBarLinkageConstants LINKAGE_CONSTANTS;

  public static final Map<IntakeState, Setpoint<AngleUnit>> INTAKE_STATES;

  static {
    switch (RobotConfig.ROBOT) {
      case V1_DOOMSPIRAL:
        INTAKE_VOLTAGE = 11.0;
        EXTAKE_VOLTAGE = -4.0;

        LINKAGE_ANGLE_INCREMENT = Rotation2d.fromDegrees(2.0);
        LINKAGE_SLOW_VOLTAGE = 1.5;

        INTAKE_ROLLER_CONSTANTS =
            GenericRollerConstants.builder()
                .withLeaderCANID(20)
                .withCurrentLimits(
                    CurrentLimits.builder()
                        .withSupplyCurrentLimit(Amps.of(40.0))
                        .withStatorCurrentLimit(Amps.of(40.0))
                        .build())
                .withNeutralMode(NeutralModeValue.Coast)
                .withRollerGearbox(DCMotor.getKrakenX60Foc(1))
                .withRollerMotorGearRatio(8.0 / 3.0)
                .withLeaderInvertedValue(InvertedValue.Clockwise_Positive)
                .withOpposedFollowerCANID(21)
                .withMomentOfInertia(Units.KilogramSquareMeters.of(0.0004))
                .withVoltageOffsetStep(Volts.of(0.25))
                .withCanBus(new CANBus())
                .withEnableFOC(false)
                .build();

        LINKAGE_OFFSET = new Translation3d(0.381, 0.141, 0.276);

        MOTOR_CAN_ID = 22;
        GEAR_RATIO = 50.79235079;
        SUPPLY_CURRENT_LIMIT = 40;
        STATOR_CURRENT_LIMIT = 40;

        MOMENT_OF_INERTIA = 0.004;
        MOTOR_CONFIG = DCMotor.getKrakenX60Foc(1);
        INTAKE_ANGLE_OFFSET = Rotation2d.fromDegrees(-30.9603232217);

        ZERO_OFFSET = Rotation2d.PI;
        MIN_ANGLE = Rotation2d.fromDegrees(9);
        MAX_ANGLE = Rotation2d.fromDegrees(190);
        // points A and D on the intake.

        PIN_LENGTH = Units.Inches.of(6.125).in(Units.Meters);

        GAINS =
            Gains.builder()
                .withPrefix("Linkage")
                .withKP(200.0)
                .withKD(0.0)
                .withKS(0.35537)
                .withKG(0.0)
                .withKV(0.0)
                .withKA(0.0)
                .build();
        CONSTRAINTS =
            AngularPositionConstraints.builder()
                .withPrefix("Linkage")
                .withMaxVelocity(RadiansPerSecond.of(50.0))
                .withMaxAcceleration(RadiansPerSecondPerSecond.of(15.0))
                .withGoalTolerance(Degrees.of(1.0))
                .build();

        LINK_LENGTHS =
            new LinkLengths(
                Units.Inches.of(6.943050).in(Units.Meters),
                Units.Inches.of(8.809879).in(Units.Meters),
                Units.Inches.of(8.284456).in(Units.Meters),
                Units.Inches.of(6.4213032).in(Units.Meters));

        LINK_BOUNDS =
            new LinkBounds(
                Units.Inches.of(0.810921).in(Units.Meters),
                Units.Inches.of(2.86545).in(Units.Meters),
                Units.Inches.of(4.752162).in(Units.Meters),
                Units.Inches.of(6.46545).in(Units.Meters));

        LINK_CONST =
            new LinkConstants(
                Units.Inches.of(6.092560).in(Units.Meters),
                Units.Inches.of(2.446682).in(Units.Meters),
                Units.Inches.of(5.376661).in(Units.Meters));

        LINKAGE_CONSTANTS =
            FourBarLinkageConstants.builder()
                .withCancoderSensorDirection(SensorDirectionValue.Clockwise_Positive)
                .withCanCoderCanId(23)
                .withConstraints(CONSTRAINTS)
                .withGains(GAINS)
                .withGearRatio(GEAR_RATIO)
                .withStartAngle(Rotation2d.fromDegrees(9.0))
                .withIntakeAngleOffset(INTAKE_ANGLE_OFFSET)
                .withLinkageOffset(LINKAGE_OFFSET)
                .withLinkBounds(LINK_BOUNDS)
                .withLinkConstants(LINK_CONST)
                .withLinkLengths(LINK_LENGTHS)
                .withMaxAngle(MAX_ANGLE)
                .withMinAngle(MIN_ANGLE)
                .withMomentOfInertia(MOMENT_OF_INERTIA)
                .withMotorCanId(MOTOR_CAN_ID)
                .withMotorConfig(MOTOR_CONFIG)
                .withPinLength(PIN_LENGTH)
                .withCurrentLimits(
                    CurrentLimits.fromDoubles()
                        .withSupplyCurrentLimit(SUPPLY_CURRENT_LIMIT)
                        .withStatorCurrentLimit(STATOR_CURRENT_LIMIT)
                        .build())
                .withZeroOffset(ZERO_OFFSET)
                .withPositionOffsetStep(LINKAGE_ANGLE_INCREMENT)
                .withCanCoderOffset(Rotation2d.fromRadians(-92.285156))
                .withEnableFoc(false)
                .build();

        INTAKE_STATES =
            Map.of(
                IntakeState.STOW,
                    new Setpoint<>(
                        Rotation2d.fromDegrees(9.0).getMeasure(),
                        LINKAGE_ANGLE_INCREMENT.getMeasure(),
                        MIN_ANGLE.getMeasure(),
                        MAX_ANGLE.getMeasure()),
                IntakeState.INTAKE,
                    new Setpoint<>(
                        Rotation2d.fromDegrees(168.134766 + 18.0).getMeasure(),
                        LINKAGE_ANGLE_INCREMENT.getMeasure(),
                        MIN_ANGLE.getMeasure(),
                        MAX_ANGLE.getMeasure()),
                IntakeState.AGITATE,
                    new Setpoint<>(
                        Rotation2d.fromDegrees(168.134766 + 18.0).getMeasure(),
                        LINKAGE_ANGLE_INCREMENT.getMeasure(),
                        MIN_ANGLE.getMeasure(),
                        MAX_ANGLE.getMeasure()));
        break;

      case V1_DOOMSPIRAL_SIM:
        INTAKE_VOLTAGE = 11.0;
        EXTAKE_VOLTAGE = -4.0;

        LINKAGE_ANGLE_INCREMENT = Rotation2d.fromDegrees(2.0);
        LINKAGE_SLOW_VOLTAGE = 1.5;

        INTAKE_ROLLER_CONSTANTS =
            GenericRollerConstants.builder()
                .withLeaderCANID(20)
                .withCurrentLimits(
                    CurrentLimits.builder()
                        .withSupplyCurrentLimit(Amps.of(40.0))
                        .withStatorCurrentLimit(Amps.of(40.0))
                        .build())
                .withNeutralMode(NeutralModeValue.Coast)
                .withRollerGearbox(DCMotor.getKrakenX60Foc(1))
                .withRollerMotorGearRatio(8.0 / 3.0)
                .withLeaderInvertedValue(InvertedValue.Clockwise_Positive)
                .withOpposedFollowerCANID(21)
                .withMomentOfInertia(Units.KilogramSquareMeters.of(0.0004))
                .withVoltageOffsetStep(Volts.of(0.25))
                .withCanBus(new CANBus())
                .withEnableFOC(false)
                .build();

        LINKAGE_OFFSET = new Translation3d(0.381, 0.141, 0.276);

        MOTOR_CAN_ID = 22;

        GEAR_RATIO = 50.79235079;
        SUPPLY_CURRENT_LIMIT = 40;
        STATOR_CURRENT_LIMIT = 40;

        MOMENT_OF_INERTIA = 0.004;
        MOTOR_CONFIG = DCMotor.getKrakenX60Foc(1);
        INTAKE_ANGLE_OFFSET = Rotation2d.fromDegrees(-30.9603232217);

        ZERO_OFFSET = Rotation2d.PI;
        MIN_ANGLE = Rotation2d.fromDegrees(9);
        MAX_ANGLE = Rotation2d.fromDegrees(190);
        // points A and D on the intake.

        PIN_LENGTH = Units.Inches.of(6.125).in(Units.Meters);

        GAINS =
            Gains.builder()
                .withPrefix("Linkage")
                .withKP(20.0)
                .withKD(0.0)
                .withKS(0.35537)
                .withKG(0.0)
                .withKV(0.0)
                .withKA(0.0)
                .build();
        CONSTRAINTS =
            AngularPositionConstraints.builder()
                .withPrefix("Linkage")
                .withMaxVelocity(RadiansPerSecond.of(10.0))
                .withMaxAcceleration(RadiansPerSecondPerSecond.of(50.0))
                .withGoalTolerance(Degrees.of(1.0))
                .build();

        LINK_LENGTHS =
            new LinkLengths(
                Units.Inches.of(6.943050).in(Units.Meters),
                Units.Inches.of(8.809879).in(Units.Meters),
                Units.Inches.of(8.284456).in(Units.Meters),
                Units.Inches.of(6.4213032).in(Units.Meters));

        LINK_BOUNDS =
            new LinkBounds(
                Units.Inches.of(0.810921).in(Units.Meters),
                Units.Inches.of(2.86545).in(Units.Meters),
                Units.Inches.of(4.752162).in(Units.Meters),
                Units.Inches.of(6.46545).in(Units.Meters));

        LINK_CONST =
            new LinkConstants(
                Units.Inches.of(6.092560).in(Units.Meters),
                Units.Inches.of(2.446682).in(Units.Meters),
                Units.Inches.of(5.376661).in(Units.Meters));

        LINKAGE_CONSTANTS =
            FourBarLinkageConstants.builder()
                .withCancoderSensorDirection(SensorDirectionValue.Clockwise_Positive)
                .withCanCoderCanId(23)
                .withConstraints(CONSTRAINTS)
                .withGains(GAINS)
                .withGearRatio(GEAR_RATIO)
                .withStartAngle(Rotation2d.fromDegrees(9.0))
                .withIntakeAngleOffset(INTAKE_ANGLE_OFFSET)
                .withLinkageOffset(LINKAGE_OFFSET)
                .withLinkBounds(LINK_BOUNDS)
                .withLinkConstants(LINK_CONST)
                .withLinkLengths(LINK_LENGTHS)
                .withMaxAngle(MAX_ANGLE)
                .withMinAngle(MIN_ANGLE)
                .withMomentOfInertia(MOMENT_OF_INERTIA)
                .withMotorCanId(MOTOR_CAN_ID)
                .withMotorConfig(MOTOR_CONFIG)
                .withPinLength(PIN_LENGTH)
                .withCurrentLimits(
                    CurrentLimits.fromDoubles()
                        .withSupplyCurrentLimit(SUPPLY_CURRENT_LIMIT)
                        .withStatorCurrentLimit(STATOR_CURRENT_LIMIT)
                        .build())
                .withZeroOffset(ZERO_OFFSET)
                .withPositionOffsetStep(LINKAGE_ANGLE_INCREMENT)
                .withCanCoderOffset(Rotation2d.fromRadians(92.285156))
                .withEnableFoc(false)
                .build();
        INTAKE_STATES =
            Map.of(
                IntakeState.STOW,
                new Setpoint<>(
                    Rotation2d.fromDegrees(9.0).getMeasure(),
                    LINKAGE_ANGLE_INCREMENT.getMeasure(),
                    MIN_ANGLE.getMeasure(),
                    MAX_ANGLE.getMeasure()),
                IntakeState.INTAKE,
                new Setpoint<>(
                    Rotation2d.fromDegrees(168.134766 + 8.0).getMeasure(),
                    LINKAGE_ANGLE_INCREMENT.getMeasure(),
                    MIN_ANGLE.getMeasure(),
                    MAX_ANGLE.getMeasure()),
                IntakeState.AGITATE,
                new Setpoint<>(
                    Rotation2d.fromDegrees(168.134766 + 8.0).getMeasure(),
                    LINKAGE_ANGLE_INCREMENT.getMeasure(),
                    MIN_ANGLE.getMeasure(),
                    MAX_ANGLE.getMeasure()));
        break;

      case V2_TURNOVER:
        INTAKE_VOLTAGE = 11.0;
        EXTAKE_VOLTAGE = -4.0;

        LINKAGE_ANGLE_INCREMENT = Rotation2d.fromDegrees(2.0);
        LINKAGE_SLOW_VOLTAGE = 1.5;

        INTAKE_ROLLER_CONSTANTS =
            GenericRollerConstants.builder()
                .withLeaderCANID(41)
                .withCurrentLimits(
                    CurrentLimits.builder()
                        .withSupplyCurrentLimit(Amps.of(40.0))
                        .withStatorCurrentLimit(Amps.of(40.0))
                        .build())
                .withNeutralMode(NeutralModeValue.Coast)
                .withRollerGearbox(DCMotor.getKrakenX60Foc(1))
                .withRollerMotorGearRatio((16.0 / 40.0) * (52.0))
                .withLeaderInvertedValue(InvertedValue.CounterClockwise_Positive)
                .withOpposedFollowerCANID(42)
                .withMomentOfInertia(Units.KilogramSquareMeters.of(0.0004))
                .withVoltageOffsetStep(Volts.of(1))
                .withCanBus(new CANBus())
                .withEnableFOC(false)
                .build();

        LINKAGE_OFFSET =
            new Translation3d(0.381, 0.141, 0.276); // TODO: Anshu says he doesn't really care

        MOTOR_CAN_ID = 40; // Pivot

        GEAR_RATIO = (62.0 / 11.0) * (68.0 / 18.0) * (34.0 / 28.0) * (30.0 / 24.0);
        SUPPLY_CURRENT_LIMIT = 40;
        STATOR_CURRENT_LIMIT = 40;

        MOMENT_OF_INERTIA = 0.004;
        MOTOR_CONFIG = DCMotor.getKrakenX60Foc(1);
        INTAKE_ANGLE_OFFSET = Rotation2d.fromDegrees(-30.838927);

        ZERO_OFFSET = Rotation2d.PI;
        MIN_ANGLE = Rotation2d.fromDegrees(30.838927);
        MAX_ANGLE = Rotation2d.fromDegrees(168.912511);
        // points A and D on the intake.

        PIN_LENGTH = Units.Inches.of(6.125).in(Units.Meters);

        GAINS =
            Gains.builder()
                .withPrefix("Linkage")
                .withKP(120)
                .withKD(5.0)
                .withKS(0.049018)
                .withKG(0.35311)
                .withKV(4.7192)
                .withKA(0.0)
                .build();
        CONSTRAINTS =
            AngularPositionConstraints.builder()
                .withPrefix("Linkage")
                .withMaxVelocity(RotationsPerSecond.of(17))
                .withMaxAcceleration(RotationsPerSecondPerSecond.of(85))
                .withGoalTolerance(Degrees.of(2.0))
                .build();

        LINK_LENGTHS =
            new LinkLengths(
                Units.Inches.of(6.500000).in(Units.Meters),
                Units.Inches.of(8.945053).in(Units.Meters),
                Units.Inches.of(7.500000).in(Units.Meters),
                Units.Inches.of(6.823672).in(Units.Meters));
        //
        // These don't matter for V2
        LINK_BOUNDS =
            new LinkBounds(
                Units.Inches.of(0.810921).in(Units.Meters),
                Units.Inches.of(2.86545).in(Units.Meters),
                Units.Inches.of(4.752162).in(Units.Meters),
                Units.Inches.of(6.46545).in(Units.Meters));

        LINK_CONST =
            new LinkConstants(
                Units.Inches.of(6.092560).in(Units.Meters),
                Units.Inches.of(2.446682).in(Units.Meters),
                Units.Inches.of(5.376661).in(Units.Meters));

        LINKAGE_CONSTANTS =
            FourBarLinkageConstants.builder()
                .withConstraints(CONSTRAINTS)
                .withGains(GAINS)
                .withGearRatio(GEAR_RATIO)
                .withStartAngle(MAX_ANGLE)
                .withIntakeAngleOffset(INTAKE_ANGLE_OFFSET)
                .withLinkageOffset(LINKAGE_OFFSET)
                .withLinkBounds(LINK_BOUNDS)
                .withLinkConstants(LINK_CONST)
                .withLinkLengths(LINK_LENGTHS)
                .withMaxAngle(MAX_ANGLE)
                .withMinAngle(MIN_ANGLE)
                .withMomentOfInertia(MOMENT_OF_INERTIA)
                .withMotorCanId(MOTOR_CAN_ID)
                .withMotorConfig(MOTOR_CONFIG)
                .withPinLength(PIN_LENGTH)
                .withCurrentLimits(
                    CurrentLimits.fromDoubles()
                        .withSupplyCurrentLimit(SUPPLY_CURRENT_LIMIT)
                        .withStatorCurrentLimit(STATOR_CURRENT_LIMIT)
                        .build())
                .withZeroOffset(ZERO_OFFSET)
                .withPositionOffsetStep(LINKAGE_ANGLE_INCREMENT)
                .withEnableFoc(true)
                .build();

        INTAKE_STATES =
            Map.of(
                IntakeState.STOW,
                new Setpoint<>(
                    MIN_ANGLE.getMeasure(),
                    LINKAGE_ANGLE_INCREMENT.getMeasure(),
                    MIN_ANGLE.getMeasure(),
                    MAX_ANGLE.getMeasure()),
                IntakeState.INTAKE,
                new Setpoint<>(
                    MAX_ANGLE.getMeasure(),
                    LINKAGE_ANGLE_INCREMENT.getMeasure(),
                    MIN_ANGLE.getMeasure(),
                    MAX_ANGLE.getMeasure()),
                IntakeState.AGITATE,
                new Setpoint<>(
                    MAX_ANGLE.getMeasure(),
                    LINKAGE_ANGLE_INCREMENT.getMeasure(),
                    MIN_ANGLE.getMeasure(),
                    MAX_ANGLE.getMeasure()));
        break;
      case V2_TURNOVER_SIM:
      default:
        INTAKE_VOLTAGE = 11.0;
        EXTAKE_VOLTAGE = -4.0;

        LINKAGE_ANGLE_INCREMENT = Rotation2d.fromDegrees(2.0);
        LINKAGE_SLOW_VOLTAGE = 1.5;

        INTAKE_ROLLER_CONSTANTS =
            GenericRollerConstants.builder()
                .withLeaderCANID(20)
                .withCurrentLimits(
                    CurrentLimits.builder()
                        .withSupplyCurrentLimit(Amps.of(40.0))
                        .withStatorCurrentLimit(Amps.of(40.0))
                        .build())
                .withNeutralMode(NeutralModeValue.Coast)
                .withRollerGearbox(DCMotor.getKrakenX60Foc(1))
                .withRollerMotorGearRatio((16.0 / 40.0) * (52.0))
                .withLeaderInvertedValue(InvertedValue.Clockwise_Positive)
                .withOpposedFollowerCANID(21)
                .withMomentOfInertia(Units.KilogramSquareMeters.of(0.0004))
                .withVoltageOffsetStep(Volts.of(0.25))
                .withCanBus(new CANBus())
                .withEnableFOC(false)
                .build();

        LINKAGE_OFFSET = new Translation3d(0, 0, 0); // wrong for testing

        MOTOR_CAN_ID = 22;
        GEAR_RATIO = (62.0 / 11.0) * (68.0 / 18.0) * (34.0 / 28.0) * (30.0 / 24.0);
        SUPPLY_CURRENT_LIMIT = 40;
        STATOR_CURRENT_LIMIT = 40;

        MOMENT_OF_INERTIA = 0.004;
        MOTOR_CONFIG = DCMotor.getKrakenX60Foc(1);
        INTAKE_ANGLE_OFFSET = Rotation2d.fromDegrees(-30.838927);

        ZERO_OFFSET = Rotation2d.PI;
        MIN_ANGLE = Rotation2d.fromDegrees(30.838927);
        MAX_ANGLE = Rotation2d.fromDegrees(168.912511);
        // points A and D on the intake.

        PIN_LENGTH = Units.Inches.of(6.125).in(Units.Meters);

        GAINS =
            Gains.builder()
                .withPrefix("Linkage")
                .withKP(20.0)
                .withKD(0.0)
                .withKS(0.35537)
                .withKG(0.0)
                .withKV(0.0)
                .withKA(0.0)
                .build();
        CONSTRAINTS =
            AngularPositionConstraints.builder()
                .withPrefix("Linkage")
                .withMaxVelocity(RadiansPerSecond.of(10.0))
                .withMaxAcceleration(RadiansPerSecondPerSecond.of(50.0))
                .withGoalTolerance(Degrees.of(1.0))
                .build();

        LINK_LENGTHS =
            new LinkLengths(
                Units.Inches.of(6.500000).in(Units.Meters),
                Units.Inches.of(8.945053).in(Units.Meters),
                Units.Inches.of(7.500000).in(Units.Meters),
                Units.Inches.of(6.823672).in(Units.Meters));

        // These don't matter for V2
        LINK_BOUNDS =
            new LinkBounds(
                Units.Inches.of(0.810921).in(Units.Meters),
                Units.Inches.of(2.86545).in(Units.Meters),
                Units.Inches.of(4.752162).in(Units.Meters),
                Units.Inches.of(6.46545).in(Units.Meters));

        LINK_CONST =
            new LinkConstants(
                Units.Inches.of(6.092560).in(Units.Meters),
                Units.Inches.of(2.446682).in(Units.Meters),
                Units.Inches.of(5.376661).in(Units.Meters));

        LINKAGE_CONSTANTS =
            FourBarLinkageConstants.builder()
                .withConstraints(CONSTRAINTS)
                .withGains(GAINS)
                .withGearRatio(GEAR_RATIO)
                .withStartAngle(MIN_ANGLE)
                .withIntakeAngleOffset(INTAKE_ANGLE_OFFSET)
                .withLinkageOffset(LINKAGE_OFFSET)
                .withLinkBounds(LINK_BOUNDS)
                .withLinkConstants(LINK_CONST)
                .withLinkLengths(LINK_LENGTHS)
                .withMaxAngle(MAX_ANGLE)
                .withMinAngle(MIN_ANGLE)
                .withMomentOfInertia(MOMENT_OF_INERTIA)
                .withMotorCanId(MOTOR_CAN_ID)
                .withMotorConfig(MOTOR_CONFIG)
                .withPinLength(PIN_LENGTH)
                .withCurrentLimits(
                    CurrentLimits.fromDoubles()
                        .withSupplyCurrentLimit(SUPPLY_CURRENT_LIMIT)
                        .withStatorCurrentLimit(STATOR_CURRENT_LIMIT)
                        .build())
                .withZeroOffset(ZERO_OFFSET)
                .withPositionOffsetStep(LINKAGE_ANGLE_INCREMENT)
                .withEnableFoc(true)
                .build();
        INTAKE_STATES =
            Map.of(
                IntakeState.STOW,
                new Setpoint<>(
                    MIN_ANGLE.getMeasure(),
                    LINKAGE_ANGLE_INCREMENT.getMeasure(),
                    MIN_ANGLE.getMeasure(),
                    MAX_ANGLE.getMeasure()),
                IntakeState.INTAKE,
                new Setpoint<>(
                    MAX_ANGLE.getMeasure(),
                    LINKAGE_ANGLE_INCREMENT.getMeasure(),
                    MIN_ANGLE.getMeasure(),
                    MAX_ANGLE.getMeasure()),
                IntakeState.AGITATE,
                new Setpoint<>(
                    MAX_ANGLE.getMeasure(),
                    LINKAGE_ANGLE_INCREMENT.getMeasure(),
                    MIN_ANGLE.getMeasure(),
                    MAX_ANGLE.getMeasure()));
        break;
    }
  }

  public enum IntakeState {
    STOW,
    INTAKE,
    AGITATE
  }
}
