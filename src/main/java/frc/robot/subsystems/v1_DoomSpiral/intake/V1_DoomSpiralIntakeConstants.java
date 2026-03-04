package frc.robot.subsystems.v1_DoomSpiral.intake;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.team190.gompeilib.core.utility.control.AngularConstraints;
import edu.wpi.team190.gompeilib.core.utility.control.CurrentLimits;
import edu.wpi.team190.gompeilib.core.utility.control.Gains;
import edu.wpi.team190.gompeilib.core.utility.tunable.LoggedTunableMeasure;
import edu.wpi.team190.gompeilib.core.utility.tunable.LoggedTunableNumber;
import edu.wpi.team190.gompeilib.subsystems.generic.roller.GenericRollerConstants;
import frc.robot.subsystems.shared.fourbarlinkage.FourBarLinkageConstants;
import frc.robot.subsystems.shared.fourbarlinkage.FourBarLinkageConstants.LinkBounds;
import frc.robot.subsystems.shared.fourbarlinkage.FourBarLinkageConstants.LinkConstants;
import frc.robot.subsystems.shared.fourbarlinkage.FourBarLinkageConstants.LinkLengths;
import lombok.Getter;
import lombok.RequiredArgsConstructor;

public class V1_DoomSpiralIntakeConstants {

  public static final double INTAKE_VOLTAGE = 12.0;
  public static final double EXTAKE_VOLTAGE = -4.0;

  public static final Rotation2d LINKAGE_ANGLE_INCREMENT = Rotation2d.fromDegrees(2.0);
  public static final double LINKAGE_SLOW_VOLTAGE = 1.5;

  public static final GenericRollerConstants INTAKE_ROLLER_CONSTANTS_TOP =
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
          .withCanBus(CANBus.roboRIO())
          .build();

  public static final Translation3d LINKAGE_OFFSET = new Translation3d(0.381, 0.141, 0.276);

  public static final int MOTOR_CAN_ID = 22;
  public static final int CAN_CODER_CAN_ID = 23;
  public static final Rotation2d CAN_CODER_OFFSET = Rotation2d.fromDegrees(-92.285156);

  public static final SensorDirectionValue CANCODER_SENSOR_DIRECTION =
      SensorDirectionValue.Clockwise_Positive;
  public static final double GEAR_RATIO = 50.79235079;
  public static final int SUPPLY_CURRENT_LIMIT = 40;
  public static final int STATOR_CURRENT_LIMIT = 40;

  public static final double MOMENT_OF_INERTIA = 0.004;
  public static final DCMotor MOTOR_CONFIG = DCMotor.getKrakenX60Foc(1);
  public static final Rotation2d INTAKE_ANGLE_OFFSET = Rotation2d.fromDegrees(-30.9603232217);

  public static final Rotation2d ZERO_OFFSET = Rotation2d.kPi;
  public static final Rotation2d MIN_ANGLE = Rotation2d.fromDegrees(9);
  public static final Rotation2d MAX_ANGLE = Rotation2d.fromDegrees(190);
  // points A and D on the intake.

  public static final double PIN_LENGTH = Units.Inches.of(6.125).in(Units.Meters);

  public static final Gains GAINS =
      Gains.builder()
          .withKP(new LoggedTunableNumber("Linkage/KP", 200.0))
          .withKD(new LoggedTunableNumber("Linkage/KD", 0.0))
          .withKS(new LoggedTunableNumber("Linkage/KS", 0.35537))
          .withKG(new LoggedTunableNumber("Linkage/KG", 0.0))
          .withKV(new LoggedTunableNumber("Linkage/KV", 0.0))
          .withKA(new LoggedTunableNumber("Linkage/KA", 0.0))
          .build();
  public static final AngularConstraints CONSTRAINTS =
      AngularConstraints.builder()
          .withMaxVelocity(
              new LoggedTunableMeasure<>("Linkage/Max Velocity", RadiansPerSecond.of(10.0)))
          .withMaxAcceleration(
              new LoggedTunableMeasure<>(
                  "Linkage/Max Acceleration", RadiansPerSecondPerSecond.of(50.0)))
          .withGoalTolerance(new LoggedTunableMeasure<>("Linkage/Goal Tolerance", Degrees.of(1.0)))
          .build();

  public static final LinkLengths LINK_LENGTHS =
      new LinkLengths(
          Units.Inches.of(6.943050).in(Units.Meters),
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
          .CAN_CODER_OFFSET(CAN_CODER_OFFSET)
          .build();

  @RequiredArgsConstructor
  @Getter
  public enum IntakeState {
    STOW(Rotation2d.fromDegrees(9)),
    INTAKE(Rotation2d.fromDegrees(168.134766)),
    BUMP(Rotation2d.fromDegrees(145));

    private final Rotation2d angle;
  }
}
