package first.robot.shared.subsystems.climber;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Radians;
import static org.wpilib.units.Units.RadiansPerSecond;
import static org.wpilib.units.Units.RadiansPerSecondPerSecond;
import static org.wpilib.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.team190.gompeilib.core.utility.control.CurrentLimits;
import edu.wpi.team190.gompeilib.core.utility.control.Gains;
import edu.wpi.team190.gompeilib.core.utility.control.constraints.AngularPositionConstraints;
import edu.wpi.team190.gompeilib.subsystems.arm.ArmConstants;
import edu.wpi.team190.gompeilib.subsystems.arm.ArmConstants.ArmParameters;
import first.robot.RobotConfig;
import lombok.AllArgsConstructor;
import lombok.Getter;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.system.DCMotor;

public class ClimberConstants {

  public static final double SLOW_VOLTAGE;
  public static final boolean FOC_ENABLE;
  public static final Gains SLOT_0_GAINS;
  public static final Gains SLOT_1_GAINS;
  public static final Gains SLOT_2_GAINS;
  public static final AngularPositionConstraints CONSTRAINTS;
  public static final CurrentLimits CURRENT_LIMITS;
  public static final ArmParameters ARM_PARAMETERS;
  public static final ArmConstants CLIMBER_CONSTANTS;

  static {
    switch (RobotConfig.ROBOT) {
      case V1_DOOMSPIRAL:
      case V1_DOOMSPIRAL_SIM:
        SLOW_VOLTAGE = 1.0;
        SLOT_0_GAINS =
            Gains.builder()
                .withPrefix("Climber/Slot0")
                .withKP(25.0)
                .withKD(0.0)
                .withKS(0.061752)
                .withKG(0.034689)
                .withKV(26.487)
                .withKA(0.36032)
                .build();

        SLOT_1_GAINS =
            Gains.builder()
                .withPrefix("Climber/Slot1")
                .withKP(450.0)
                .withKD(0.0)
                .withKS(.1)
                .withKG(0.034689)
                .withKV(26.487)
                .withKA(0.36032)
                .build();

        SLOT_2_GAINS =
            Gains.builder()
                .withPrefix("Climber/Slot1")
                .withKP(450.0)
                .withKD(0.0)
                .withKS(.1)
                .withKG(0.034689)
                .withKV(26.487)
                .withKA(0.36032)
                .build();

        CONSTRAINTS =
            AngularPositionConstraints.builder()
                .withPrefix("Climber")
                .withMaxVelocity(RadiansPerSecond.of(18))
                .withMaxAcceleration(RadiansPerSecondPerSecond.of(100))
                .withGoalTolerance(Radians.of(0.01))
                .build();

        CURRENT_LIMITS =
            CurrentLimits.builder()
                .withSupplyCurrentLimit(Amps.of(40.0))
                .withStatorCurrentLimit(Amps.of(80.0))
                .build();

        FOC_ENABLE = false;

        ARM_PARAMETERS =
            ArmParameters.builder()
                .withMotorConfig(DCMotor.getKrakenX60Foc(1))
                .withMinAngle(Rotation2d.fromRadians(-100 * Math.PI))
                .withMaxAngle(Rotation2d.fromRadians(100 * Math.PI))
                .withNumMotors(1)
                .withGearRatio(224.0)
                .withLengthMeters(0.259)
                .withContinuousOutput(false)
                .withMomentOfInertia(0.0817231996)
                .build();

        CLIMBER_CONSTANTS =
            ArmConstants.builder()
                .withArmCANID(60)
                .withArmParameters(ARM_PARAMETERS)
                .withSlot0Gains(SLOT_0_GAINS)
                .withSlot1Gains(SLOT_1_GAINS)
                .withSlot2Gains(SLOT_0_GAINS)
                .withConstraints(CONSTRAINTS)
                .withCurrentLimits(CURRENT_LIMITS)
                .withEnableFOC(FOC_ENABLE)
                .withInvertedValue(InvertedValue.CounterClockwise_Positive)
                .withCanBus(new CANBus())
                .withVoltageOffsetStep(Volts.of(5.0))
                .withPositionOffsetStep(new Rotation2d(5 * Math.PI / 180))
                .build();
        break;

      case V2_TURNOVER:
      case V2_TURNOVER_SIM:
      default:
        SLOW_VOLTAGE = 1.0;
        SLOT_0_GAINS =
            Gains.builder()
                .withPrefix("Climber/Slot0")
                .withKP(25.0)
                .withKD(0.0)
                .withKS(0.061752)
                .withKG(0.034689)
                .withKV(26.487)
                .withKA(0.36032)
                .build();

        SLOT_1_GAINS =
            Gains.builder()
                .withPrefix("Climber/Slot1")
                .withKP(450.0)
                .withKD(0.0)
                .withKS(.1)
                .withKG(0.034689)
                .withKV(26.487)
                .withKA(0.36032)
                .build();

        SLOT_2_GAINS =
            Gains.builder()
                .withPrefix("Climber/Slot1")
                .withKP(450.0)
                .withKD(0.0)
                .withKS(.1)
                .withKG(0.034689)
                .withKV(26.487)
                .withKA(0.36032)
                .build();

        CONSTRAINTS =
            AngularPositionConstraints.builder()
                .withPrefix("Climber")
                .withMaxVelocity(RadiansPerSecond.of(18))
                .withMaxAcceleration(RadiansPerSecondPerSecond.of(100))
                .withGoalTolerance(Radians.of(0.01))
                .build();

        CURRENT_LIMITS =
            CurrentLimits.builder()
                .withSupplyCurrentLimit(Amps.of(40.0))
                .withStatorCurrentLimit(Amps.of(80.0))
                .build();

        FOC_ENABLE = false;

        ARM_PARAMETERS =
            ArmParameters.builder()
                .withMotorConfig(DCMotor.getKrakenX60Foc(1))
                .withMinAngle(Rotation2d.fromRadians(-100 * Math.PI))
                .withMaxAngle(Rotation2d.fromRadians(100 * Math.PI))
                .withNumMotors(1)
                .withGearRatio(224.0)
                .withLengthMeters(0.259)
                .withContinuousOutput(false)
                .withMomentOfInertia(0.0817231996)
                .build();

        CLIMBER_CONSTANTS =
            ArmConstants.builder()
                .withArmCANID(50)
                .withArmParameters(ARM_PARAMETERS)
                .withSlot0Gains(SLOT_0_GAINS)
                .withSlot1Gains(SLOT_1_GAINS)
                .withSlot2Gains(SLOT_0_GAINS)
                .withConstraints(CONSTRAINTS)
                .withCurrentLimits(CURRENT_LIMITS)
                .withEnableFOC(FOC_ENABLE)
                .withInvertedValue(InvertedValue.Clockwise_Positive)
                .withCanBus(new CANBus())
                .withVoltageOffsetStep(Volts.of(0.0))
                .withPositionOffsetStep(Rotation2d.fromRadians(0.0))
                .build();
        break;
    }
  }

  @AllArgsConstructor
  public enum ClimberGoal {
    L1_POSITION_GOAL(Rotation2d.fromDegrees(115.927734)),
    L1_AUTO_POSITION_GOAL(Rotation2d.fromDegrees(100.898438)),
    L1_AUTO_POSITION_GOAL_CLIMBED(Rotation2d.fromDegrees(38.144531)),
    UNCLIMB(Rotation2d.fromDegrees(200.748047)),
    L2_POSITION_GOAL(new Rotation2d(3 * Math.PI / 2)),
    L2_FLIP_GOAL(Rotation2d.fromDegrees(455.009766)),
    DEFAULT(new Rotation2d(0));

    @Getter private final Rotation2d position;
  }

  public record RollPIDConstants(
      double kP, double kD, double maxVelocity, double maxAcceleration) {}

  public static final RollPIDConstants ROLL_PID_CONSTANTS =
      new RollPIDConstants(0.0, 0.0, 0.0, 0.0);
}
