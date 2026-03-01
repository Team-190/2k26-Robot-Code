package frc.robot.subsystems.v1_DoomSpiral.climber;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.team190.gompeilib.core.utility.control.AngularConstraints;
import edu.wpi.team190.gompeilib.core.utility.control.CurrentLimits;
import edu.wpi.team190.gompeilib.core.utility.control.Gains;
import edu.wpi.team190.gompeilib.core.utility.tunable.LoggedTunableMeasure;
import edu.wpi.team190.gompeilib.core.utility.tunable.LoggedTunableNumber;
import edu.wpi.team190.gompeilib.subsystems.arm.ArmConstants;
import edu.wpi.team190.gompeilib.subsystems.arm.ArmConstants.ArmParameters;
import lombok.AllArgsConstructor;
import lombok.Getter;

public class V1_DoomSpiralClimberConstants {

  public static final double SLOW_VOLTAGE = 1.0;
  public static final Gains SLOT_0_GAINS =
      Gains.builder()
          .withKP(new LoggedTunableNumber("Climber/Slot0/kP"))
          .withKD(new LoggedTunableNumber("Climber/Slot0/kD"))
          .withKS(new LoggedTunableNumber("Climber/Slot0/kS"))
          .withKG(new LoggedTunableNumber("Climber/Slot0/kG"))
          .withKV(new LoggedTunableNumber("Climber/Slot0/kV"))
          .withKA(new LoggedTunableNumber("Climber/Slot0/kA"))
          .build();

  public static final AngularConstraints CONSTRAINTS =
      AngularConstraints.builder()
          .withMaxVelocity(
              new LoggedTunableMeasure<>("Climber/MaxVelocity", RadiansPerSecond.of(4)))
          .withMaxAcceleration(
              new LoggedTunableMeasure<>(
                  "Climber/MaxAcceleration", RadiansPerSecondPerSecond.of(6)))
          .withGoalTolerance(new LoggedTunableMeasure<>("Climber/GoalTolerance", Radians.of(0.05)))
          .build();

  public static final CurrentLimits CURRENT_LIMITS =
      CurrentLimits.builder()
          .withSupplyCurrentLimit(Amps.of(40.0))
          .withStatorCurrentLimit(Amps.of(80.0))
          .build();

  public static boolean ENABLE_FOC = false;

  public static final ArmParameters ARM_PARAMETERS =
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

  @AllArgsConstructor
  public enum ClimberGoal {
    L1_POSITION_GOAL(Rotation2d.fromDegrees(115.576172 - 8)),
    L1_AUTO_POSITION_GOAL(new Rotation2d(-Math.PI / 2)),
    L2_POSITION_GOAL(new Rotation2d(3 * Math.PI / 2)),
    L2_FLIP_GOAL(new Rotation2d(5 * Math.PI / 2).plus(Rotation2d.fromDegrees(7.64))),
    DEFAULT(new Rotation2d(0));

    @Getter private final Rotation2d position;
  }

  public static final ArmConstants CLIMBER_CONSTANTS =
      ArmConstants.builder()
          .withArmCANID(60)
          .withArmParameters(ARM_PARAMETERS)
          .withSlot0Gains(SLOT_0_GAINS)
          .withSlot1Gains(SLOT_0_GAINS)
          .withSlot2Gains(SLOT_0_GAINS)
          .withConstraints(CONSTRAINTS)
          .withCurrentLimits(CURRENT_LIMITS)
          .withEnableFOC(ENABLE_FOC)
          .withInvertedValue(InvertedValue.Clockwise_Positive)
          .withCanBus(CANBus.roboRIO())
          .build();

  public record RollPIDConstants(
      double kP, double kD, double maxVelocity, double maxAcceleration) {}

  public static final RollPIDConstants ROLL_PID_CONSTANTS =
      new RollPIDConstants(0.0, 0.0, 0.0, 0.0);
}
