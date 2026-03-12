package frc.robot.subsystems.shared.climber;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.InvertedValue;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.team190.gompeilib.core.utility.control.AngularConstraints;
import edu.wpi.team190.gompeilib.core.utility.control.CurrentLimits;
import edu.wpi.team190.gompeilib.core.utility.control.Gains;
import edu.wpi.team190.gompeilib.core.utility.tunable.LoggedTunableMeasure;
import edu.wpi.team190.gompeilib.core.utility.tunable.LoggedTunableNumber;
import edu.wpi.team190.gompeilib.subsystems.arm.ArmConstants.ArmParameters;
import frc.robot.RobotConfig;
import frc.robot.subsystems.v1_DoomSpiral.climber.V1_DoomSpiralClimberConstants;
import lombok.Builder;
import lombok.NonNull;

@Builder(setterPrefix = "with")
public class ClimberConstants {

    @NonNull
    public final Double slowVoltage;
    @NonNull
    public final Gains slot0Gains;
    @NonNull
    public final Gains slot1Gains;
    @NonNull
    public final Gains slot2Gains;
    @NonNull
    public final AngularConstraints constraints;
    @NonNull
    public final CurrentLimits currentLimits;
    @NonNull
    public final Boolean enableFOC;
    @NonNull
    public final ArmParameters armParameters;

    @NonNull
    public final Integer motorCanID;
    @NonNull
    public final CANBus canBus;

    @NonNull
    public final InvertedValue invertedValue;

    @NonNull
    public final ClimberGoal climberGoal;

    @Builder
    public record ClimberGoal(Rotation2d L1_POSITION_GOAL, Rotation2d L1_AUTO_POSITION_GOAL,
            Rotation2d L1_AUTO_POSITION_GOAL_CLIMBED, Rotation2d UNCLIMB,
            Rotation2d L2_POSITION_GOAL, Rotation2d L2_FLIP_GOAL, Rotation2d DEFAULT) {
    }

    public static ClimberConstants getClimberConstants() {
        switch (RobotConfig.ROBOT) {
            case V1_DOOMSPIRAL:
                return ClimberConstants.builder()
                        .withSlowVoltage(1.0)
                        .withSlot0Gains(Gains.builder()
                                .withKP(new LoggedTunableNumber("Climber/Slot0/kP", 25.0))
                                .withKD(new LoggedTunableNumber("Climber/Slot0/kD", 0.0))
                                .withKS(new LoggedTunableNumber("Climber/Slot0/kS", 0.061752))
                                .withKG(new LoggedTunableNumber("Climber/Slot0/kG", 0.034689))
                                .withKV(new LoggedTunableNumber("Climber/Slot0/kV", 26.487))
                                .withKA(new LoggedTunableNumber("Climber/Slot0/kA", 0.36032))
                                .build())
                        .withSlot1Gains(Gains.builder()
                                .withKP(new LoggedTunableNumber("Climber/Slot1/kP", 450.0))
                                .withKD(new LoggedTunableNumber("Climber/Slot1/kD", 0.0))
                                .withKS(new LoggedTunableNumber("Climber/Slot1/kS", .1))
                                .withKG(new LoggedTunableNumber("Climber/Slot1/kG", 0.034689))
                                .withKV(new LoggedTunableNumber("Climber/Slot1/kV", 26.487))
                                .withKA(new LoggedTunableNumber("Climber/Slot1/kA", 0.36032))
                                .build())
                        .withSlot2Gains(Gains.builder()
                                .withKP(new LoggedTunableNumber("Climber/Slot2/kP", 450.0))
                                .withKD(new LoggedTunableNumber("Climber/Slot2/kD", 0.0))
                                .withKS(new LoggedTunableNumber("Climber/Slot2/kS", .1))
                                .withKG(new LoggedTunableNumber("Climber/Slot2/kG", 0.034689))
                                .withKV(new LoggedTunableNumber("Climber/Slot2/kV", 26.487))
                                .withKA(new LoggedTunableNumber("Climber/Slot2/kA", 0.36032))
                                .build())
                        .withConstraints(AngularConstraints.builder()
                                .withMaxVelocity(
                                        new LoggedTunableMeasure<>("Climber/MaxVelocity", RadiansPerSecond.of(18)))
                                .withMaxAcceleration(
                                        new LoggedTunableMeasure<>(
                                                "Climber/MaxAcceleration", RadiansPerSecondPerSecond.of(100)))
                                .withGoalTolerance(
                                        new LoggedTunableMeasure<>("Climber/GoalTolerance", Radians.of(0.01)))
                                .build())
                        .withCurrentLimits(CurrentLimits.builder()
          .withSupplyCurrentLimit(Amps.of(40.0))
          .withStatorCurrentLimit(Amps.of(80.0))
          .build())
                        .withEnableFOC(false)
                        .withArmParameters(ArmParameters.builder()
          .withMotorConfig(DCMotor.getKrakenX60Foc(1))
          .withMinAngle(Rotation2d.fromRadians(-100 * Math.PI))
          .withMaxAngle(Rotation2d.fromRadians(100 * Math.PI))
          .withNumMotors(1)
          .withGearRatio(224.0)
          .withLengthMeters(0.259)
          .withContinuousOutput(false)
          .withMomentOfInertia(0.0817231996)
          .build())
                        .withMotorCanID(60)

//TODO: keep updating this Ananth has to leave

                        .withCanBus(V1_DoomSpiralClimberConstants.CLIMBER_CONSTANTS.canBus)
                        .withInvertedValue(InvertedValue.Clockwise_Positive)
                        .withClimberGoal(ClimberGoal.builder()
                                .L1_POSITION_GOAL(
                                        V1_DoomSpiralClimberConstants.ClimberGoal.L1_POSITION_GOAL.getPosition())
                                .L1_AUTO_POSITION_GOAL(
                                        V1_DoomSpiralClimberConstants.ClimberGoal.L1_AUTO_POSITION_GOAL.getPosition())
                                .L1_AUTO_POSITION_GOAL_CLIMBED(
                                        V1_DoomSpiralClimberConstants.ClimberGoal.L1_AUTO_POSITION_GOAL_CLIMBED
                                                .getPosition())
                                .UNCLIMB(V1_DoomSpiralClimberConstants.ClimberGoal.UNCLIMB.getPosition())
                                .L2_POSITION_GOAL(
                                        V1_DoomSpiralClimberConstants.ClimberGoal.L2_POSITION_GOAL.getPosition())
                                .L2_FLIP_GOAL(V1_DoomSpiralClimberConstants.ClimberGoal.L2_FLIP_GOAL.getPosition())
                                .DEFAULT(V1_DoomSpiralClimberConstants.ClimberGoal.DEFAULT.getPosition())
                                .build())

                        .build();
            case V1_DOOMSPIRAL_SIM:
                return null;
            default:
                return null;
        }
    }
}
