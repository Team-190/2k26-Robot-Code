package first.robot.v0_Funky.subsystems.shooter;

import static org.wpilib.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.team190.gompeilib.core.utility.control.CurrentLimits;
import edu.wpi.team190.gompeilib.core.utility.control.Gains;
import edu.wpi.team190.gompeilib.core.utility.control.constraints.AngularPositionConstraints;
import edu.wpi.team190.gompeilib.core.utility.control.constraints.AngularVelocityConstraints;
import edu.wpi.team190.gompeilib.subsystems.generic.flywheel.GenericFlywheelConstants;
import first.robot.shared.subsystems.turret.TurretConstants;
import first.robot.shared.subsystems.turret.TurretConstants.TurretAngleCalculation;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.system.DCMotor;

public class V0_FunkyShooterConstants {
  public static final GenericFlywheelConstants SHOOT_CONSTANTS =
      GenericFlywheelConstants.builder()
          .withLeaderCANID(30)
          .withCanBus(new CANBus())
          .withLeaderInversion(InvertedValue.CounterClockwise_Positive)
          .withCurrentLimit(
              CurrentLimits.builder()
                  .withSupplyCurrentLimit(Amps.of(40.0))
                  .withStatorCurrentLimit(Amps.of(80))
                  .build())
          .withMomentOfInertia(0.05)
          .withGearRatio(1.0)
          .withMotorConfig(DCMotor.getKrakenX60Foc(2))
          .withVoltageGains(
              Gains.builder()
                  .withPrefix("Shooter/Flywheel/Voltage")
                  .withKP(0.0)
                  .withKD(0.0)
                  .withKS(0.0)
                  .withKV(0.0)
                  .withKA(0.0)
                  .build())
          .withTorqueGains(
              Gains.builder()
                  .withPrefix("Shooter/Flywheel/Torque")
                  .withKP(0.1)
                  .withKI(0.0)
                  .withKD(0.0)
                  .withKS(0.2)
                  .withKV(0.12)
                  .withKA(0.0)
                  .withKG(0.0)
                  .build())
          .withConstraints(
              AngularVelocityConstraints.builder()
                  .withPrefix("Shooter/Flywheel")
                  .withMaxAcceleration(RadiansPerSecondPerSecond.of(6))
                  .withMaxVelocity(RadiansPerSecond.of(4))
                  .withGoalTolerance(RadiansPerSecond.of(5))
                  .build())
          .withVelocityOffsetStep(RadiansPerSecond.of(5))
          .withVoltageOffsetStep(Volts.of(1))
          .withEnableFOC(false)
          .build();

  public static final TurretConstants TURRET_CONSTANTS =
      TurretConstants.builder()
          .withMotorConfig(DCMotor.getKrakenX60Foc(1))
          .withMomentOfInertia(0.004)
          .withTurretCANID(18)
          .withMotorInversion(InvertedValue.Clockwise_Positive)
          .withEncoderInversion(SensorDirectionValue.Clockwise_Positive)
          .withCanBus(new CANBus())
          .withEncoder1ID(17)
          .withEncoder2ID(16)
          .withMaxAngle(Rotation2d.fromRadians(2 * Math.PI))
          .withMinAngle(Rotation2d.fromRadians(-2 * Math.PI))
          .withGearRatio(120.0 / 20)
          .withSupplyCurrentLimit(30.0)
          .withStatorCurrentLimit(30.0)
          .withRobotToTurretTransform(new Transform3d())
          .withE1Offset(
              Rotation2d.fromRotations(-0.521973)
                  .minus(Rotation2d.fromDegrees(309.726563 + 301.201172 + 88.681641)))
          .withE2Offset(
              Rotation2d.fromRotations(-0.44458)
                  .minus(Rotation2d.fromDegrees(325.371094 + 233.261719)))
          .withGains(
              Gains.builder()
                  .withPrefix("Turret")
                  .withKP(28.624920)
                  .withKD(0.5)
                  .withKS(0.158040)
                  .withKV(0.11377)
                  .withKA(0.0031713)
                  .build())
          .withConstraints(
              AngularPositionConstraints.builder()
                  .withPrefix("Turret")
                  .withMaxAcceleration(RadiansPerSecondPerSecond.of(35.566371))
                  .withMaxVelocity(RadiansPerSecond.of(89.566371))
                  .withGoalTolerance(Degrees.of(3))
                  .build())
          .withAimingFeedforwardGains(
              Gains.builder()
                  .withPrefix("Turret/Aiming")
                  .withKS(0.5)
                  .withKV(0.1)
                  .withKA(0.025)
                  .build())
          .withTurretAngleCalculation(
              TurretAngleCalculation.builder()
                  .withGear0ToothCount(120)
                  .withGear1ToothCount(17)
                  .withGear2ToothCount(16)
                  .build())
          .withVoltageStep(Volts.of(0.5))
          .withAngleStep(Rotation2d.fromDegrees(1.0))
          .build();
}
