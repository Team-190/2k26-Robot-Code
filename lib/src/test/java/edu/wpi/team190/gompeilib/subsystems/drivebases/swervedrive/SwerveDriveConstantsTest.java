package edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive;

import static org.junit.jupiter.api.Assertions.*;
import static org.wpilib.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerFeedbackType;
import edu.wpi.team190.gompeilib.core.utility.control.Gains;
import edu.wpi.team190.gompeilib.core.utility.control.constraints.AngularPositionConstraints;
import edu.wpi.team190.gompeilib.core.utility.control.constraints.LinearConstraints;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.SwerveDriveKinematics;
import org.wpilib.math.system.DCMotor;
import org.wpilib.tunable.Tunable;

public class SwerveDriveConstantsTest {

  public static SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      createModuleConstants(int driveId, int steerId, int encoderId, double x, double y) {
    SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> c =
        new SwerveModuleConstants<>();
    c.DriveMotorId = driveId;
    c.SteerMotorId = steerId;
    c.EncoderId = encoderId;
    c.LocationX = x;
    c.LocationY = y;
    c.DriveMotorGearRatio = 6.0;
    c.SteerMotorGearRatio = 10.0;
    c.SteerInertia = 0.01;
    c.DriveInertia = 0.01;
    c.EncoderOffset = 0.0;
    c.FeedbackSource = SteerFeedbackType.FusedCANcoder;
    c.DriveMotorClosedLoopOutput = ClosedLoopOutputType.Voltage;
    c.SteerMotorClosedLoopOutput = ClosedLoopOutputType.Voltage;
    c.EncoderInitialConfigs = new CANcoderConfiguration();
    c.DriveMotorGains = new Slot0Configs();
    c.SteerMotorGains = new Slot0Configs();
    return c;
  }

  @Test
  public void testConstantsAndBuilders() {
    CANBus canBus = new CANBus("rio");
    DCMotor driveModel = DCMotor.getKrakenX60(1);
    DCMotor turnModel = DCMotor.getKrakenX44(1);

    var fl = createModuleConstants(1, 2, 3, 0.5, 0.5);
    var fr = createModuleConstants(4, 5, 6, 0.5, -0.5);
    var bl = createModuleConstants(7, 8, 9, -0.5, 0.5);
    var br = createModuleConstants(10, 11, 12, -0.5, -0.5);

    SwerveDriveConstants.DriveConfig driveConfig =
        SwerveDriveConstants.DriveConfig.builder()
            .withCanBus(canBus)
            .withPigeon2Id(13)
            .withMaxLinearVelocityMetersPerSecond(5.0)
            .withWheelRadiusMeters(0.05)
            .withDriveModel(driveModel)
            .withTurnModel(turnModel)
            .withFrontLeft(fl)
            .withFrontRight(fr)
            .withBackLeft(bl)
            .withBackRight(br)
            .withDriveClosedLoopOutputType(ClosedLoopOutputType.Voltage)
            .withSteerClosedLoopOutputType(ClosedLoopOutputType.Voltage)
            .withBumperWidth(0.8)
            .withBumperLength(0.8)
            .withRobotMassKilograms(45.0)
            .withTrackWidth(0.6)
            .withRobotMOI(2.5)
            .withModuleCurrentLimit(40.0)
            .withWheelCOF(1.2)
            .build();

    Gains driveGains = Gains.builder().withKP(1.0).withKD(0.1).build();

    Gains turnGains = Gains.builder().withKP(2.0).withKD(0.2).build();

    SwerveDriveConstants.AutoAlignConstants autoAlignConstants =
        SwerveDriveConstants.AutoAlignConstants.builder()
            .withXGains(driveGains)
            .withXConstraints(
                LinearConstraints.builder()
                    .withMaxVelocity(MetersPerSecond.of(1.0))
                    .withMaxAcceleration(MetersPerSecondPerSecond.of(1.0))
                    .withGoalTolerance(Meters.of(0.05))
                    .build())
            .withYGains(driveGains)
            .withYConstraints(
                LinearConstraints.builder()
                    .withMaxVelocity(MetersPerSecond.of(1.0))
                    .withMaxAcceleration(MetersPerSecondPerSecond.of(1.0))
                    .withGoalTolerance(Meters.of(0.05))
                    .build())
            .withRotationGains(turnGains)
            .withRotationConstraints(
                AngularPositionConstraints.builder()
                    .withMaxVelocity(RadiansPerSecond.of(1.0))
                    .withMaxAcceleration(RadiansPerSecondPerSecond.of(1.0))
                    .withGoalTolerance(Radians.of(0.05))
                    .build())
            .withLinearThreshold(Tunable.create(Meters.of(0.01)))
            .withAngularThreshold(Tunable.create(Radians.of(0.01)))
            .build();

    SwerveDriveConstants constants =
        SwerveDriveConstants.builder()
            .withDriveConfig(driveConfig)
            .withDriveGains(driveGains)
            .withTurnGains(turnGains)
            .withAutoTranslationGains(driveGains)
            .withAutoRotationGains(turnGains)
            .withAutoAlignConstants(autoAlignConstants)
            .withOdometryFrequency(250.0)
            .withDriverDeadband(0.1)
            .withOperatorDeadband(0.1)
            .build();

    assertNotNull(constants);
    assertEquals(250.0, constants.odometryFrequency);
    assertEquals(0.1, constants.driverDeadband);
    assertEquals(0.1, constants.operatorDeadband);

    // Verify DriveConfig calculations
    double expectedRadius = Math.hypot(0.5, 0.5);
    assertEquals(expectedRadius, driveConfig.driveBaseRadius(), 1e-6);
    assertEquals(5.0 / expectedRadius, driveConfig.maxAngularVelocity(), 1e-6);

    Translation2d[] translations = driveConfig.getModuleTranslations();
    assertEquals(4, translations.length);
    assertEquals(new Translation2d(0.5, 0.5), translations[0]);
    assertEquals(new Translation2d(0.5, -0.5), translations[1]);
    assertEquals(new Translation2d(-0.5, 0.5), translations[2]);
    assertEquals(new Translation2d(-0.5, -0.5), translations[3]);

    SwerveDriveKinematics kinematics = driveConfig.kinematics();
    assertNotNull(kinematics);
  }
}
