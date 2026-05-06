package edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.team190.gompeilib.core.utility.control.Gains;
import edu.wpi.team190.gompeilib.core.utility.control.constraints.AngularPositionConstraints;
import edu.wpi.team190.gompeilib.core.utility.control.constraints.LinearConstraints;
import edu.wpi.team190.gompeilib.core.utility.tunable.LoggedTunableMeasure;
import java.util.concurrent.locks.ReentrantLock;
import lombok.Builder;
import lombok.NonNull;

@Builder(setterPrefix = "with")
public class SwerveDriveConstants {
  @Builder.Default public final ReentrantLock reentrantLock = new ReentrantLock();

  @NonNull public final DriveConfig driveConfig;

  @NonNull public final Gains driveGains;
  @NonNull public final Gains turnGains;

  @NonNull public final Gains autoTranslationGains;
  @NonNull public final Gains autoRotationGains;

  @NonNull public final AutoAlignConstants autoAlignConstants;

  @NonNull public final Double odometryFrequency;
  @NonNull public final Double driverDeadband;
  @NonNull public final Double operatorDeadband;

  @Builder(setterPrefix = "with")
  public record DriveConfig(
      @NonNull CANBus canBus,
      @NonNull Integer pigeon2Id,
      @NonNull Double maxLinearVelocityMetersPerSecond,
      @NonNull Double wheelRadiusMeters,
      @NonNull DCMotor driveModel,
      @NonNull DCMotor turnModel,
      @NonNull
          SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
              frontLeft,
      @NonNull
          SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
              frontRight,
      @NonNull
          SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
              backLeft,
      @NonNull
          SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
              backRight,
      @NonNull SwerveModuleConstants.ClosedLoopOutputType driveClosedLoopOutputType,
      @NonNull SwerveModuleConstants.ClosedLoopOutputType steerClosedLoopOutputType,
      @NonNull Double bumperWidth,
      @NonNull Double bumperLength,
      @NonNull Double robotMassKilograms,
      @NonNull Double trackWidth,
      @NonNull Double robotMOI,
      @NonNull Double moduleCurrentLimit,
      @NonNull Double wheelCOF) {
    public Double driveBaseRadius() {
      return Math.hypot(
          (Math.abs(frontLeft.LocationX) + Math.abs(frontRight.LocationX)) / 2.0,
          (Math.abs(frontLeft.LocationY) + Math.abs(backLeft.LocationY)) / 2.0);
    }

    public Double maxAngularVelocity() {
      return maxLinearVelocityMetersPerSecond / driveBaseRadius();
    }

    public Translation2d[] getModuleTranslations() {
      return new Translation2d[] {
        new Translation2d(frontLeft.LocationX, frontLeft.LocationY),
        new Translation2d(frontRight.LocationX, frontRight.LocationY),
        new Translation2d(backLeft.LocationX, backLeft.LocationY),
        new Translation2d(backRight.LocationX, backRight.LocationY)
      };
    }

    public SwerveDriveKinematics kinematics() {
      return new SwerveDriveKinematics(getModuleTranslations());
    }
  }

  @Builder(setterPrefix = "with")
  public record AutoAlignConstants(
      @NonNull Gains xGains,
      @NonNull LinearConstraints xConstraints,
      @NonNull Gains yGains,
      @NonNull LinearConstraints yConstraints,
      @NonNull Gains rotationGains,
      @NonNull AngularPositionConstraints rotationConstraints,
      @NonNull LoggedTunableMeasure<DistanceUnit> linearThreshold,
      @NonNull LoggedTunableMeasure<AngleUnit> angularThreshold) {}
}
