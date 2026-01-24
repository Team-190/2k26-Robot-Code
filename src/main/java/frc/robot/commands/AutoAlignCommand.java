package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveDrive;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveDriveConstants;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveDriveConstants.AutoAlignNearConstants;


public class AutoAlignCommand extends Command {
    private final SwerveDrive drive;
    private final Pose2d targetPose;
    private final BooleanSupplier valid;
    private final Supplier<Pose2d> robotPose;
    private final AutoAlignNearConstants constants;

    private ChassisSpeeds speeds;

    private final ProfiledPIDController alignXController;
    private final ProfiledPIDController alignYController;
    private final ProfiledPIDController alignHeadingController;

    public AutoAlignCommand(SwerveDrive drive, Pose2d targetPose, BooleanSupplier valid, Supplier<Pose2d> robotPose, AutoAlignNearConstants constants) {
        this.addRequirements(drive);
        this.drive = drive;
        this.targetPose = targetPose;
        this.valid = valid;
        this.robotPose = robotPose;
        this.constants = constants;

        alignXController =
        new ProfiledPIDController(
            constants.xPIDConstants().kP().get(),
            0.0,
            constants.xPIDConstants().kD().get(),
            new TrapezoidProfile.Constraints(
                constants
                    .xPIDConstants()
                    .maxVelocity()
                    .get(),
                Double.POSITIVE_INFINITY));
        alignYController =
            new ProfiledPIDController(
                constants.yPIDConstants().kP().get(),
                0.0,
                constants.yPIDConstants().kD().get(),
                new TrapezoidProfile.Constraints(
                    constants
                        .yPIDConstants()
                        .maxVelocity()
                        .get(),
                    Double.POSITIVE_INFINITY));
        alignHeadingController =
            new ProfiledPIDController(
                constants.omegaPIDConstants().kP().get(),
                0.0,
                constants.omegaPIDConstants().kD().get(),
                new TrapezoidProfile.Constraints(
                    constants
                        .omegaPIDConstants()
                        .maxVelocity()
                        .get(),
                    Double.POSITIVE_INFINITY));


        alignXController.setTolerance(
            constants.xPIDConstants().tolerance().get());
        alignYController.setTolerance(
            constants.yPIDConstants().tolerance().get());

        alignXController.setTolerance(
            constants.xPIDConstants().tolerance().get());
        alignYController.setTolerance(
            constants.yPIDConstants().tolerance().get());

        alignHeadingController.enableContinuousInput(-Math.PI, Math.PI);
        alignHeadingController.setTolerance(
            constants.omegaPIDConstants().tolerance().get());
        alignHeadingController.setTolerance(
            constants.omegaPIDConstants().tolerance().get());
        
        speeds = new ChassisSpeeds();

    }

    @Override
    public void initialize() {
        if (!valid.getAsBoolean()) {
          alignHeadingController.reset(
              robotPose.get().getRotation().getRadians());
          alignXController.reset(robotPose.get().getX());
          alignYController.reset(robotPose.get().getY());
        }
    }

    @Override
    public void execute() {

                      if (valid.getAsBoolean()) {
                        double xSpeed = 0.0;
                        double ySpeed = 0.0;

                        double ex =
                            targetPose.getX()
                                - robotPose.get().getX();
                        double ey =
                            targetPose.getY()
                                - robotPose.get().getY();

                        // Rotate errors into the reef post's coordinate frame
                        double ex_prime =
                            ex
                                    * Math.cos(
                                        targetPose
                                            .getRotation()
                                            .getRadians())
                                + ey
                                    * Math.sin(
                                        targetPose
                                            .getRotation()
                                            .getRadians());
                        double ey_prime =
                            -ex
                                    * Math.sin(
                                        targetPose
                                            .getRotation()
                                            .getRadians())
                                + ey
                                    * Math.cos(
                                        targetPose
                                            .getRotation()
                                            .getRadians());

                        ChassisSpeeds measuredSpeeds = drive.getMeasuredChassisSpeeds();
                        double vx_prime =
                            measuredSpeeds.vxMetersPerSecond
                                    * Math.cos(
                                        targetPose
                                            .getRotation()
                                            .getRadians())
                                + measuredSpeeds.vyMetersPerSecond
                                    * Math.sin(
                                        targetPose
                                            .getRotation()
                                            .getRadians());

                        double vy_prime =
                            -measuredSpeeds.vxMetersPerSecond
                                    * Math.sin(
                                        targetPose
                                            .getRotation()
                                            .getRadians())
                                + measuredSpeeds.vyMetersPerSecond
                                    * Math.cos(
                                        targetPose
                                            .getRotation()
                                            .getRadians());

                        if (!alignXController.atSetpoint()) {
                          xSpeed = alignXController.calculate(0, ex_prime);
                        } else {
                          alignXController.reset(ex_prime, vx_prime);
                        }

                        if (!alignYController.atSetpoint()) {
                          ySpeed = alignYController.calculate(0, ey_prime);

                        } else {
                          alignYController.reset(ey_prime, vy_prime);
                        }

                        // Re-rotate the speeds into field relative coordinate frame
                        double adjustedXSpeed =
                            xSpeed
                                    * Math.cos(
                                        targetPose
                                            .getRotation()
                                            .getRadians())
                                - ySpeed
                                    * Math.sin(
                                        targetPose
                                            .getRotation()
                                            .getRadians());
                        double adjustedYSpeed =
                            xSpeed
                                    * Math.sin(
                                        targetPose
                                            .getRotation()
                                            .getRadians())
                                + ySpeed
                                    * Math.cos(
                                        targetPose
                                            .getRotation()
                                            .getRadians());
                        speeds =
                            ChassisSpeeds.fromFieldRelativeSpeeds(
                                -adjustedXSpeed,
                                -adjustedYSpeed,
                                reefThetaSpeedCalculate(),
                                robotPose.get()
                                    .getRotation()
                                    .plus(new Rotation2d(Math.PI)));

                      } else {
                        speeds = new ChassisSpeeds();
                      }
                      Logger.recordOutput("Drive/Auto Align/xSpeed", -speeds.vxMetersPerSecond);
                      Logger.recordOutput("Drive/Auto Align/ySpeed", -speeds.vyMetersPerSecond);
                      Logger.recordOutput("Drive/Auto Align/thetaSpeed", speeds.omegaRadiansPerSecond);
                      drive.runVelocity(speeds);
                      
    }

    @Override
    public void end(boolean interrupted) {
                      drive.runVelocity(new ChassisSpeeds());
                      alignHeadingController.reset(
                          robotPose.get().getRotation().getRadians());
                      alignXController.reset(robotPose.get().getX());
                      alignYController.reset(robotPose.get().getY());
    }

    @Override
    public boolean isFinished() {
        // Return true when the command should end
        return Math.abs(targetPose.getX() - robotPose.get().getX()) < 0.1 &&
               Math.abs(targetPose.getY() - robotPose.get().getY()) < 0.1 &&
               Math.abs(targetPose.getRotation().getDegrees() - robotPose.get().getRotation().getDegrees()) < 5.0;
    }

      private double reefThetaSpeedCalculate() {
    double thetaSpeed = 0.0;

    alignHeadingController.setTolerance(
        constants.omegaPIDConstants().tolerance().get());

    alignHeadingController.enableContinuousInput(-Math.PI, Math.PI);

    if (!alignHeadingController.atSetpoint())
      thetaSpeed =
          alignHeadingController.calculate(
              robotPose.get().getRotation().getRadians(),
              targetPose.getRotation().getRadians());
    else alignHeadingController.reset(robotPose.get().getRotation().getRadians());

    return thetaSpeed;
  }
}
