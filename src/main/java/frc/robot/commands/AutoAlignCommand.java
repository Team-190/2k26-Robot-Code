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

public class AutoAlignCommand extends Command {
    private final SwerveDrive drive;
    private final Pose2d targetPose;
    private final BooleanSupplier valid;
    private final Supplier<Pose2d> robotPose;

    private ChassisSpeeds speeds;

    private final ProfiledPIDController alignXController;
    private final ProfiledPIDController alignYController;
    private final ProfiledPIDController alignHeadingController;

    public AutoAlignCommand(SwerveDrive drive, Pose2d targetPose, BooleanSupplier valid, Supplier<Pose2d> robotPose) {
        this.addRequirements(drive);
        this.drive = drive;
        this.targetPose = targetPose;
        this.valid = valid;
        this.robotPose = robotPose;

        alignXController =
        new ProfiledPIDController(
            DriveConstants.ALIGN_ROBOT_TO_APRIL_TAG_CONSTANTS.xPIDConstants().kP().get(),
            0.0,
            DriveConstants.ALIGN_ROBOT_TO_APRIL_TAG_CONSTANTS.xPIDConstants().kD().get(),
            new TrapezoidProfile.Constraints(
                DriveConstants.ALIGN_ROBOT_TO_APRIL_TAG_CONSTANTS
                    .xPIDConstants()
                    .maxVelocity()
                    .get(),
                Double.POSITIVE_INFINITY));
        alignYController =
            new ProfiledPIDController(
                DriveConstants.ALIGN_ROBOT_TO_APRIL_TAG_CONSTANTS.yPIDConstants().kP().get(),
                0.0,
                DriveConstants.ALIGN_ROBOT_TO_APRIL_TAG_CONSTANTS.yPIDConstants().kD().get(),
                new TrapezoidProfile.Constraints(
                    DriveConstants.ALIGN_ROBOT_TO_APRIL_TAG_CONSTANTS
                        .yPIDConstants()
                        .maxVelocity()
                        .get(),
                    Double.POSITIVE_INFINITY));
        alignHeadingController =
            new ProfiledPIDController(
                DriveConstants.ALIGN_ROBOT_TO_APRIL_TAG_CONSTANTS.omegaPIDConstants().kP().get(),
                0.0,
                DriveConstants.ALIGN_ROBOT_TO_APRIL_TAG_CONSTANTS.omegaPIDConstants().kD().get(),
                new TrapezoidProfile.Constraints(
                    DriveConstants.ALIGN_ROBOT_TO_APRIL_TAG_CONSTANTS
                        .omegaPIDConstants()
                        .maxVelocity()
                        .get(),
                    Double.POSITIVE_INFINITY));


        alignXController.setTolerance(
            DriveConstants.ALIGN_ROBOT_TO_APRIL_TAG_CONSTANTS.xPIDConstants().tolerance().get());
        alignYController.setTolerance(
            DriveConstants.ALIGN_ROBOT_TO_APRIL_TAG_CONSTANTS.yPIDConstants().tolerance().get());

        alignXController.setTolerance(
            DriveConstants.ALIGN_ROBOT_TO_APRIL_TAG_CONSTANTS.xPIDConstants().tolerance().get());
        alignYController.setTolerance(
            DriveConstants.ALIGN_ROBOT_TO_APRIL_TAG_CONSTANTS.yPIDConstants().tolerance().get());

        alignHeadingController.enableContinuousInput(-Math.PI, Math.PI);
        alignHeadingController.setTolerance(
            DriveConstants.ALIGN_ROBOT_TO_APRIL_TAG_CONSTANTS.omegaPIDConstants().tolerance().get());
        alignHeadingController.setTolerance(
            DriveConstants.ALIGN_ROBOT_TO_APRIL_TAG_CONSTANTS.omegaPIDConstants().tolerance().get());
        
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
        DriveConstants.ALIGN_ROBOT_TO_APRIL_TAG_CONSTANTS.omegaPIDConstants().tolerance().get());

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
