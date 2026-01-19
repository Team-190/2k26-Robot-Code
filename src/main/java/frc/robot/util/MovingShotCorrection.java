package frc.robot.util;



import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import java.util.function.Function;

public interface MovingShotCorrection {
    /**
     * Calculates a corrected pose for a moving target based on the shooter's current velocity.
     * 
     * @param initialPose The current pose of the shooter.
     * @param targetPose The pose of the target.
     * @param velocityMetersPerSecond The shooter's velocity in meters per second.
     * @param distanceToTimeFunction A function that converts distance to time.
     * @return The corrected pose to aim at.
     */

    public default Translation2d getCorrection(Pose2d initialPose, 
                                              Pose2d targetPose, 
                                              ChassisSpeeds velocityMetersPerSecond, 
                                              Function<Double, Double> distanceToTimeFunction,
                                              Transform2d centerToShooterCenter) {



        Pose2d shooterPose = new Pose2d(initialPose.getX() + initialPose.getRotation().getCos() * centerToShooterCenter.getX() - initialPose.getRotation().getSin() * centerToShooterCenter.getY(),
                                        initialPose.getY() + initialPose.getRotation().getSin() * centerToShooterCenter.getX() + initialPose.getRotation().getCos() * centerToShooterCenter.getY(),
                                        initialPose.getRotation().plus(centerToShooterCenter.getRotation()));

        Transform2d shooterToTarget = new Transform2d(shooterPose, targetPose);

        double shooterXVelocityMetersPerSecond = centerToShooterCenter.getTranslation().getNorm() * 
        velocityMetersPerSecond.omegaRadiansPerSecond * -centerToShooterCenter.getRotation().getSin() + 
        velocityMetersPerSecond.vxMetersPerSecond;

        double shooterYVelocityMetersPerSecond = centerToShooterCenter.getTranslation().getNorm() * 
        velocityMetersPerSecond.omegaRadiansPerSecond * centerToShooterCenter.getRotation().getCos() +
        velocityMetersPerSecond.vyMetersPerSecond;

        double xVelocityMetersPerSecond = shooterXVelocityMetersPerSecond * initialPose.getRotation().getCos() - shooterYVelocityMetersPerSecond * initialPose.getRotation().getSin() + velocityMetersPerSecond.vxMetersPerSecond;
        double yVelocityMetersPerSecond = shooterYVelocityMetersPerSecond * initialPose.getRotation().getCos() + shooterXVelocityMetersPerSecond * initialPose.getRotation().getSin() + velocityMetersPerSecond.vyMetersPerSecond;

        double correctedX = targetPose.getX() - xVelocityMetersPerSecond * distanceToTimeFunction.apply(shooterToTarget.getTranslation().getNorm());
        double correctedY = targetPose.getY() - yVelocityMetersPerSecond * distanceToTimeFunction.apply(shooterToTarget.getTranslation().getNorm());
        
        return new Translation2d(correctedX, correctedY);
    }
}