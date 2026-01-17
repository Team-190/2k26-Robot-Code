package frc.robot.util;



import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import java.util.function.Function;

class MovingShotCorrection {
    /**
     * Calculates a corrected pose for a moving target based on the shooter's current velocity.
     * 
     * @param initialPose The current pose of the shooter.
     * @param targetPose The pose of the target.
     * @param velocityMetersPerSecond The shooter's velocity in meters per second.
     * @param distanceToTimeFunction A function that converts distance to time.
     * @return The corrected pose to aim at.
     */

    public static Translation2d getCorrection(Pose2d initialPose, 
                                              Pose2d targetPose, 
                                              ChassisSpeeds velocityMetersPerSecond, 
                                              Function<Double, Double> distanceToTimeFunction,
                                              Transform2d centerToShooterCenter) {
          
        double distanceMeters = initialPose.getTranslation().getDistance(targetPose.getTranslation());
        double time = distanceToTimeFunction.apply(distanceMeters);
        Translation2d shooterOffset = centerToShooterCenter.getTranslation();
        Rotation2d changeInAngle = initialPose.getRotation().plus(centerToShooterCenter.getRotation());

        double xVelocityMetersPerSecond = shooterOffset.getNorm() * 
        velocityMetersPerSecond.omegaRadiansPerSecond * changeInAngle.plus(new Rotation2d(Math.PI/2)).getCos() + 
        velocityMetersPerSecond.vxMetersPerSecond;

        double yVelocityMetersPerSecond = shooterOffset.getNorm() * 
        velocityMetersPerSecond.omegaRadiansPerSecond * changeInAngle.plus(new Rotation2d(Math.PI/2)).getSin() + 
        velocityMetersPerSecond.vyMetersPerSecond;

        double correctedX = targetPose.getX() - xVelocityMetersPerSecond * time;
        double correctedY = targetPose.getY() - yVelocityMetersPerSecond * time;
        
        return new Translation2d(correctedX, correctedY);
    }
}