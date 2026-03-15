package frc.robot.subsystems.v2_Delta.shooter;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.FieldConstants;
import frc.robot.subsystems.shared.hood.HoodIO;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class FuelSimulator {
  private List<SimulatedFuel> activeShots = new ArrayList<>();
  public int shotsMade = 0;
  public int shotsMissed = 0;
  private static HoodIO hoodIO;

  private static final double TURRET_X_OFFSET = 0.0; // TODO: get actual values for these
  private static final double TURRET_Z_HEIGHT = 0.6;

  private static final Translation2d HUB_CENTER = new Translation2d(8.23, 4.11);
  private static final double HUB_HEIGHT_Z = 2.64;
  private static final double HUB_RADIUS = 0.6;

  private static final double HEIGHT_ROBOT = 4;

  private static final Rotation2d hoodPitch =
      hoodIO.getPosition(); // Replace with actual value from log

  private static final double INITIAL_VELOCITY_X = 0; // Get actual velocity from log
  private static final double INITIAL_VELOCITY_Y = 0; // Get actual velocity from log
  private static final double INITIAL_VELOCITY_Z = 0; // Get actual velocity from log

  public void fireShot(
      Pose2d robotPose,
      ChassisSpeeds chassisSpeeds,
      Rotation2d hoodPitch,
      double flywheelSurfaceSpeed) {
    Translation3d initialPos = calculateGlobalLaunchPosition(robotPose);
    Translation3d initialVel = calculateGlobalLaunchVelocity(hoodPitch);

    activeShots.add(new SimulatedFuel(initialPos, initialVel));
  }

  private Translation3d calculateGlobalLaunchPosition(Pose2d robotPose) {
    return new Translation3d(robotPose.getX() + TURRET_X_OFFSET, robotPose.getY(), TURRET_Z_HEIGHT);
  }

  private Translation3d calculateGlobalLaunchVelocity(Rotation2d hoodPitch) {

    // Shooter exit velocity vector
    double vx = INITIAL_VELOCITY_X;
    double vy = INITIAL_VELOCITY_Y;
    double vz = INITIAL_VELOCITY_Z;
    Translation3d shooterVel = new Translation3d(vx, vy, vz);

    return shooterVel;
  }

  public void periodic() {
    Iterator<SimulatedFuel> iterator = activeShots.iterator();
    while (iterator.hasNext()) {
      SimulatedFuel fuel = iterator.next();
      fuel.updatePhysics(0.02);

      if (checkHubCollision(fuel, hoodPitch)) {
        shotsMade++;
        iterator.remove();
      } else if (fuel.isBelowFloor()) {
        shotsMissed++;
        iterator.remove();
      }
    }
    logToAdvantageKit();
  }

  private boolean checkHubCollision(SimulatedFuel fuel, Rotation2d hoodPitch) {
    double collisionTime = (INITIAL_VELOCITY_Z + Math.sqrt(Math.pow(INITIAL_VELOCITY_Z, 2))) / 9.8;
    double FUEL_X_FINAL =
        TURRET_X_OFFSET
            + INITIAL_VELOCITY_X
                * Math.cos(hoodPitch.getRadians())
                * Math.sqrt(
                    Math.pow(INITIAL_VELOCITY_X * Math.sin(hoodPitch.getRadians()), 2)
                        + 19.6 * (HEIGHT_ROBOT - FieldConstants.Hub.innerHeight))
                / 9.8;
    boolean atRimHeight = Math.abs(fuel.position.getZ() - HUB_HEIGHT_Z) < 0.1;
    double distanceToHub =
        new Translation2d(FUEL_X_FINAL, fuel.position.getY()).getDistance(HUB_CENTER);
    return atRimHeight && (distanceToHub < HUB_RADIUS);
  }

  private void logToAdvantageKit() {
    Pose3d[] poses = new Pose3d[activeShots.size()];
    for (int i = 0; i < activeShots.size(); i++) {
      poses[i] = activeShots.get(i).getPose();
    }
    Logger.recordOutput("FuelSimulator/ActiveFuel", poses);
    Logger.recordOutput("FuelSimulator/ShotsMade", shotsMade);
    Logger.recordOutput("FuelSimulator/ShotsMissed", shotsMissed);
  }
}
