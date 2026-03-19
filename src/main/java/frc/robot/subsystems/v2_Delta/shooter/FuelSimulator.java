package frc.robot.subsystems.v2_Delta.shooter;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.team190.gompeilib.subsystems.generic.flywheel.GenericFlywheelConstants;
import edu.wpi.team190.gompeilib.subsystems.generic.flywheel.GenericFlywheelIOInputsAutoLogged;
import edu.wpi.team190.gompeilib.subsystems.generic.roller.GenericRollerIOInputsAutoLogged;
import frc.robot.FieldConstants;
import frc.robot.subsystems.shared.hood.HoodIOInputsAutoLogged;
import frc.robot.subsystems.v1_DoomSpiral.spindexer.*;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class FuelSimulator {
  private List<SimulatedFuel> activeShots = new ArrayList<>();
  public int shotsMade = 0;
  public int shotsMissed = 0;

  private final GenericFlywheelIOInputsAutoLogged flywheelInputs;
  private final HoodIOInputsAutoLogged hoodInputs;
  private final GenericRollerIOInputsAutoLogged feederInputs;
  private final V1_DoomSpiralSpindexerIOInputsAutoLogged spindexerInputs;

  private Rotation2d hoodPitch;
  private double spindexerVelocity;
  private double flywheelVelocity;
  private double feederVelocity;

  private static final double SPINDEXER_RADIUS = Units.inchesToMeters(19.45);
  private static final double FRICTION_COEFF = 0.7; // Coeff between rubber and foam is between (0.5 and 0.9). Avg = 0.7.

  private static final Translation3d HUB_CENTER = FieldConstants.Hub.innerCenterPoint;
  private static final double HUB_HEIGHT_Z = FieldConstants.Hub.height;
  private static final double HUB_RADIUS = FieldConstants.Hub.innerWidth / 2;

  private static final double MOTOR_WORK = 70.85;
  private static final double TURRET_X_OFFSET = 2.3; // TODO: get actual values for these
  private static final double TURRET_Z_HEIGHT = 0.6; // TODO: get actual values for these

  private static final double FLYWHEEL_MASS = 0.27;
  private static final double FLYWHEEL_RADIUS = Units.inchesToMeters(2); // TODO: Account for the metal rod in code
  

  private static final double FEEDER_MASS = 0.23; // Check value
  
  private static final double FEEDER_RADIUS = Units.inchesToMeters(0.9825);
  private static double FEEER_MOMENT_OF_INTERTIA = 0.5 * FEEDER_MASS * Math.pow(FEEDER_RADIUS, 2);

  private static final double KICKER_MASS = 0.0;

  private static final double FUEL_MASS = 0.23;
  private static final double FUEL_RADIUS = Units.inchesToMeters(2.95);
  private static final double FUEL_MOMENT_OF_INERTIA = 0.4 * FUEL_MASS * Math.pow(FUEL_RADIUS, 2);

  private static final double DISPLACEMENT = 0.05; // Distances force of friction between fuel and roller/flywheel.

  private static final double ROBOT_HEIGHT = 4;

  public FuelSimulator(V1_DoomSpiralSpindexerIOInputsAutoLogged spindexerInputs, GenericFlywheelIOInputsAutoLogged flywheelInputs, GenericRollerIOInputsAutoLogged feederInputs, HoodIOInputsAutoLogged hoodInputs) {
    this.feederInputs = feederInputs;
    this.spindexerInputs = spindexerInputs;
    this.flywheelInputs = flywheelInputs;
    this.hoodInputs = hoodInputs;

    spindexerVelocity = spindexerInputs.velocity.baseUnitMagnitude();
    flywheelVelocity = flywheelInputs.velocityRadiansPerSecond;
    feederVelocity = feederInputs.velocityRadiansPerSecond;
    hoodPitch = hoodInputs.position;
  }

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
    double vx = getInitialVelocityX();
    double vy = getInitialVelocityY();
    double vz = getInitialVelocityZ();
    Translation3d shooterVel = new Translation3d(vx, vy, vz);

    return shooterVel;
  }

  public void periodic() {

    feederVelocity = feederInputs.velocityRadiansPerSecond;
    flywheelVelocity = flywheelInputs.velocityRadiansPerSecond;
    spindexerVelocity = spindexerInputs.velocity.baseUnitMagnitude();

    hoodPitch = hoodInputs.position;

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
    double collisionTime =
        (getInitialVelocityZ() + Math.sqrt(Math.pow(getInitialVelocityZ(), 2))) / 9.8;
    double FUEL_X_FINAL =
        TURRET_X_OFFSET
            + getInitialVelocityX()
                * Math.cos(hoodPitch.getRadians())
                * Math.sqrt(
                    Math.pow(getInitialVelocityX() * Math.sin(hoodPitch.getRadians()), 2)
                        + 19.6 * (ROBOT_HEIGHT - FieldConstants.Hub.innerHeight))
                / 9.8;
    boolean atRimHeight = Math.abs(fuel.position.getZ() - HUB_HEIGHT_Z) < 0.1;
    double distanceToHub =
        new Translation3d(FUEL_X_FINAL, fuel.position.getY(), fuel.position.getZ())
            .getDistance(HUB_CENTER);
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

  private static double velocityFunction(double massOfRollerOrFlywheel, double previousVelocity) {
    return Math.sqrt(
        (2 * (FRICTION_COEFF * massOfRollerOrFlywheel * 9.8 * DISPLACEMENT)
                    + Math.pow(previousVelocity, 2)
                        * (FUEL_MASS + (FUEL_MOMENT_OF_INERTIA / FUEL_RADIUS)))
                / FUEL_MASS
            + (FUEL_MOMENT_OF_INERTIA / Math.pow(FUEL_RADIUS, 2)));
  }

  private double getInitialVelocity() {
    double startingVelocity =
        (FUEL_MOMENT_OF_INERTIA * (spindexerVelocity / SPINDEXER_RADIUS))
            / (FUEL_MASS * SPINDEXER_RADIUS / 2);
    double initialVelocity =
        velocityFunction(
            FLYWHEEL_MASS,
            velocityFunction(FEEDER_MASS, velocityFunction(KICKER_MASS, startingVelocity)));

    return initialVelocity;
  }

  private double getInitialVelocityX() {
    return getInitialVelocity()
        * Math.cos(
            hoodInputs.position
                .getDegrees()); // Not accounting for friction between fuel and flywheel
  }

  private double getInitialVelocityY() {
    return getInitialVelocity()
        * Math.sin(
            hoodInputs.position
                .getDegrees()); // Not accounting for friction between fuel and flywheel
  }

  private double getInitialVelocityZ() {
    return getInitialVelocityY();
  }
}
