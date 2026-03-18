package frc.robot.subsystems.v2_Delta.shooter;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

// Source for physics behind drag:
// http://math.libretexts.org/Bookshelves/Differential_Equations/A_First_Course_in_Differential_Equations_for_Scientists_and_Engineers_(Herman)/03%3A_Numerical_Solutions/3.05%3A_Numerical_Applications/3.5.03%3A_The_Flight_of_Sports_Balls

public class SimulatedFuel {
  public Translation3d position;
  public Translation3d velocity;
  public Translation3d angularVelocity;

  private static final double MASS_KG = 0.215;
  private static final double RADIUS_M = 0.075;
  private static final double AIR_DENSITY = 1.2793; // Check if this is right
  private static final double CL = 0.45;
  private static final double CD = 0.2;
  private static final double FUEL_RADIUS = Units.inchesToMeters(2.95);
  private static final double AREA = Math.PI * FUEL_RADIUS;
  private static final double ALPHA = (AIR_DENSITY * AREA) / (MASS_KG);

  public SimulatedFuel(Translation3d position, Translation3d velocity) {
    this.position = position;
    this.velocity = velocity;
  }

  public void updatePhysics(double deltaTime) {

    // Magnitude of velocity
    double speed = velocity.getNorm();

    // Drag acceleration
    Translation3d dragAccel = velocity.times(-ALPHA * CD * speed);

    // Lift Acceleration Vector (Perpendicular to velocity and spin)
    // If shooting forward (+X) with backspin, the spin axis points left (-Y).
    Translation3d spinAxis = new Translation3d(0, -1, 0);

    // Calculate cross product between lift acceleration vector and velocity vector
    double liftX = (spinAxis.getY() * velocity.getZ()) - (spinAxis.getZ() * velocity.getY());
    double liftY = (spinAxis.getZ() * velocity.getX()) - (spinAxis.getX() * velocity.getZ());
    double liftZ = (spinAxis.getX() * velocity.getY()) - (spinAxis.getY() * velocity.getX());
    Translation3d liftDirection = new Translation3d(liftX, liftY, liftZ);

    // a_lift = α * CL * v * (spinAxis × velocity)
    Translation3d liftAccel = liftDirection.times(ALPHA * CL * speed);

    // Gravity vector
    Translation3d gravityAccel = new Translation3d(0, 0, -9.81);

    // Sum all acceleration vectors
    Translation3d acceleration = gravityAccel.plus(liftAccel).plus(dragAccel);

    // Get final updated velocity and position vector
    velocity = velocity.plus(acceleration.times(deltaTime));
    position = position.plus(velocity.times(deltaTime));
  }

  public Pose3d getPose() {
    return new Pose3d(position, new Rotation3d());
  }

  public boolean isBelowFloor() {
    return position.getZ() <= 0.0;
  }
}
