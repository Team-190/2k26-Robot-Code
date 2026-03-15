package frc.robot.subsystems.v2_Delta.shooter;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;

public class SimulatedFuel {
  public Translation3d position;
  public Translation3d velocity;
  public Translation3d angularVelocity;

  private static final double MASS_KG = 0.215;
  private static final double RADIUS_M = 0.075;

  public SimulatedFuel(Translation3d position, Translation3d velocity) {
    this.position = position;
    this.velocity = velocity;
  }

  public void updatePhysics(double deltaTime) {
    Translation3d gravityForce = new Translation3d(0, 0, -9.81 * MASS_KG);

    Translation3d netForce = gravityForce;

    // newtons laws type shi
    Translation3d acceleration = netForce.div(MASS_KG);

    // euler was on ts as a kid fr
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
