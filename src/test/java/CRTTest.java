import static edu.wpi.first.units.Units.Rotations;
import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import frc.robot.subsystems.shared.turret.Turret;
import org.junit.jupiter.api.Test;

public class CRTTest {
  // CRT is precise to every 3 degrees, so there is always up to 1.5 degrees of error.
  // delta represents that necessary tolerance
  private static final double delta = Rotation2d.fromDegrees(0.1).getRadians();

  @Test
  public void crtTestZero() {
    Angle e1 = Angle.ofRelativeUnits((0 % 16) / 16.0, Rotations);
    Angle e2 = Angle.ofRelativeUnits((0 % 17) / 17.0, Rotations);

    Rotation2d expectedTurretRotations = Rotation2d.fromRotations(0.0 / 120.0);
    Rotation2d actualTurretRotations = Turret.calculateTurretAngle(e1, e2);

    assertEquals(expectedTurretRotations.getRadians(), actualTurretRotations.getRadians(), delta);
  }

  @Test
  public void crtTest15() {
    Angle e1 = Angle.ofRelativeUnits(15 / 16.0, Rotations);
    Angle e2 = Angle.ofRelativeUnits(15 / 17.0, Rotations);

    Rotation2d expectedTurretRotations = Rotation2d.fromRotations(0.125);
    Rotation2d actualTurretRotations = Turret.calculateTurretAngle(e1, e2);

    assertEquals(expectedTurretRotations.getRadians(), actualTurretRotations.getRadians(), delta);
  }

  @Test
  public void crtTest30() {
    Angle e1 = Angle.ofRelativeUnits((30 % 16) / 16.0, Rotations);
    Angle e2 = Angle.ofRelativeUnits((30 % 17) / 17.0, Rotations);

    Rotation2d expectedTurretRotations = Rotation2d.fromRotations(30 / 120.0);
    Rotation2d actualTurretRotations = Turret.calculateTurretAngle(e1, e2);

    assertEquals(expectedTurretRotations.getRadians(), actualTurretRotations.getRadians(), delta);
  }

  @Test
  public void crtTest60() {
    Angle e1 = Angle.ofRelativeUnits((60 % 16) / 16.0, Rotations);
    Angle e2 = Angle.ofRelativeUnits((60 % 17) / 17.0, Rotations);

    Rotation2d expectedTurretRotations = Rotation2d.fromRotations(60.0 / 120.0);
    Rotation2d actualTurretRotations = Turret.calculateTurretAngle(e1, e2);

    assertEquals(expectedTurretRotations.getRadians(), actualTurretRotations.getRadians(), delta);
  }

  @Test
  public void crtTest120() {
    Angle e1 = Angle.ofRelativeUnits((120 % 16) / 16.0, Rotations);
    Angle e2 = Angle.ofRelativeUnits((120 % 17) / 17.0, Rotations);

    Rotation2d expectedTurretRotations = Rotation2d.fromRotations(120.0 / 120.0);
    Rotation2d actualTurretRotations = Turret.calculateTurretAngle(e1, e2);

    assertEquals(expectedTurretRotations.getRadians(), actualTurretRotations.getRadians(), delta);
  }

  @Test
  public void crtTest180() {
    Angle e1 = Angle.ofRelativeUnits((180 % 16) / 16.0, Rotations);
    Angle e2 = Angle.ofRelativeUnits((180 % 17) / 17.0, Rotations);

    Rotation2d expectedTurretRotations = Rotation2d.fromRotations(180.0 / 120.0);
    Rotation2d actualTurretRotations = Turret.calculateTurretAngle(e1, e2);

    assertEquals(expectedTurretRotations.getRadians(), actualTurretRotations.getRadians(), delta);
  }

  @Test
  public void crtTestMany() {
    for (double i = 0; i < 16 * 17; i += .1) {
      Angle e1 = Angle.ofRelativeUnits((i % 16) / 16.0, Rotations);
      Angle e2 = Angle.ofRelativeUnits((i % 17) / 17.0, Rotations);

      Rotation2d expectedTurretRotations = Rotation2d.fromRotations(i / 120.0);
      Rotation2d actualTurretRotations = Turret.calculateTurretAngle(e1, e2);

      assertEquals(expectedTurretRotations.getRadians(), actualTurretRotations.getRadians(), delta);
    }
  }
}
