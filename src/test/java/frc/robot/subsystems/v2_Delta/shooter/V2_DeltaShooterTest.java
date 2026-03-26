package frc.robot.subsystems.v2_Delta.shooter;

import static edu.wpi.first.units.Units.*;
import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import org.junit.jupiter.api.*;

@TestMethodOrder(MethodOrderer.OrderAnnotation.class)
public class V2_DeltaShooterTest {
    @Test
    @Order(1)
    public void testZero() {
        Angle angle = V2_DeltaShooter.fieldToTurret(
                new Pose2d(new Translation2d(Meters.of(1), Meters.of(0)), new Rotation2d()),
                new Translation2d(Meters.of(2), Meters.of(0)));
        System.out.println(angle.in(Degrees));
        assertEquals(Degrees.of(0), angle);
    }

    @Test
    @Order(2)
    public void test90() {
        Angle angle = V2_DeltaShooter.fieldToTurret(
                new Pose2d(new Translation2d(Meters.of(1), Meters.of(0)), new Rotation2d()),
                new Translation2d(Meters.of(1), Meters.of(1)));
        System.out.println(angle.in(Degrees));
        assertEquals(Degrees.of(90), angle);
    }

    @Test
    @Order(3)
    public void test180() {
        Angle angle = V2_DeltaShooter.fieldToTurret(
                new Pose2d(new Translation2d(Meters.of(1), Meters.of(0)), new Rotation2d()),
                new Translation2d(Meters.of(0), Meters.of(0)));

        System.out.println(angle.in(Degrees));
        assertEquals(Degrees.of(180), angle);
    }

    @Test
    @Order(4)
    public void test270() {
        Angle angle = V2_DeltaShooter.fieldToTurret(
                new Pose2d(new Translation2d(Meters.of(1), Meters.of(0)), new Rotation2d()),
                new Translation2d(Meters.of(1), Meters.of(-1)));
        System.out.println(angle.in(Degrees));
        assertEquals(Degrees.of(-90), angle);
    }

    @Test
    @Order(5)
    public void test450() {
        Angle angle = V2_DeltaShooter.fieldToTurret(
                new Pose2d(new Translation2d(Meters.of(1), Meters.of(0)), new Rotation2d()),
                new Translation2d(Meters.of(1), Meters.of(-1)));
        System.out.println(angle.in(Degrees));
        assertEquals(Degrees.of(90), angle);
    }
}
