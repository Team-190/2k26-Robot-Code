package edu.wpi.team190.gompeilib.core.utility.control;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.geometry.Twist2d;
import org.junit.jupiter.api.Test;

public class LinearProfileTest {
  @Test
  public void testLinearProfile() {
    LinearProfile profile = new LinearProfile(10.0, 100.0, 0.02);
    assertEquals(0.02, profile.getPeriod());

    // Set max velocity and acceleration
    profile.setMaxVelocity(50.0);
    profile.setMaxAcceleration(20.0); // dv = 20.0 * 0.02 = 0.4

    profile.setGoal(10.0, 5.0);
    assertEquals(10.0, profile.getGoal());
    assertEquals(5.0, profile.getCurrentSetpoint());

    // calculateSetpoint towards positive goal
    double setpoint = profile.calculateSetpoint();
    assertEquals(5.4, setpoint);

    // reach goal
    profile.setGoal(5.5, 5.4);
    assertEquals(5.5, profile.calculateSetpoint());

    // epsilon equal setpoint
    assertEquals(5.5, profile.calculateSetpoint());

    // towards negative goal
    profile.setGoal(4.0, 5.0);
    assertEquals(4.6, profile.calculateSetpoint());

    // reach negative goal
    profile.setGoal(4.5, 4.6);
    assertEquals(4.5, profile.calculateSetpoint());

    // reset
    profile.reset();
    assertEquals(0.0, profile.getGoal());
    assertEquals(0.0, profile.getCurrentSetpoint());

    // test inner EqualsUtil
    profile.new EqualsUtil(); // coverage
    assertTrue(LinearProfile.EqualsUtil.epsilonEquals(1.0, 1.0000000001));
    assertFalse(LinearProfile.EqualsUtil.epsilonEquals(1.0, 1.1));

    // test inner GeomExtensions
    new LinearProfile.EqualsUtil.GeomExtensions(); // coverage
    Twist2d t1 = new Twist2d(1.0, 2.0, 3.0);
    Twist2d t2 = new Twist2d(1.0, 2.0, 3.0);
    Twist2d t3 = new Twist2d(1.1, 2.0, 3.0);
    Twist2d t4 = new Twist2d(1.0, 2.1, 3.0);
    Twist2d t5 = new Twist2d(1.0, 2.0, 3.1);
    assertTrue(LinearProfile.EqualsUtil.GeomExtensions.epsilonEquals(t1, t2));
    assertFalse(LinearProfile.EqualsUtil.GeomExtensions.epsilonEquals(t1, t3)); // dx diff
    assertFalse(LinearProfile.EqualsUtil.GeomExtensions.epsilonEquals(t1, t4)); // dy diff
    assertFalse(LinearProfile.EqualsUtil.GeomExtensions.epsilonEquals(t1, t5)); // dtheta diff

    // test calculateSetpoint with NaN goal (covers else branch of goal < currentSetpoint)
    profile.setGoal(Double.NaN, 5.0);
    assertEquals(5.0, profile.calculateSetpoint());
  }
}
