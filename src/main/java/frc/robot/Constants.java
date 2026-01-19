package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.team190.gompeilib.core.robot.RobotMode;

public final class Constants {
  public static final boolean TUNING_MODE = false;
  public static final double LOOP_PERIOD_SECONDS = 0.02;

  public static RobotMode getMode() {
    switch (RobotConfig.ROBOT) {
      case V0_FUNKY:
        return RobotBase.isReal() ? RobotMode.REAL : RobotMode.REPLAY;

      case V0_FUNKY_SIM:
        return RobotMode.SIM;

      default:
        return RobotMode.REAL;
    }
  }

  public static void main(String... args) {
    if (getMode().equals(RobotMode.SIM)) {
      System.err.println("Cannot deploy, invalid mode selected: " + RobotConfig.ROBOT);
      System.exit(1);
    }
  }

  public enum RobotType {
    V0_FUNKY,
    V0_FUNKY_SIM,
  }
}
