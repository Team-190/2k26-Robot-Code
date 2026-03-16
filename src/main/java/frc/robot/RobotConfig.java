package frc.robot;

import lombok.Getter;

public final class RobotConfig {
  public static final RobotType ROBOT = RobotType.V1_DOOMSPIRAL_SIM;

  public enum RobotType {
    V0_FUNKY("2026.+"),
    V0_FUNKY_SIM("2026.+"),
    V1_DOOMSPIRAL("2026.5.0"),
    V1_DOOMSPIRAL_SIM("2026.5.0");

    @Getter private String gompeiLibVersion;

    private RobotType(String gompeiLibVersion) {
      this.gompeiLibVersion = gompeiLibVersion;
    }
  }
}
