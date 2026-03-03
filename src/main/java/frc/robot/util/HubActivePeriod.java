package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import lombok.Getter;
import lombok.RequiredArgsConstructor;

public class HubActivePeriod {

  @RequiredArgsConstructor
  public enum Shift {
    AUTO(0.0, new ActiveSchedule(true, true)),
    TRANSITION(0.0, new ActiveSchedule(true, true)),
    SHIFT1(10.0, new ActiveSchedule(false, true)),
    SHIFT2(35.0, new ActiveSchedule(false, false)),
    SHIFT3(60.0, new ActiveSchedule(false, true)),
    SHIFT4(85.0, new ActiveSchedule(false, false)),
    ENDGAME(110.0, new ActiveSchedule(true, true)),
    END(140.0, new ActiveSchedule(false, false));

    @Getter private final double startTime;
    private final ActiveSchedule schedule;

    private static class ActiveSchedule {
      private final boolean activeSchedule, inactiveSchedule;

      public ActiveSchedule(boolean alwaysActive, boolean active) {
        if (alwaysActive) {
          activeSchedule = active;
          inactiveSchedule = active;
        } else {
          activeSchedule = active;
          inactiveSchedule = !active;
        }
      }
    }

    public boolean isActive(Alliance alliance) {
      return alliance == getFirstActiveAlliance()
          ? schedule.activeSchedule
          : schedule.inactiveSchedule;
    }

    public double getDuration() {
      if (ordinal() + 1 >= shifts.length) {
        return 0.0;
      }
      return shifts[ordinal() + 1].getStartTime() - startTime;
    }
  }

  private static final Timer shiftTimer = new Timer();
  private static final Shift[] shifts = Shift.values();

  public static Alliance getFirstActiveAlliance() {
    // var alliance = DriverStation.getAlliance().orElse(Alliance.Blue);

    String message = DriverStation.getGameSpecificMessage();
    if (!message.isEmpty()) {
      char character = message.charAt(0);
      if (character == 'R') {
        return Alliance.Blue;
      } else if (character == 'B') {
        return Alliance.Red;
      }
    }

    return Alliance.Blue; // Per Marcus's request
  }

  /** Call at start of teleop */
  public static void initialize() {
    shiftTimer.restart();
  }

  public static Shift getCurrentShift() {
    // Auto always returns AUTO
    if (DriverStation.isAutonomousEnabled()) {
      return Shift.AUTO;
    }

    double time = shiftTimer.get();

    Shift current = Shift.AUTO;

    for (Shift shift : shifts) {
      if (time >= shift.getStartTime()) {
        current = shift;
      } else {
        break;
      }
    }

    return current;
  }

  public static boolean isHubActive() {
    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
    return getCurrentShift().isActive(alliance);
  }

  public static double getShiftTimeRemaining() {
    return getCurrentShift().ordinal() != Shift.values().length - 1 ? (getCurrentShift().getDuration() - (shiftTimer.get() - getCurrentShift().startTime)) : 0.0;
  }
}
