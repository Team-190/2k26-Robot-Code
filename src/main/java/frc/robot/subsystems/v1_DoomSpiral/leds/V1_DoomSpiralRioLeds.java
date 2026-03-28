package frc.robot.subsystems.v1_DoomSpiral.leds;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.subsystems.shared.leds.Leds;
import frc.robot.subsystems.v1_DoomSpiral.V1_DoomSpiralRobotState;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.junction.Logger;

public class V1_DoomSpiralRioLeds extends Leds {

  private final Trigger lowBatteryTrigger;

  private AnimationType desiredAnimation;

  private int loopCycleCount = 0;

  public V1_DoomSpiralRioLeds() {
    super(42, 9);

    lowBatteryTrigger =
        new Trigger(
                () ->
                    RobotController.getBatteryVoltage()
                        < V1_DoomSpiralCANdleConstants.LOW_BATTERY_VOLTAGE)
            .debounce(0.5);

    desiredAnimation = AnimationType.DEFAULT;
  }

  @RequiredArgsConstructor
  private enum AnimationType {
    E_STOPPED,
    LOW_BATTERY,
    INTAKE_IN,
    INTAKE_COLLECTING,
    PREPPING,
    SHOOTING,
    SPITTING,
    AUTO_CLIMB,
    JITTING,
    DEFAULT
  }

  @Override
  public synchronized void periodic() {

    if (loopCycleCount <= MIN_LOOP_CYCLE_COUNT) {
      loopCycleCount++;
      return;
    } else {
      loadingNotifier.stop();
    }

    if (DriverStation.isEStopped()) {
      runAnimation(AnimationType.E_STOPPED);
      leds.setData(buffer);
      return;
    }

    runAnimation(getPrimaryAnimationType());
    leds.setData(buffer);

    Logger.recordOutput("Leds/Animation Type", desiredAnimation);
  }

  private void runAnimation(AnimationType type) {

    switch (type) {
      case E_STOPPED:
        solid(Color.kRed);
        break;

      case LOW_BATTERY:
        strobe(Color.kOrange, Color.kBlack, STROBE_FAST_DURATION);
        break;

      case INTAKE_IN:
        solid(Color.kDarkOrange);
        break;

      case INTAKE_COLLECTING:
        // inward wave
        wave(Color.kAqua, Color.kBlack, WAVE_FAST_CYCLE_LENGTH, WAVE_FAST_DURATION);
        break;

      case PREPPING:
        strobe(Color.kGreen, Color.kBlack, STROBE_SLOW_DURATION);
        break;

      case SHOOTING:
        Color allianceColor =
            DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
                    == DriverStation.Alliance.Red
                ? Color.kRed
                : Color.kBlue;

        strobe(allianceColor, Color.kBlack, STROBE_FAST_DURATION);
        break;

      case SPITTING:
        wave(Color.kPurple, Color.kBlack, WAVE_FAST_CYCLE_LENGTH, WAVE_FAST_DURATION);
        break;

      case AUTO_CLIMB:
        rainbow(RAINBOW_CYCLE_LENGTH, RAINBOW_DURATION);
        break;

      case JITTING:
        breath(
            new Color(204 / 255.0, 1.0, 0),
            Color.kBlack,
            edu.wpi.first.wpilibj.Timer.getFPGATimestamp());
        break;

      case DEFAULT:
      default:
        breath(Color.kGreen, Color.kBlack, edu.wpi.first.wpilibj.Timer.getFPGATimestamp());
        break;
    }

    desiredAnimation = type;
  }

  private AnimationType getPrimaryAnimationType() {

    if (DriverStation.isDisabled()) {

      if (lowBatteryTrigger.getAsBoolean()) {
        return AnimationType.LOW_BATTERY;
      }

      if (Robot.isJitting()) {
        return AnimationType.JITTING;
      }

      return AnimationType.DEFAULT;
    }

    if (DriverStation.isAutonomous() && V1_DoomSpiralRobotState.getLedStates().isAutoClimbing()) {
      return AnimationType.AUTO_CLIMB;
    }

    if (V1_DoomSpiralRobotState.getLedStates().isShooterShooting()) {
      return AnimationType.SHOOTING;
    }

    if (V1_DoomSpiralRobotState.getLedStates().isShooterPrepping()) {
      return AnimationType.PREPPING;
    }

    if (V1_DoomSpiralRobotState.getLedStates().isIntakeCollecting()) {
      return AnimationType.INTAKE_COLLECTING;
    }

    if (V1_DoomSpiralRobotState.getLedStates().isSpitting()) {
      return AnimationType.SPITTING;
    }

    if (V1_DoomSpiralRobotState.getLedStates().isIntakeIn()) {
      return AnimationType.INTAKE_IN;
    }

    if (Robot.isJitting()) {
      return AnimationType.JITTING;
    }

    return AnimationType.DEFAULT;
  }
}
