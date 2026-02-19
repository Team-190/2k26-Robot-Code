package frc.robot.subsystems.v1_DoomSpiral.leds;

import static com.ctre.phoenix6.signals.StripTypeValue.RGBW;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.AnimationDirectionValue;
import com.ctre.phoenix6.signals.RGBWColor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.team190.gompeilib.core.GompeiLib;
import edu.wpi.team190.gompeilib.core.utility.PhoenixUtil;
import edu.wpi.team190.gompeilib.core.utility.VirtualSubsystem;
import frc.robot.Robot;
import frc.robot.subsystems.v1_DoomSpiral.V1_DoomSpiralRobotState;
import java.util.function.BiConsumer;
import lombok.RequiredArgsConstructor;

public class V1_DoomSpiralCANdle extends VirtualSubsystem {
  private final CANdle leds;
  private final CANdleConfiguration config;

  private final Notifier loadingNotifier;

  private AnimationType statusLights;
  private AnimationType mainLights;

  private int loopCycleCount = 0;
  private boolean estopped = false;

  @RequiredArgsConstructor
  private enum AnimationType {
    E_STOPPED(
        (leds, section) ->
            leds.setControl(
                new SolidColor(section.getStart(), section.getEnd())
                    .withColor(new RGBWColor(Color.kRed)))),
    LOW_BATTERY(
        (leds, section) ->
            leds.setControl(
                new StrobeAnimation(section.getStart(), section.getEnd())
                    .withColor(new RGBWColor(Color.kOrange))
                    .withSlot(section.getSlot())
                    .withFrameRate(120))),
    INTAKE_IN(
        (leds, section) ->
            leds.setControl(
                new SolidColor(section.getStart(), section.getEnd())
                    .withColor(new RGBWColor(Color.kDarkOrange)))),
    INTAKE_COLLECTING(
        (leds, section) -> {
          leds.setControl(
              new ColorFlowAnimation(
                      section.getEnd(),
                      (section.getSlot() + (section.getEnd() - section.getSlot()) / 2) + 1)
                  .withSlot(section.getSlot())
                  .withColor(new RGBWColor(Color.kAqua))
                  .withDirection(AnimationDirectionValue.Forward)
                  .withFrameRate(50));

          leds.setControl(
              new ColorFlowAnimation(
                      section.getSlot() + (section.getEnd() - section.getSlot()) / 2,
                      section.getStart() + 1)
                  .withSlot(section.getSlot())
                  .withColor(new RGBWColor(Color.kAqua))
                  .withDirection(AnimationDirectionValue.Forward)
                  .withFrameRate(50));
        }),

    PREPPING(
        (leds, section) ->
            leds.setControl(
                new StrobeAnimation(section.getStart(), section.getEnd())
                    .withSlot(section.getSlot())
                    .withColor(new RGBWColor(Color.kGreen))
                    .withFrameRate(70))),
    SHOOTING(
        (leds, section) ->
            leds.setControl(
                new StrobeAnimation(section.getStart(), section.getEnd())
                    .withSlot(section.getSlot())
                    .withColor(new RGBWColor(Color.kYellowGreen))
                    .withFrameRate(70))),
    SPITTING(
        (leds, section) -> {
          leds.setControl(
              new ColorFlowAnimation(
                      section.getEnd(),
                      (section.getSlot() + (section.getEnd() - section.getSlot()) / 2) + 1)
                  .withSlot(section.getSlot())
                  .withColor(new RGBWColor(Color.kPurple))
                  .withDirection(AnimationDirectionValue.Backward)
                  .withFrameRate(50));

          leds.setControl(
              new ColorFlowAnimation(
                      section.getSlot() + (section.getEnd() - section.getSlot()) / 2,
                      section.getStart() + 1)
                  .withSlot(section.getSlot() + 1)
                  .withColor(new RGBWColor(Color.kPurple))
                  .withDirection(AnimationDirectionValue.Backward)
                  .withFrameRate(50));
        }),
    AUTO_CLIMB(
        (leds, section) ->
            leds.setControl(
                new RainbowAnimation(section.getStart(), section.getEnd())
                    .withSlot(section.getSlot())
                    .withFrameRate(150)
                    .withDirection(AnimationDirectionValue.Forward))),
    JITTING(
        (leds, section) ->
            leds.setControl(
                new SingleFadeAnimation(section.getStart(), section.getEnd())
                    .withColor(new RGBWColor(204, 255, 0))
                    .withSlot(section.getSlot())
                    .withFrameRate(30))),
    DEFAULT(
        (leds, section) ->
            leds.setControl(
                new SingleFadeAnimation(section.getStart(), section.getEnd())
                    .withColor(new RGBWColor(Color.kGreen))
                    .withSlot(section.getSlot())
                    .withFrameRate(30)));

    public final BiConsumer<CANdle, Section> animationSetter;
  }

  private enum Section {
    MAIN,
    STATUS,
    WHOLE;

    public int getStart() {
      return switch (this) {
        case STATUS, WHOLE -> 0;
        case MAIN -> 8;
      };
    }

    public int getEnd() {
      return switch (this) {
        case STATUS -> 7;
        case MAIN, WHOLE -> V1_DoomSpiralCANdleConstants.LED_COUNT;
      };
    }

    public int getSlot() {
      return switch (this) {
        case MAIN -> 1; //  and up to 7 if required
        case STATUS, WHOLE -> 0;
      };
    }
  }

  public V1_DoomSpiralCANdle() {
    super();
    leds = new CANdle(V1_DoomSpiralCANdleConstants.CAN_ID, V1_DoomSpiralCANdleConstants.CAN_LOOP);
    config = new CANdleConfiguration();
    config.LED.BrightnessScalar = 1.00;
    config.LED.StripType = RGBW;
    PhoenixUtil.tryUntilOk(5, () -> leds.getConfigurator().apply(config, 0.25));
    leds.setControl(new EmptyControl());
    for (int i = 0; i < 8; i++) {
      leds.setControl(new EmptyAnimation(i));
    }

    loadingNotifier =
        new Notifier(
            () -> {
              synchronized (this) {
                leds.setControl(
                    new SingleFadeAnimation(0, V1_DoomSpiralCANdleConstants.LED_COUNT - 1)
                        .withSlot(7)
                        .withColor(new RGBWColor(Color.kRed)));
              }
            });
    loadingNotifier.startPeriodic(GompeiLib.getLoopPeriod());
    statusLights = AnimationType.JITTING;
    mainLights = AnimationType.DEFAULT;
  }

  private void clearSlots(int startSlot, int endSlot) {
    for (int i = startSlot; i <= endSlot; i++) {
      leds.setControl(new EmptyAnimation(i));
    }
  }

  @Override
  public synchronized void periodic() {

    if (DriverStation.isEStopped()) {
      loadingNotifier.stop();

      if (!estopped) {
        clearSlots(0, 7);
        AnimationType.E_STOPPED.animationSetter.accept(leds, Section.WHOLE);
        estopped = true;
      }

      return;
    }

    if (loopCycleCount > V1_DoomSpiralCANdleConstants.MIN_LOOP_CYCLE_COUNT) {
      loadingNotifier.stop();
    } else {
      loopCycleCount++;
      return;
    }

    V1_DoomSpiralRobotState.getLedStates()
        .setLowBattery(
            RobotController.getBatteryVoltage() < V1_DoomSpiralCANdleConstants.LOW_BATTERY_VOLTAGE);

    AnimationType primaryAnimationType = getPrimaryAnimationType();
    AnimationType secondaryAnimationType = getSecondaryAnimationType();

    if (primaryAnimationType != mainLights) {
      clearSlots(1, 7);
      primaryAnimationType.animationSetter.accept(leds, Section.MAIN);
      mainLights = primaryAnimationType;
    }

    if (secondaryAnimationType != statusLights) {
      secondaryAnimationType.animationSetter.accept(leds, Section.STATUS);
      statusLights = secondaryAnimationType;
    }
  }

  private AnimationType getPrimaryAnimationType() {
    if (DriverStation.isDisabled()) {
      return getSecondaryAnimationType();
    }
    if (DriverStation.isAutonomous() && V1_DoomSpiralRobotState.getLedStates().isAutoClimbing()) {
      return AnimationType.AUTO_CLIMB;
    }

    if (V1_DoomSpiralRobotState.getLedStates().isIntakeIn()) {
      return AnimationType.INTAKE_IN;
    }

    if (V1_DoomSpiralRobotState.getLedStates().isIntakeCollecting()) {
      return AnimationType.INTAKE_COLLECTING;
    }

    if (V1_DoomSpiralRobotState.getLedStates().isShooterPrepping()) {
      return AnimationType.PREPPING;
    }

    if (V1_DoomSpiralRobotState.getLedStates().isShooterShooting()) {
      return AnimationType.SHOOTING;
    }

    if (V1_DoomSpiralRobotState.getLedStates().isSpitting()) {
      return AnimationType.SPITTING;
    }

    if (Robot.isJitting()) {
      return AnimationType.JITTING;
    }

    return AnimationType.DEFAULT;
  }

  private AnimationType getSecondaryAnimationType() {

    if (V1_DoomSpiralRobotState.getLedStates().isLowBattery()) {
      return AnimationType.LOW_BATTERY;
    }

    if (Robot.isJitting()) {
      return AnimationType.JITTING;
    }
    return AnimationType.DEFAULT;
  }
}
