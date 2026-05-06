package frc.robot.subsystems.v2_Turnover.leds;

import static com.ctre.phoenix6.signals.StripTypeValue.GRB;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.AnimationDirectionValue;
import com.ctre.phoenix6.signals.LossOfSignalBehaviorValue;
import com.ctre.phoenix6.signals.RGBWColor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.team190.gompeilib.core.logging.Trace;
import edu.wpi.team190.gompeilib.core.utility.VirtualSubsystem;
import edu.wpi.team190.gompeilib.core.utility.phoenix.PhoenixUtil;
import frc.robot.subsystems.v2_Turnover.V2_TurnoverRobotState;
import java.util.function.BiConsumer;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.junction.Logger;

public class V2_TurnoverCANdle extends VirtualSubsystem {
  private final CANdle leds;
  private final CANdleConfiguration config;

  private final Trigger lowBatteryTrigger;

  private AnimationType lightType;

  private final RainbowAnimation candleAnimation;

  private int loopCycleCount = 0;
  private boolean estopped = false;

  @RequiredArgsConstructor
  private enum AnimationType {
    E_STOPPED(
        (leds, section) ->
            leds.setControl(
                new SolidColor(section.getStart(), section.getEnd())
                    .withColor(new RGBWColor(Color.kWhite)))),
    LOW_BATTERY(
        (leds, section) ->
            leds.setControl(
                new StrobeAnimation(section.getStart(), section.getEnd())
                    .withColor(new RGBWColor(Color.kOrange))
                    .withSlot(section.getSlot())
                    .withFrameRate(120))),
    INTAKE_COLLECTING(
        (leds, section) -> {
          leds.setControl(
              new SolidColor(section.getStart(), section.getEnd())
                  .withColor(new RGBWColor(Color.kAqua)));
        }),
    SHOOTING(
        (leds, section) ->
            leds.setControl(
                new SolidColor(section.getStart(), section.getEnd())
                    .withColor(new RGBWColor(Color.kGreen)))),
    FEEDING(
        (leds, section) ->
            leds.setControl(
                new SolidColor(section.getStart(), section.getEnd())
                    .withColor(new RGBWColor(Color.kYellow)))),
    INTAKE_SLOWER(
        (leds, section) -> {
          leds.setControl(
              new SolidColor(section.getStart(), section.getEnd())
                  .withColor(new RGBWColor(Color.kPurple)));
        }),
    DEFAULT(
        (leds, section) ->
            leds.setControl(
                new SingleFadeAnimation(section.getStart(), section.getEnd())
                    .withColor(new RGBWColor(Color.kRed))
                    .withSlot(section.getSlot())
                    .withFrameRate(2)));

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
        case MAIN, WHOLE -> V2_TurnoverCANdleConstants.LED_COUNT;
      };
    }

    public int getSlot() {
      return switch (this) {
        case MAIN -> 1; //  and up to 7 if required
        case STATUS, WHOLE -> 0;
      };
    }
  }

  public V2_TurnoverCANdle() {
    super();
    leds = new CANdle(V2_TurnoverCANdleConstants.CAN_ID, V2_TurnoverCANdleConstants.CAN_LOOP);
    config = new CANdleConfiguration();
    config.LED.BrightnessScalar = 1.00;
    config.LED.StripType = GRB;
    config.LED.LossOfSignalBehavior = LossOfSignalBehaviorValue.DisableLEDs;
    PhoenixUtil.tryUntilOk(5, () -> leds.getConfigurator().apply(config, 0.25));
    lowBatteryTrigger =
        new Trigger(
                () ->
                    RobotController.getBatteryVoltage()
                        < V2_TurnoverCANdleConstants.LOW_BATTERY_VOLTAGE)
            .debounce(.5);
    candleAnimation =
        new RainbowAnimation(Section.STATUS.getStart(), Section.STATUS.getEnd())
            .withFrameRate(80)
            .withDirection(AnimationDirectionValue.Forward);
    leds.setControl(new EmptyControl());
    for (int i = 0; i < 8; i++) {
      leds.setControl(new EmptyAnimation(i));
    }

    synchronized (this) {
      leds.setControl(
          new SingleFadeAnimation(0, V2_TurnoverCANdleConstants.LED_COUNT)
              .withSlot(1)
              .withColor(new RGBWColor(Color.kWhite)));
    }
    lightType = AnimationType.DEFAULT;
  }

  private void clearSlots(int startSlot, int endSlot) {
    for (int i = startSlot; i <= endSlot; i++) {
      leds.setControl(new EmptyAnimation(i));
    }
  }

  @Override
  @Trace
  public synchronized void periodic() {

    Logger.recordOutput("Leds/Animation Type", lightType);

    if (DriverStation.isEStopped()) {
      if (!estopped) {
        clearSlots(0, 7);
        AnimationType.E_STOPPED.animationSetter.accept(leds, Section.WHOLE);
        estopped = true;
      }

      return;
    }

    if (loopCycleCount <= V2_TurnoverCANdleConstants.MIN_LOOP_CYCLE_COUNT) {
      loopCycleCount++;
      return;
    }

    AnimationType primaryAnimationType = getPrimaryAnimationType();

    if (primaryAnimationType != lightType) {
      clearSlots(1, 7);
      leds.setControl(candleAnimation);
      primaryAnimationType.animationSetter.accept(leds, Section.MAIN);
      lightType = primaryAnimationType;
      leds.setControl(
          new RainbowAnimation(Section.STATUS.getStart(), Section.STATUS.getEnd())
              .withFrameRate(80));
    }
  }

  private AnimationType getPrimaryAnimationType() {
    if (DriverStation.isDisabled()) {

      if (lowBatteryTrigger.getAsBoolean()) {
        return AnimationType.LOW_BATTERY;
      }

      return AnimationType.DEFAULT;
    }

    if (V2_TurnoverRobotState.getLedStates().isIntakeCollecting()) {
      return AnimationType.INTAKE_COLLECTING;
    }

    if (V2_TurnoverRobotState.getLedStates().isShooterShooting()) {
      return AnimationType.SHOOTING;
    }

    if (V2_TurnoverRobotState.getLedStates().isShooterFeeding()) {
      return AnimationType.FEEDING;
    }

    if (V2_TurnoverRobotState.getLedStates().isIntakeSlowRolling()) {
      return AnimationType.INTAKE_SLOWER;
    }

    return AnimationType.DEFAULT;
  }
}
