package first.robot.shared.util.input;

import org.wpilib.command2.CommandScheduler;
import org.wpilib.command2.button.CommandXboxController;
import org.wpilib.command2.button.Trigger;
import org.wpilib.driverstation.XboxController;
import org.wpilib.event.EventLoop;

public class XboxElite2Input extends CommandXboxController {

  public XboxElite2Input(int port) {
    super(port);
  }

  /**
   * Constructs a Trigger instance around the top left paddle's digital signal.
   *
   * @return a Trigger instance representing the top left paddle's digital signal attached to the
   *     {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
   * @see #topLeftPaddle(EventLoop)
   */
  public Trigger topLeftPaddle() {
    return view(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  /**
   * Constructs a Trigger instance around the top left paddle's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return a Trigger instance representing the top left paddle's digital signal attached to the
   *     given loop.
   */
  public Trigger topLeftPaddle(EventLoop loop) {
    return button(XboxController.Button.VIEW, loop);
  }

  /**
   * Constructs a Trigger instance around the top right paddle's digital signal.
   *
   * @return a Trigger instance representing the top right paddle's digital signal attached to the
   *     {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
   * @see #topRightPaddle(EventLoop)
   */
  public Trigger topRightPaddle() {
    return menu(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  /**
   * Constructs a Trigger instance around the top right paddle's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return a Trigger instance representing the top right paddle's digital signal attached to the
   *     given loop.
   */
  public Trigger topRightPaddle(EventLoop loop) {
    return button(XboxController.Button.MENU, loop);
  }

  /**
   * Constructs a Trigger instance around the bottom left paddle's digital signal.
   *
   * @return a Trigger instance representing the bottom left paddle's digital signal attached to the
   *     {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
   * @see #bottomLeftPaddle(EventLoop)
   */
  public Trigger bottomLeftPaddle() {
    return leftStick(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  /**
   * Constructs a Trigger instance around the bottom left paddle's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return a Trigger instance representing the bottom left paddle's digital signal attached to the
   *     given loop.
   */
  public Trigger bottomLeftPaddle(EventLoop loop) {
    return button(XboxController.Button.LEFT_STICK, loop);
  }

  /**
   * Constructs a Trigger instance around the bottom right paddle's digital signal.
   *
   * @return a Trigger instance representing the bottom right paddle's digital signal attached to
   *     the {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
   * @see #bottomRightPaddle(EventLoop)
   */
  public Trigger bottomRightPaddle() {
    return rightStick(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  /**
   * Constructs a Trigger instance around the bottom right paddle's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return a Trigger instance representing the bottom right paddle's digital signal attached to
   *     the given loop.
   */
  public Trigger bottomRightPaddle(EventLoop loop) {
    return button(XboxController.Button.RIGHT_STICK, loop);
  }
}
