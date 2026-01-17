package frc.robot.util;

import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

@SuppressWarnings("MethodName")
public class XKeysInput extends CommandGenericHID {
  /**
   * Construct an instance of a controller.
   *
   * @param port The port index on the Driver Station that the controller is plugged into.
   */
  public XKeysInput(int port) {
    super(port);
  }

  // Rows are lettered a-h from top to bottom
  public enum Rows {
    ka(11),
    kb(12),
    kc(13),
    kd(14),
    ke(15),
    kf(16),
    kg(17),
    kh(18);
    public final int value;

    Rows(int value) {
      this.value = value;
    }
  }

  // Columns are numbered 1-10 from left to right
  public enum Cols {
    k1(1),
    k2(2),
    k3(3),
    k4(4),
    k5(5),
    k6(6),
    k7(7),
    k8(8),
    k9(9),
    k10(10);
    public final int value;

    Cols(int value) {
      this.value = value;
    }
  }

  public Trigger a1() {
    EventLoop loop = CommandScheduler.getInstance().getDefaultButtonLoop();
    return button(Rows.ka.value, loop).and(button(Cols.k1.value, loop));
  }

  public Trigger a2() {
    EventLoop loop = CommandScheduler.getInstance().getDefaultButtonLoop();
    return button(Rows.ka.value, loop).and(button(Cols.k2.value, loop));
  }

  public Trigger a3() {
    EventLoop loop = CommandScheduler.getInstance().getDefaultButtonLoop();
    return button(Rows.ka.value, loop).and(button(Cols.k3.value, loop));
  }

  public Trigger a4() {
    EventLoop loop = CommandScheduler.getInstance().getDefaultButtonLoop();
    return button(Rows.ka.value, loop).and(button(Cols.k4.value, loop));
  }

  public Trigger a5() {
    EventLoop loop = CommandScheduler.getInstance().getDefaultButtonLoop();
    return button(Rows.ka.value, loop).and(button(Cols.k5.value, loop));
  }

  public Trigger a6() {
    EventLoop loop = CommandScheduler.getInstance().getDefaultButtonLoop();
    return button(Rows.ka.value, loop).and(button(Cols.k6.value, loop));
  }

  public Trigger a7() {
    EventLoop loop = CommandScheduler.getInstance().getDefaultButtonLoop();
    return button(Rows.ka.value, loop).and(button(Cols.k7.value, loop));
  }

  public Trigger a8() {
    EventLoop loop = CommandScheduler.getInstance().getDefaultButtonLoop();
    return button(Rows.ka.value, loop).and(button(Cols.k8.value, loop));
  }

  public Trigger a9() {
    EventLoop loop = CommandScheduler.getInstance().getDefaultButtonLoop();
    return button(Rows.ka.value, loop).and(button(Cols.k9.value, loop));
  }

  public Trigger a10() {
    EventLoop loop = CommandScheduler.getInstance().getDefaultButtonLoop();
    return button(Rows.ka.value, loop).and(button(Cols.k10.value, loop));
  }

  public Trigger b1() {
    EventLoop loop = CommandScheduler.getInstance().getDefaultButtonLoop();
    return button(Rows.kb.value, loop).and(button(Cols.k1.value, loop));
  }

  public Trigger b2() {
    EventLoop loop = CommandScheduler.getInstance().getDefaultButtonLoop();
    return button(Rows.kb.value, loop).and(button(Cols.k2.value, loop));
  }

  public Trigger b3() {
    EventLoop loop = CommandScheduler.getInstance().getDefaultButtonLoop();
    return button(Rows.kb.value, loop).and(button(Cols.k3.value, loop));
  }

  public Trigger b4() {
    EventLoop loop = CommandScheduler.getInstance().getDefaultButtonLoop();
    return button(Rows.kb.value, loop).and(button(Cols.k4.value, loop));
  }

  public Trigger b5() {
    EventLoop loop = CommandScheduler.getInstance().getDefaultButtonLoop();
    return button(Rows.kb.value, loop).and(button(Cols.k5.value, loop));
  }

  public Trigger b6() {
    EventLoop loop = CommandScheduler.getInstance().getDefaultButtonLoop();
    return button(Rows.kb.value, loop).and(button(Cols.k6.value, loop));
  }

  public Trigger b7() {
    EventLoop loop = CommandScheduler.getInstance().getDefaultButtonLoop();
    return button(Rows.kb.value, loop).and(button(Cols.k7.value, loop));
  }

  public Trigger b8() {
    EventLoop loop = CommandScheduler.getInstance().getDefaultButtonLoop();
    return button(Rows.kb.value, loop).and(button(Cols.k8.value, loop));
  }

  public Trigger b9() {
    EventLoop loop = CommandScheduler.getInstance().getDefaultButtonLoop();
    return button(Rows.kb.value, loop).and(button(Cols.k9.value, loop));
  }

  public Trigger b10() {
    EventLoop loop = CommandScheduler.getInstance().getDefaultButtonLoop();
    return button(Rows.kb.value, loop).and(button(Cols.k10.value, loop));
  }

  public Trigger c1() {
    EventLoop loop = CommandScheduler.getInstance().getDefaultButtonLoop();
    return button(Rows.kc.value, loop).and(button(Cols.k1.value, loop));
  }

  public Trigger c2() {
    EventLoop loop = CommandScheduler.getInstance().getDefaultButtonLoop();
    return button(Rows.kc.value, loop).and(button(Cols.k2.value, loop));
  }

  public Trigger c3() {
    EventLoop loop = CommandScheduler.getInstance().getDefaultButtonLoop();
    return button(Rows.kc.value, loop).and(button(Cols.k3.value, loop));
  }

  public Trigger c4() {
    EventLoop loop = CommandScheduler.getInstance().getDefaultButtonLoop();
    return button(Rows.kc.value, loop).and(button(Cols.k4.value, loop));
  }

  public Trigger c5() {
    EventLoop loop = CommandScheduler.getInstance().getDefaultButtonLoop();
    return button(Rows.kc.value, loop).and(button(Cols.k5.value, loop));
  }

  public Trigger c6() {
    EventLoop loop = CommandScheduler.getInstance().getDefaultButtonLoop();
    return button(Rows.kc.value, loop).and(button(Cols.k6.value, loop));
  }

  public Trigger c7() {
    EventLoop loop = CommandScheduler.getInstance().getDefaultButtonLoop();
    return button(Rows.kc.value, loop).and(button(Cols.k7.value, loop));
  }

  public Trigger c8() {
    EventLoop loop = CommandScheduler.getInstance().getDefaultButtonLoop();
    return button(Rows.kc.value, loop).and(button(Cols.k8.value, loop));
  }

  public Trigger c9() {
    EventLoop loop = CommandScheduler.getInstance().getDefaultButtonLoop();
    return button(Rows.kc.value, loop).and(button(Cols.k9.value, loop));
  }

  public Trigger c10() {
    EventLoop loop = CommandScheduler.getInstance().getDefaultButtonLoop();
    return button(Rows.kc.value, loop).and(button(Cols.k10.value, loop));
  }

  public Trigger d1() {
    EventLoop loop = CommandScheduler.getInstance().getDefaultButtonLoop();
    return button(Rows.kd.value, loop).and(button(Cols.k1.value, loop));
  }

  public Trigger d2() {
    EventLoop loop = CommandScheduler.getInstance().getDefaultButtonLoop();
    return button(Rows.kd.value, loop).and(button(Cols.k2.value, loop));
  }

  public Trigger d3() {
    EventLoop loop = CommandScheduler.getInstance().getDefaultButtonLoop();
    return button(Rows.kd.value, loop).and(button(Cols.k3.value, loop));
  }

  public Trigger d4() {
    EventLoop loop = CommandScheduler.getInstance().getDefaultButtonLoop();
    return button(Rows.kd.value, loop).and(button(Cols.k4.value, loop));
  }

  public Trigger d5() {
    EventLoop loop = CommandScheduler.getInstance().getDefaultButtonLoop();
    return button(Rows.kd.value, loop).and(button(Cols.k5.value, loop));
  }

  public Trigger d6() {
    EventLoop loop = CommandScheduler.getInstance().getDefaultButtonLoop();
    return button(Rows.kd.value, loop).and(button(Cols.k6.value, loop));
  }

  public Trigger d7() {
    EventLoop loop = CommandScheduler.getInstance().getDefaultButtonLoop();
    return button(Rows.kd.value, loop).and(button(Cols.k7.value, loop));
  }

  public Trigger d8() {
    EventLoop loop = CommandScheduler.getInstance().getDefaultButtonLoop();
    return button(Rows.kd.value, loop).and(button(Cols.k8.value, loop));
  }

  public Trigger d9() {
    EventLoop loop = CommandScheduler.getInstance().getDefaultButtonLoop();
    return button(Rows.kd.value, loop).and(button(Cols.k9.value, loop));
  }

  public Trigger d10() {
    EventLoop loop = CommandScheduler.getInstance().getDefaultButtonLoop();
    return button(Rows.kd.value, loop).and(button(Cols.k10.value, loop));
  }

  public Trigger e1() {
    EventLoop loop = CommandScheduler.getInstance().getDefaultButtonLoop();
    return button(Rows.ke.value, loop).and(button(Cols.k1.value, loop));
  }

  public Trigger e2() {
    EventLoop loop = CommandScheduler.getInstance().getDefaultButtonLoop();
    return button(Rows.ke.value, loop).and(button(Cols.k2.value, loop));
  }

  public Trigger e3() {
    EventLoop loop = CommandScheduler.getInstance().getDefaultButtonLoop();
    return button(Rows.ke.value, loop).and(button(Cols.k3.value, loop));
  }

  public Trigger e4() {
    EventLoop loop = CommandScheduler.getInstance().getDefaultButtonLoop();
    return button(Rows.ke.value, loop).and(button(Cols.k4.value, loop));
  }

  public Trigger e5() {
    EventLoop loop = CommandScheduler.getInstance().getDefaultButtonLoop();
    return button(Rows.ke.value, loop).and(button(Cols.k5.value, loop));
  }

  public Trigger e6() {
    EventLoop loop = CommandScheduler.getInstance().getDefaultButtonLoop();
    return button(Rows.ke.value, loop).and(button(Cols.k6.value, loop));
  }

  public Trigger e7() {
    EventLoop loop = CommandScheduler.getInstance().getDefaultButtonLoop();
    return button(Rows.ke.value, loop).and(button(Cols.k7.value, loop));
  }

  public Trigger e8() {
    EventLoop loop = CommandScheduler.getInstance().getDefaultButtonLoop();
    return button(Rows.ke.value, loop).and(button(Cols.k8.value, loop));
  }

  public Trigger e9() {
    EventLoop loop = CommandScheduler.getInstance().getDefaultButtonLoop();
    return button(Rows.ke.value, loop).and(button(Cols.k9.value, loop));
  }

  public Trigger e10() {
    EventLoop loop = CommandScheduler.getInstance().getDefaultButtonLoop();
    return button(Rows.ke.value, loop).and(button(Cols.k10.value, loop));
  }

  public Trigger f1() {
    EventLoop loop = CommandScheduler.getInstance().getDefaultButtonLoop();
    return button(Rows.kf.value, loop).and(button(Cols.k1.value, loop));
  }

  public Trigger f2() {
    EventLoop loop = CommandScheduler.getInstance().getDefaultButtonLoop();
    return button(Rows.kf.value, loop).and(button(Cols.k2.value, loop));
  }

  public Trigger f3() {
    EventLoop loop = CommandScheduler.getInstance().getDefaultButtonLoop();
    return button(Rows.kf.value, loop).and(button(Cols.k3.value, loop));
  }

  public Trigger f4() {
    EventLoop loop = CommandScheduler.getInstance().getDefaultButtonLoop();
    return button(Rows.kf.value, loop).and(button(Cols.k4.value, loop));
  }

  public Trigger f5() {
    EventLoop loop = CommandScheduler.getInstance().getDefaultButtonLoop();
    return button(Rows.kf.value, loop).and(button(Cols.k5.value, loop));
  }

  public Trigger f6() {
    EventLoop loop = CommandScheduler.getInstance().getDefaultButtonLoop();
    return button(Rows.kf.value, loop).and(button(Cols.k6.value, loop));
  }

  public Trigger f7() {
    EventLoop loop = CommandScheduler.getInstance().getDefaultButtonLoop();
    return button(Rows.kf.value, loop).and(button(Cols.k7.value, loop));
  }

  public Trigger f8() {
    EventLoop loop = CommandScheduler.getInstance().getDefaultButtonLoop();
    return button(Rows.kf.value, loop).and(button(Cols.k8.value, loop));
  }

  public Trigger f9() {
    EventLoop loop = CommandScheduler.getInstance().getDefaultButtonLoop();
    return button(Rows.kf.value, loop).and(button(Cols.k9.value, loop));
  }

  public Trigger f10() {
    EventLoop loop = CommandScheduler.getInstance().getDefaultButtonLoop();
    return button(Rows.kf.value, loop).and(button(Cols.k10.value, loop));
  }

  public Trigger g1() {
    EventLoop loop = CommandScheduler.getInstance().getDefaultButtonLoop();
    return button(Rows.kg.value, loop).and(button(Cols.k1.value, loop));
  }

  public Trigger g2() {
    EventLoop loop = CommandScheduler.getInstance().getDefaultButtonLoop();
    return button(Rows.kg.value, loop).and(button(Cols.k2.value, loop));
  }

  public Trigger g3() {
    EventLoop loop = CommandScheduler.getInstance().getDefaultButtonLoop();
    return button(Rows.kg.value, loop).and(button(Cols.k3.value, loop));
  }

  public Trigger g4() {
    EventLoop loop = CommandScheduler.getInstance().getDefaultButtonLoop();
    return button(Rows.kg.value, loop).and(button(Cols.k4.value, loop));
  }

  public Trigger g5() {
    EventLoop loop = CommandScheduler.getInstance().getDefaultButtonLoop();
    return button(Rows.kg.value, loop).and(button(Cols.k5.value, loop));
  }

  public Trigger g6() {
    EventLoop loop = CommandScheduler.getInstance().getDefaultButtonLoop();
    return button(Rows.kg.value, loop).and(button(Cols.k6.value, loop));
  }

  public Trigger g7() {
    EventLoop loop = CommandScheduler.getInstance().getDefaultButtonLoop();
    return button(Rows.kg.value, loop).and(button(Cols.k7.value, loop));
  }

  public Trigger g8() {
    EventLoop loop = CommandScheduler.getInstance().getDefaultButtonLoop();
    return button(Rows.kg.value, loop).and(button(Cols.k8.value, loop));
  }

  public Trigger g9() {
    EventLoop loop = CommandScheduler.getInstance().getDefaultButtonLoop();
    return button(Rows.kg.value, loop).and(button(Cols.k9.value, loop));
  }

  public Trigger g10() {
    EventLoop loop = CommandScheduler.getInstance().getDefaultButtonLoop();
    return button(Rows.kg.value, loop).and(button(Cols.k10.value, loop));
  }

  public Trigger h1() {
    EventLoop loop = CommandScheduler.getInstance().getDefaultButtonLoop();
    return button(Rows.kh.value, loop).and(button(Cols.k1.value, loop));
  }

  public Trigger h2() {
    EventLoop loop = CommandScheduler.getInstance().getDefaultButtonLoop();
    return button(Rows.kh.value, loop).and(button(Cols.k2.value, loop));
  }

  public Trigger h3() {
    EventLoop loop = CommandScheduler.getInstance().getDefaultButtonLoop();
    return button(Rows.kh.value, loop).and(button(Cols.k3.value, loop));
  }

  public Trigger h4() {
    EventLoop loop = CommandScheduler.getInstance().getDefaultButtonLoop();
    return button(Rows.kh.value, loop).and(button(Cols.k4.value, loop));
  }

  public Trigger h5() {
    EventLoop loop = CommandScheduler.getInstance().getDefaultButtonLoop();
    return button(Rows.kh.value, loop).and(button(Cols.k5.value, loop));
  }

  public Trigger h6() {
    EventLoop loop = CommandScheduler.getInstance().getDefaultButtonLoop();
    return button(Rows.kh.value, loop).and(button(Cols.k6.value, loop));
  }

  public Trigger h7() {
    EventLoop loop = CommandScheduler.getInstance().getDefaultButtonLoop();
    return button(Rows.kh.value, loop).and(button(Cols.k7.value, loop));
  }

  public Trigger h8() {
    EventLoop loop = CommandScheduler.getInstance().getDefaultButtonLoop();
    return button(Rows.kh.value, loop).and(button(Cols.k8.value, loop));
  }

  public Trigger h9() {
    EventLoop loop = CommandScheduler.getInstance().getDefaultButtonLoop();
    return button(Rows.kh.value, loop).and(button(Cols.k9.value, loop));
  }

  public Trigger h10() {
    EventLoop loop = CommandScheduler.getInstance().getDefaultButtonLoop();
    return button(Rows.kh.value, loop).and(button(Cols.k10.value, loop));
  }
}
