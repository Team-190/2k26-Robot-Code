package edu.wpi.team190.gompeilib.core.utility.control;

import edu.wpi.team190.gompeilib.core.utility.tunable.LoggedTunableNumber;
import java.util.function.Consumer;
import lombok.Builder;
import lombok.NonNull;

@Builder(setterPrefix = "with")
public record Gains(
    LoggedTunableNumber kP,
    LoggedTunableNumber kI,
    LoggedTunableNumber kD,
    LoggedTunableNumber kS,
    LoggedTunableNumber kV,
    LoggedTunableNumber kA,
    LoggedTunableNumber kG) {

  @Builder(
      setterPrefix = "with",
      builderMethodName = "fromDoubles",
      builderClassName = "FromDoubles")
  public Gains(
      @NonNull String prefix,
      double kP,
      double kI,
      double kD,
      double kS,
      double kV,
      double kA,
      double kG) {
    this(
        new LoggedTunableNumber(prefix + "/Kp", kP),
        new LoggedTunableNumber(prefix + "/Ki", kI),
        new LoggedTunableNumber(prefix + "/Kd", kD),
        new LoggedTunableNumber(prefix + "/Ks", kS),
        new LoggedTunableNumber(prefix + "/Kv", kV),
        new LoggedTunableNumber(prefix + "/Ka", kA),
        new LoggedTunableNumber(prefix + "/Kg", kG));
  }

  public Gains {
    if (kP == null) {
      kP = new LoggedTunableNumber("");
    }
    if (kI == null) {
      kI = new LoggedTunableNumber("");
    }
    if (kD == null) {
      kD = new LoggedTunableNumber("");
    }
    if (kS == null) {
      kS = new LoggedTunableNumber("");
    }
    if (kV == null) {
      kV = new LoggedTunableNumber("");
    }
    if (kA == null) {
      kA = new LoggedTunableNumber("");
    }
    if (kG == null) {
      kG = new LoggedTunableNumber("");
    }
  }

  public double getKP() {
    return kP.get();
  }

  public double getKI() {
    return kI.get();
  }

  public double getKD() {
    return kD.get();
  }

  public double getKS() {
    return kS.get();
  }

  public double getKV() {
    return kV.get();
  }

  public double getKA() {
    return kA.get();
  }

  public double getKG() {
    return kG.get();
  }

  public void update(int id, Consumer<Gains> consumer) {
    LoggedTunableNumber.ifChanged(id, g -> consumer.accept(this), kP, kI, kD, kS, kV, kA, kG);
  }
}
