import lombok.Builder;
import lombok.Getter;

public class Wheel {
  @Getter private double radius;
  @Getter private double momentOfInertia;
  @Getter private double height;

  @Builder
  public Wheel(double radius, double momentOfInertia, double height) {
    this.radius = radius;
    this.momentOfInertia = momentOfInertia;
    this.height = height;
  }

  public double apply(double angularVelocity) {
    return angularVelocity * radius;
  }
}
