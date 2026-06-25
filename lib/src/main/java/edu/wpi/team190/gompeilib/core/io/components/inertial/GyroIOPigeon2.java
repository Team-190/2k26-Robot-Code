package edu.wpi.team190.gompeilib.core.io.components.inertial;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.team190.gompeilib.core.GompeiLib;
import edu.wpi.team190.gompeilib.core.logging.Trace;
import edu.wpi.team190.gompeilib.core.utility.phoenix.PhoenixUtil;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveDriveConstants;
import java.util.Queue;
import java.util.function.Consumer;
import lombok.Getter;

/** IO implementation for Pigeon 2. */
public class GyroIOPigeon2 implements GyroIO {
  @Getter private final StatusSignal<Angle> yaw;
  private final StatusSignal<AngularVelocity> yawVelocity;

  @Getter private final StatusSignal<Angle> pitch;
  private final StatusSignal<AngularVelocity> pitchVelocity;

  @Getter private final StatusSignal<Angle> roll;
  private final StatusSignal<AngularVelocity> rollVelocity;

  private final Consumer<Long> networktablesTimestampConsumer;

  public GyroIOPigeon2(
      SwerveDriveConstants driveConstants, Consumer<Long> networkTablesTimestampConsumer) {
    Pigeon2 pigeon =
        new Pigeon2(driveConstants.driveConfig.pigeon2Id(), driveConstants.driveConfig.canBus());
    pigeon.getConfigurator().apply(new Pigeon2Configuration());
    pigeon.getConfigurator().setYaw(0.0);

    yaw = pigeon.getYaw();
    yawVelocity = pigeon.getAngularVelocityZWorld();

    yaw.setUpdateFrequency(driveConstants.odometryFrequency);

    pitch = pigeon.getPitch();
    pitchVelocity = pigeon.getAngularVelocityXWorld();

    roll = pigeon.getRoll();
    rollVelocity = pigeon.getAngularVelocityYWorld();

    this.networktablesTimestampConsumer = networkTablesTimestampConsumer;

    BaseStatusSignal.setUpdateFrequencyForAll(
        2 / GompeiLib.getLoopPeriod(), yawVelocity, pitch, pitchVelocity, roll, rollVelocity);

    pigeon.optimizeBusUtilization();

    PhoenixUtil.registerSignals(
        driveConstants.driveConfig.canBus().isNetworkFD(),
        yaw,
        yawVelocity,
        pitch,
        pitchVelocity,
        roll,
        rollVelocity);
  }

  @Trace
  @Override
  public void updateInputs(
      GyroIOInputs inputs, Queue<Double> yawTimestampQueue, Queue<Double> yawPositionQueue) {
    inputs.connected =
        BaseStatusSignal.isAllGood(yaw, yawVelocity, pitch, pitchVelocity, roll, rollVelocity);
    inputs.yawPosition = Rotation2d.fromDegrees(yaw.getValueAsDouble());
    networktablesTimestampConsumer.accept(NetworkTablesJNI.now());
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(yawVelocity.getValueAsDouble());

    inputs.pitchPosition = new Rotation2d(pitch.getValue());
    inputs.pitchVelocity = pitchVelocity.getValue();

    inputs.rollPosition = new Rotation2d(roll.getValue());
    inputs.rollVelocity = rollVelocity.getValue();

    inputs.odometryYawTimestamps =
        yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryYawPositions =
        yawPositionQueue.stream().map(Rotation2d::fromDegrees).toArray(Rotation2d[]::new);
  }
}
