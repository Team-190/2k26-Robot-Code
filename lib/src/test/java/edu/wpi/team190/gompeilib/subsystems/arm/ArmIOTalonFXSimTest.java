package edu.wpi.team190.gompeilib.subsystems.arm;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.team190.gompeilib.core.GompeiLib;
import edu.wpi.team190.gompeilib.core.robot.RobotMode;
import edu.wpi.team190.gompeilib.core.utility.control.CurrentLimits;
import edu.wpi.team190.gompeilib.core.utility.control.Gains;
import edu.wpi.team190.gompeilib.core.utility.control.constraints.AngularPositionConstraints;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.mockito.MockedConstruction;
import org.mockito.MockedStatic;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.system.DCMotor;
import org.wpilib.units.Units;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Temperature;
import org.wpilib.units.measure.Voltage;

@SuppressWarnings("unchecked") // Mockito mocks of generic types
public class ArmIOTalonFXSimTest {
  private ArmConstants constants;

  @BeforeEach
  public void setUp() {
    org.wpilib.hardware.hal.HAL.initialize();
    try {
      GompeiLib.deinit();
    } catch (Exception e) {
    }
    GompeiLib.init(RobotMode.SIM, false, 0.02);

    DCMotor motor = DCMotor.getNeo550(1);
    ArmConstants.ArmParameters params =
        ArmConstants.ArmParameters.builder()
            .withMotorConfig(motor)
            .withMinAngle(Rotation2d.fromDegrees(-90))
            .withMaxAngle(Rotation2d.fromDegrees(90))
            .withContinuousOutput(true)
            .withNumMotors(1)
            .withGearRatio(100.0)
            .withLengthMeters(0.5)
            .withMomentOfInertia(0.1)
            .build();

    constants =
        ArmConstants.builder()
            .withArmCANID(4)
            .withCanBus(new CANBus("rio"))
            .withArmParameters(params)
            .withSlot0Gains(
                Gains.builder()
                    .withPrefix("slot0")
                    .withKP(1.0)
                    .withKI(0.0)
                    .withKD(0.1)
                    .withKS(0.01)
                    .withKV(0.01)
                    .withKA(0.01)
                    .withKG(0.1)
                    .build())
            .withSlot1Gains(
                Gains.builder().withPrefix("slot1").withKP(2.0).withKI(0.0).withKD(0.2).build())
            .withSlot2Gains(
                Gains.builder().withPrefix("slot2").withKP(3.0).withKI(0.0).withKD(0.3).build())
            .withConstraints(
                AngularPositionConstraints.builder()
                    .withPrefix("constraints")
                    .withMaxVelocity(Units.RadiansPerSecond.of(2.0))
                    .withMaxAcceleration(Units.RadiansPerSecondPerSecond.of(2.0))
                    .withGoalTolerance(Units.Radians.of(0.05))
                    .build())
            .withCurrentLimits(
                CurrentLimits.fromDoubles()
                    .withSupplyCurrentLimit(40.0)
                    .withStatorCurrentLimit(40.0)
                    .build())
            .withEnableFOC(false)
            .withInvertedValue(InvertedValue.Clockwise_Positive)
            .withVoltageOffsetStep(Units.Volts.of(0.5))
            .withPositionOffsetStep(Rotation2d.fromDegrees(5))
            .build();
  }

  @Test
  public void testArmIOTalonFXSim() {
    TalonFXConfigurator configurator = mock(TalonFXConfigurator.class);
    when(configurator.apply(any(TalonFXConfiguration.class))).thenReturn(StatusCode.OK);
    when(configurator.apply(any(TalonFXConfiguration.class), anyDouble()))
        .thenReturn(StatusCode.OK);

    StatusSignal<Angle> positionRotations = mock(StatusSignal.class);
    StatusSignal<AngularVelocity> velocityRotationsPerSecond = mock(StatusSignal.class);
    StatusSignal<Voltage> appliedVolts = mock(StatusSignal.class);
    StatusSignal<Current> supplyCurrentAmps = mock(StatusSignal.class);
    StatusSignal<Current> torqueCurrentAmps = mock(StatusSignal.class);
    StatusSignal<Temperature> temperatureCelsius = mock(StatusSignal.class);
    StatusSignal<Double> positionSetpointRotations = mock(StatusSignal.class);
    StatusSignal<Double> positionErrorRotations = mock(StatusSignal.class);
    StatusSignal<Integer> closedLoopSlot = mock(StatusSignal.class);

    when(positionRotations.getValue()).thenReturn(Rotation2d.fromDegrees(10.0).getMeasure());
    when(positionRotations.getValueAsDouble())
        .thenReturn(Rotation2d.fromDegrees(10.0).getRotations());
    when(velocityRotationsPerSecond.getValue()).thenReturn(Units.RadiansPerSecond.of(1.5));
    when(velocityRotationsPerSecond.getValueAsDouble()).thenReturn(1.5);
    when(appliedVolts.getValueAsDouble()).thenReturn(6.0);
    when(appliedVolts.getValue()).thenReturn(Units.Volts.of(6.0));
    when(supplyCurrentAmps.getValueAsDouble()).thenReturn(15.0);
    when(torqueCurrentAmps.getValueAsDouble()).thenReturn(12.0);
    when(temperatureCelsius.getValueAsDouble()).thenReturn(45.0);
    when(positionSetpointRotations.getValueAsDouble()).thenReturn(0.5);
    when(positionErrorRotations.getValueAsDouble()).thenReturn(0.01);
    when(closedLoopSlot.getValue()).thenReturn(0);

    TalonFXSimState simState = mock(TalonFXSimState.class);
    when(simState.getMotorVoltage()).thenReturn(6.0);

    try (MockedConstruction<TalonFX> _ =
            mockConstruction(
                TalonFX.class,
                (mock, context) -> {
                  when(mock.getConfigurator()).thenReturn(configurator);
                  when(mock.getPosition()).thenReturn(positionRotations);
                  when(mock.getVelocity()).thenReturn(velocityRotationsPerSecond);
                  when(mock.getMotorVoltage()).thenReturn(appliedVolts);
                  when(mock.getSupplyCurrent()).thenReturn(supplyCurrentAmps);
                  when(mock.getTorqueCurrent()).thenReturn(torqueCurrentAmps);
                  when(mock.getDeviceTemp()).thenReturn(temperatureCelsius);
                  when(mock.getClosedLoopReference()).thenReturn(positionSetpointRotations);
                  when(mock.getClosedLoopError()).thenReturn(positionErrorRotations);
                  when(mock.getClosedLoopSlot()).thenReturn(closedLoopSlot);
                  when(mock.getSimState()).thenReturn(simState);
                });
        MockedStatic<BaseStatusSignal> mockBss = mockStatic(BaseStatusSignal.class)) {

      mockBss
          .when(
              () ->
                  BaseStatusSignal.setUpdateFrequencyForAll(
                      anyDouble(), any(BaseStatusSignal[].class)))
          .thenReturn(null);

      ArmIOTalonFXSim sim = new ArmIOTalonFXSim(constants);
      ArmIO.ArmIOInputs inputs = new ArmIO.ArmIOInputs();

      sim.updateInputs(inputs);

      assertEquals(10.0, inputs.position.getDegrees(), 0.01);
      assertEquals(1.5, inputs.velocity.in(Units.RadiansPerSecond), 0.01);

      verify(simState).setSupplyVoltage(anyDouble());
      verify(simState).setRawRotorPosition(any(Angle.class));
      verify(simState).setRotorVelocity(any(AngularVelocity.class));
    }
  }
}
