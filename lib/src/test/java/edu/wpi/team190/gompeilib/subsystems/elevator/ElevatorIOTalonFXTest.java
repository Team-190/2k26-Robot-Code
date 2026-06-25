package edu.wpi.team190.gompeilib.subsystems.elevator;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.team190.gompeilib.core.GompeiLib;
import edu.wpi.team190.gompeilib.core.robot.RobotMode;
import edu.wpi.team190.gompeilib.core.utility.control.Gains;
import edu.wpi.team190.gompeilib.core.utility.control.constraints.LinearConstraints;
import edu.wpi.team190.gompeilib.core.utility.phoenix.GainSlot;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.mockito.MockedConstruction;
import org.mockito.MockedStatic;

public class ElevatorIOTalonFXTest {
  private ElevatorConstants constants;

  @BeforeEach
  public void setUp() {
    edu.wpi.first.hal.HAL.initialize(500, 0);
    try {
      GompeiLib.deinit();
    } catch (Exception e) {
    }
    GompeiLib.init(RobotMode.SIM, false, 0.02);

    DCMotor motor = DCMotor.getNeo550(1);
    ElevatorConstants.ElevatorParameters params =
        ElevatorConstants.ElevatorParameters.builder()
            .withELEVATOR_MOTOR_CONFIG(motor)
            .withCARRIAGE_MASS_KG(15.0)
            .withMIN_HEIGHT(Units.Meters.of(0.0))
            .withMAX_HEIGHT(Units.Meters.of(1.5))
            .withNUM_MOTORS(1)
            .build();

    constants =
        ElevatorConstants.builder()
            .withLeaderCANID(5)
            .withElevatorGearRatio(10.0)
            .withDrumRadius(0.02)
            .withElevatorSupplyCurrentLimit(40.0)
            .withElevatorStatorCurrentLimit(40.0)
            .withElevatorParameters(params)
            .withSlot0Gains(
                Gains.fromDoubles()
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
                Gains.fromDoubles().withPrefix("slot1").withKP(2.0).withKI(0.0).withKD(0.2).build())
            .withSlot2Gains(
                Gains.fromDoubles().withPrefix("slot2").withKP(3.0).withKI(0.0).withKD(0.3).build())
            .withConstraints(
                LinearConstraints.fromMeasures()
                    .withPrefix("constraints")
                    .withMaxVelocity(Units.MetersPerSecond.of(2.0))
                    .withMaxAcceleration(Units.MetersPerSecondPerSecond.of(2.0))
                    .withGoalTolerance(Units.Meters.of(0.05))
                    .build())
            .withVoltageOffsetStep(Units.Volts.of(0.5))
            .withHeightOffsetStep(Units.Meters.of(0.05))
            .build();
  }

  @Test
  public void testElevatorIOTalonFX() {
    TalonFXConfigurator configurator = mock(TalonFXConfigurator.class);
    when(configurator.apply(any(TalonFXConfiguration.class))).thenReturn(StatusCode.OK);
    when(configurator.apply(any(TalonFXConfiguration.class), anyDouble()))
        .thenReturn(StatusCode.OK);

    StatusSignal<Angle> positionRotations = mock(StatusSignal.class);
    StatusSignal<AngularVelocity> velocityRotationsPerSecond = mock(StatusSignal.class);
    StatusSignal<AngularAcceleration> accelerationRotationsPerSecondPerSecond =
        mock(StatusSignal.class);
    StatusSignal<Voltage> appliedVolts = mock(StatusSignal.class);
    StatusSignal<Current> supplyCurrentAmps = mock(StatusSignal.class);
    StatusSignal<Current> torqueCurrentAmps = mock(StatusSignal.class);
    StatusSignal<Temperature> temperatureCelsius = mock(StatusSignal.class);
    StatusSignal<Double> positionSetpointRotations = mock(StatusSignal.class);
    StatusSignal<Double> positionErrorRotations = mock(StatusSignal.class);
    StatusSignal<Integer> closedLoopSlot = mock(StatusSignal.class);

    when(positionRotations.getValue()).thenReturn(Units.Rotations.of(10.0));
    when(positionRotations.getValueAsDouble()).thenReturn(10.0);
    when(velocityRotationsPerSecond.getValue()).thenReturn(Units.RotationsPerSecond.of(1.5));
    when(velocityRotationsPerSecond.getValueAsDouble()).thenReturn(1.5);
    when(accelerationRotationsPerSecondPerSecond.getValue())
        .thenReturn(Units.RotationsPerSecondPerSecond.of(0.5));
    when(appliedVolts.getValueAsDouble()).thenReturn(6.0);
    when(appliedVolts.getValue()).thenReturn(Units.Volts.of(6.0));
    when(supplyCurrentAmps.getValueAsDouble()).thenReturn(15.0);
    when(torqueCurrentAmps.getValueAsDouble()).thenReturn(12.0);
    when(temperatureCelsius.getValueAsDouble()).thenReturn(45.0);
    when(positionSetpointRotations.getValueAsDouble()).thenReturn(0.5);
    when(positionErrorRotations.getValueAsDouble()).thenReturn(0.01);
    when(closedLoopSlot.getValue()).thenReturn(0);

    try (MockedConstruction<TalonFX> mockTalon =
            mockConstruction(
                TalonFX.class,
                (mock, context) -> {
                  when(mock.getConfigurator()).thenReturn(configurator);
                  when(mock.getPosition()).thenReturn(positionRotations);
                  when(mock.getVelocity()).thenReturn(velocityRotationsPerSecond);
                  when(mock.getAcceleration()).thenReturn(accelerationRotationsPerSecondPerSecond);
                  when(mock.getMotorVoltage()).thenReturn(appliedVolts);
                  when(mock.getSupplyCurrent()).thenReturn(supplyCurrentAmps);
                  when(mock.getTorqueCurrent()).thenReturn(torqueCurrentAmps);
                  when(mock.getDeviceTemp()).thenReturn(temperatureCelsius);
                  when(mock.getClosedLoopReference()).thenReturn(positionSetpointRotations);
                  when(mock.getClosedLoopError()).thenReturn(positionErrorRotations);
                  when(mock.getClosedLoopSlot()).thenReturn(closedLoopSlot);
                });
        MockedStatic<BaseStatusSignal> mockBss = mockStatic(BaseStatusSignal.class)) {

      mockBss
          .when(
              () ->
                  BaseStatusSignal.setUpdateFrequencyForAll(
                      anyDouble(), any(BaseStatusSignal[].class)))
          .thenReturn(null);

      ElevatorIOTalonFX io = new ElevatorIOTalonFX(constants);
      ElevatorIO.ElevatorIOInputs inputs = new ElevatorIO.ElevatorIOInputs();

      io.updateInputs(inputs);

      assertEquals(10.0, inputs.position.in(Units.Meters), 0.01);
      assertEquals(1.5, inputs.velocity.in(Units.MetersPerSecond), 0.01);
      assertEquals(6.0, inputs.appliedVolts[0], 0.01);
      assertEquals(15.0, inputs.supplyCurrentAmps[0], 0.01);
      assertEquals(12.0, inputs.torqueCurrentAmps[0], 0.01);
      assertEquals(45.0, inputs.temperatureCelsius[0], 0.01);
      assertEquals(0.5, inputs.positionSetpointMeters.in(Units.Meters), 0.001);
      assertEquals(GainSlot.ZERO, inputs.gainSlot);

      io.setVoltageGoal(Units.Volts.of(8.0));
      io.setPositionGoal(Units.Meters.of(1.0));
      assertTrue(io.atVoltageGoal(Units.Volts.of(6.0)));
      assertFalse(io.atVoltageGoal(Units.Volts.of(0.0)));
      assertTrue(io.atPositionGoal(Units.Meters.of(10.0)));
      assertFalse(io.atPositionGoal(Units.Meters.of(5.0)));

      io.setPosition(Units.Meters.of(0.5));

      io.setGainSlot(GainSlot.ZERO);
      io.setGainSlot(GainSlot.ONE);
      io.setGainSlot(GainSlot.TWO);

      io.updateGains(Gains.fromDoubles().withPrefix("slot0").build(), GainSlot.ZERO);
      io.updateGains(Gains.fromDoubles().withPrefix("slot1").build(), GainSlot.ONE);
      io.updateGains(Gains.fromDoubles().withPrefix("slot2").build(), GainSlot.TWO);

      io.updateConstraints(
          LinearConstraints.fromMeasures()
              .withPrefix("constraints")
              .withGoalTolerance(Units.Meters.of(0.01))
              .withMaxVelocity(Units.MetersPerSecond.of(1.0))
              .withMaxAcceleration(Units.MetersPerSecondPerSecond.of(1.0))
              .build());
    }
  }
}
