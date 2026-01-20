package frc.robot.subsystems.v0_Funky.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

public class V0_FunkyTurret {
    private final V0_FunkyTurretIO io;
    private final String aKitTopic;
    private final V0_FunkyTurretIOInputsAutoLogged inputs;

    private final Rotation2d previousPosition;

    private final SysIdRoutine characterizationRoutine;

    private V0_FunkyTurretState state;

    public V0_FunkyTurret(V0_FunkyTurretIO io, Subsystem subsystem, int index) {
        this.io = io;
        inputs = new V0_FunkyTurretIOInputsAutoLogged();
        previousPosition = inputs.turretAngle;
        desiredRotations = new Rotation2d();

        characterizationRoutine =
                new SysIdRoutine(
                        new SysIdRoutine.Config(
                                Volts.of(0.5).per(Seconds),
                                Volts.of(3.5),
                                Seconds.of(10),
                                (state) -> Logger.recordOutput("Turret/sysIDState", state.toString())),
                        new SysIdRoutine.Mechanism(
                                (volts) -> io.setTurretVoltage(volts.in(Volts)), null, subsystem));

        aKitTopic = subsystem.getName() + "/Turret" + index;
        state = V0_FunkyTurretState.IDLE;
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs(aKitTopic, inputs);

        Logger.recordOutput(aKitTopic + "/At Goal", atTurretPositionGoal());

        switch (state) {
            case CLOSED_LOOP_ABSOLUTE_POSITION -> io.setTurretGoal(state.getRotation());
            case OPEN_LOOP_VOLTAGE_CONTROL -> io.setTurretVoltage(state.getVoltage());
            case CLOSED_LOOP_RELATIVE_POSITION -> io.setTurretGoal(
                    inputs.turretAngle.plus(state.getRotation()));
            default -> {
            }
        }
    }

    public boolean outOfRange(Rotation2d angle) {
        return (!(previousPosition.getDegrees() + angle.getDegrees()
                <= V0_FunkyTurretConstants.MAX_ANGLE)
                || !(previousPosition.getDegrees() + angle.getDegrees()
                >= V0_FunkyTurretConstants.MIN_ANGLE));
    }

    public Command setTurretVoltage(double volts) {
        return Commands.run(
                () -> {
                    state = V0_FunkyTurretState.OPEN_LOOP_VOLTAGE_CONTROL;
                    state.setVoltage(volts);
                });
    }

    public Command setTurretGoal(Rotation2d goal) {
        return Commands.run(
                () -> {
                    state = V0_FunkyTurretState.CLOSED_LOOP_ABSOLUTE_POSITION;
                    state.setRotation(goal);
                });
    }

    public Command stopTurret() {
        return setTurretVoltage(0);
    }

    public boolean atTurretPositionGoal() {
        return io.atTurretPositionGoal();
    }

    public Command incrementTurret(Rotation2d increment) {
        return Commands.run(
                () -> {
                    state = V0_FunkyTurretState.CLOSED_LOOP_RELATIVE_POSITION;
                    state.setRotation(increment);
                });
    }

    public void updateGains(double kP, double kD, double kS, double kV, double kA) {
        io.updateGains(kP, kD, kS, kV, kA);
    }

    public void updateConstraints(double maxAcceleration, double maxVelocity, double goalTolerance) {
        io.updateConstraints(maxAcceleration, maxVelocity, goalTolerance);
    }

    public void resetTurret() {
        io.setPosition(new Rotation2d());
    }

    public Command runSysId() {
        return Commands.sequence(
                Commands.runOnce(() -> state = V0_FunkyTurretState.CHARACTERIZING),
                characterizationRoutine
                        .quasistatic(Direction.kForward)
                        .until(() -> outOfRange(Rotation2d.fromRadians(V0_FunkyTurretConstants.CURRENT_ANGLE))),
                Commands.waitSeconds(3),
                characterizationRoutine
                        .quasistatic(Direction.kReverse)
                        .until(() -> outOfRange(Rotation2d.fromRadians(V0_FunkyTurretConstants.CURRENT_ANGLE))),
                Commands.waitSeconds(3),
                characterizationRoutine.dynamic(Direction.kForward),
                Commands.waitSeconds(3),
                characterizationRoutine.dynamic(Direction.kReverse));
    }

    /**
     * Method that calculates turret angle based on encoder values
     */
    @AutoLogOutput(key = "Turret" + "/Current Angle")
    public static Rotation2d calculateTurretAngle(Angle e1, Angle e2) {
        // Apply offsets and wrap to [-pi, pi)
        double a1 =
                MathUtil.angleModulus(e1.in(Units.Radians) - V0_FunkyTurretConstants.E1_OFFSET_RADIANS);

        double a2 =
                MathUtil.angleModulus(e2.in(Units.Radians) - V0_FunkyTurretConstants.E2_OFFSET_RADIANS);

        // Gear ratios
        double g0 = V0_FunkyTurretConstants.TURRET_ANGLE_CALCULATION.GEAR_0_TOOTH_COUNT();
        double g1 = V0_FunkyTurretConstants.TURRET_ANGLE_CALCULATION.GEAR_1_TOOTH_COUNT();
        double slope = V0_FunkyTurretConstants.TURRET_ANGLE_CALCULATION.SLOPE();

        // Initial estimate from encoder 1
        double baseTurret = a1 / g0;

        // Period of ambiguity from encoder 1
        double period = (2.0 * Math.PI) / g0;

        // Predicted encoder 2 value based on encoder 1
        double predictedA2 = MathUtil.angleModulus(g1 * baseTurret);

        // Error between predicted and actual encoder 2
        double error = MathUtil.angleModulus(predictedA2 - a2);

        double k = Math.round(error / (g1 * period));
        double turretAngle = baseTurret - k * period;

        turretAngle *= slope;

        return Rotation2d.fromRadians(turretAngle);
    }
}
