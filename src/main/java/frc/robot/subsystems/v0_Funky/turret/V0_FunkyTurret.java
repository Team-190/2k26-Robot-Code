package frc.robot.subsystems.v0_Funky.turret;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class V0_FunkyTurret extends SubsystemBase {
    private final V0_FunkyTurretIO io;
    private final V0_FunkyTurretIOInputsLogged inputs;

    private Rotation2d previousPosition;
    private Rotation2d desiredRotations;


    
    private boolean atSetPoint;
  

    public V0_FunkyTurret(V0_FunkyTurretIO io){
        this.io = io;
        inputs = new V0_FunkyTurretIOInputsLogged();

      
        previousPosition = inputs.position;
        desiredRotations = new Rotation2d();

    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Turret",inputs);
        Logger.recordOutput("Turret At Setpoint", atSetPoint);
        Logger.recordOutput("Turret At Setpoint", previousPosition.getRadians() - desiredRotations.getRadians());
    }

    public boolean withinRange(Rotation2d angle){
        return (previousPosition.getDegrees() + angle.getDegrees() <= V0_FunkyTurretConstants.MAX_ANGLE && previousPosition.getDegrees() + angle.getDegrees() >= V0_FunkyTurretConstants.MIN_ANGLE);
    }
}
