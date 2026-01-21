package frc.robot.subsystems.climber;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.drive.ModuleIO.ModuleIOInputs;

public class ClimberIOReal implements ClimberIO {
public final TalonFX climbertTalonFX;
private final StatusSignal<Voltage> climberAppliedVolts;
private final StatusSignal<Current> CurrentAmps;

@Override
public void updateInputs(ClimberIOInputs inputs) {
    //Update 
    inputs.climberConnected = climberConnected
}


}
