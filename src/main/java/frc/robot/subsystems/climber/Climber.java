package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase{

    private ClimberIO io;
    private ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();
    private final Alert ClimberDisconnectedAlert;

    public Climber(ClimberIO io) {
        this.io = io;
        ClimberDisconnectedAlert = new Alert("Climber Freaking Disconnected Bruh", AlertType.kError);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Climber/IO", inputs);
        ClimberDisconnectedAlert.set(!inputs.climberConnected);
        // This method will be called once per scheduler run
    }

    public void setClimberVoltage(double speed) {
        io.setClimberVoltage(speed);
    }

    public void setClimberPosition(double position) {
            io.setClimberPosition(position);
    }
    
    public Command climbUp(){    
        return this.runOnce(() -> setClimberPosition(Constants.CLIMBER_UP_POSITION));
    }

    public Command climbDown(){    
        return this.runOnce(() -> setClimberPosition(Constants.CLIMBER_DOWN_POSITION));
    }
    public void setIdleMode(NeutralModeValue idleMode) {
        io.setIdleMode(idleMode);
    }
}
