package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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
}
