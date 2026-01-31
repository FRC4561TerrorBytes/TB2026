package frc.robot.subsystems.climber;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.Constants;

public class ClimberIOReal implements ClimberIO {
public final TalonFX climberMotor = new TalonFX(Constants.CLIMBER_ID);
private final StatusSignal<Voltage> climberVoltage;
private final StatusSignal<AngularVelocity> climberVelocity;
private final StatusSignal<Current> climberCurrent;
private final StatusSignal<Angle> climberPosition;

private final MotionMagicVoltage climberControl = new MotionMagicVoltage(0);

private final Alert climberAlert = new Alert("Climber Disconnected.", AlertType.kWarning);

public ClimberIOReal() {
    //Update 
    var climberConfig = new TalonFXConfiguration();
    climberConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    climberConfig.CurrentLimits.SupplyCurrentLimit = Constants.CLIMBER_SUPPLY_CURRENT_LIMIT;
    climberConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    climberConfig.CurrentLimits.StatorCurrentLimit = Constants.CLIMBER_STATOR_CURRENT_LIMIT;

    climberConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        
    var climberSlot0Config = climberConfig.Slot0;
    climberSlot0Config.kS = 0.5; // Add 0.25 V output to overcome static friction
    climberSlot0Config.kV = 0.15; // A velocity target of 1 rps results in 0.12 V output
    climberSlot0Config.kA = 0.02; // An acceleration of 1 rps/s requires 0.01 V output
    climberSlot0Config.kP = 0.4; // An error of 1 rps results in 0.11 V output
    climberSlot0Config.kI = 0.0; // no output for integrated error
    climberSlot0Config.kD = 0.0; // no output for error derivative

    var leftMotionMagicConfig = climberConfig.MotionMagic;
    leftMotionMagicConfig.MotionMagicAcceleration = 10; // Target acceleration of 400 rps/s (0.25 seconds to max)
    leftMotionMagicConfig.MotionMagicJerk = 4000; // Target jerk of 4000 rps/s/s (0.1 seconds)

    climberMotor.getConfigurator().apply(climberConfig);

    climberVelocity = climberMotor.getVelocity();
    climberVoltage = climberMotor.getMotorVoltage();
    climberCurrent = climberMotor.getStatorCurrent();
    climberPosition = climberMotor.getPosition();

    BaseStatusSignal.setUpdateFrequencyForAll(50.0, 
            climberVelocity,
            climberVoltage,
            climberCurrent,
            climberPosition
            );

    ParentDevice.optimizeBusUtilizationForAll(climberMotor);
}

@Override
public void updateInputs(ClimberIOInputs inputs) {
    var climberStatus = BaseStatusSignal.refreshAll(
        climberVelocity,
        climberVoltage,
        climberCurrent
    );

        inputs.climberConnected = climberStatus.isOK();

        inputs.climberVelocity = climberVelocity.getValueAsDouble();
        inputs.climberVoltage = climberVoltage.getValueAsDouble();
        inputs.climberCurrent = climberCurrent.getValueAsDouble();
        inputs.climberPosition = climberPosition.getValueAsDouble();
    }

    @Override
    public void setClimberVoltage(double voltage) {
        climberMotor.set(voltage);
    }

    @Override
    public void setClimberPosition(double position) {
        climberMotor.setControl(climberControl.withPosition(position));
    }
}
