package frc.robot.subsystems.flywheels;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.subsystems.flywheels.FlywheelsIO.FlywheelsIOInputs;

public class FlywheelsIOReal implements FlywheelsIO {
    private final TalonFX flywheelsTalon = new TalonFX(Constants.TURRET_FLYWHEELS_ID);

    private final MotionMagicVelocityVoltage m_request = new MotionMagicVelocityVoltage(0);

    private final StatusSignal<AngularVelocity> flywheelsVelocity;
    private final StatusSignal<Voltage> flywheelsVoltage;
    private final StatusSignal<Current> flywheelsCurrent;

    public FlywheelsIOReal () { 
        //constructor go brrrrrrr
        var flywheelsConfig = new TalonFXConfiguration();

        flywheelsConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        flywheelsConfig.CurrentLimits.SupplyCurrentLimit = Constants.FLYWHEELS_SUPPLY_CURRENT_LIMIT;

        flywheelsConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        flywheelsConfig.CurrentLimits.StatorCurrentLimit = Constants.FLYWHEELS_STATOR_CURRENT_LIMIT;

        flywheelsConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        
        var flywheelsSlot0Config = flywheelsConfig.Slot0;
        flywheelsSlot0Config.kS = 0.5; // Add 0.25 V output to overcome static friction
        flywheelsSlot0Config.kV = 0.15; // A velocity target of 1 rps results in 0.12 V output
        flywheelsSlot0Config.kA = 0.02; // An acceleration of 1 rps/s requires 0.01 V output
        flywheelsSlot0Config.kP = 0.4; // An error of 1 rps results in 0.11 V output
        flywheelsSlot0Config.kI = 0.0; // no output for integrated error
        flywheelsSlot0Config.kD = 0.0; // no output for error derivative

        var leftMotionMagicConfig = flywheelsConfig.MotionMagic;
        leftMotionMagicConfig.MotionMagicAcceleration = 10; // Target acceleration of 400 rps/s (0.25 seconds to max)
        leftMotionMagicConfig.MotionMagicJerk = 4000; // Target jerk of 4000 rps/s/s (0.1 seconds)

        flywheelsTalon.getConfigurator().apply(flywheelsConfig);

        flywheelsVelocity = flywheelsTalon.getVelocity();
        flywheelsVoltage = flywheelsTalon.getMotorVoltage();
        flywheelsCurrent = flywheelsTalon.getStatorCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(50.0, 
            flywheelsVelocity,
            flywheelsVoltage,
            flywheelsCurrent );

        ParentDevice.optimizeBusUtilizationForAll(flywheelsTalon);
    }

    @Override
    public void updateInputs(FlywheelsIOInputs inputs) {
        var flywheelsStatus = BaseStatusSignal.refreshAll(
            flywheelsVelocity,
            flywheelsVoltage,
            flywheelsCurrent
        );

        inputs.flywheelsConnected = flywheelsStatus.isOK();

        inputs.flywheelsVelocity = flywheelsVelocity.getValueAsDouble();
        inputs.flywheelsVoltage = flywheelsVoltage.getValueAsDouble();
        inputs.flywheelsCurrent = flywheelsCurrent.getValueAsDouble();
    }
        public void setFlywheelsSpeed(double velocity){
        // VELOCITY IN MPS
        velocity = velocity/Constants.FLYWHEELS_CIRCUMFERENCE;
        flywheelsTalon.setControl(m_request.withVelocity(velocity));
        }

        public void stopFlywheels(){
        flywheelsTalon.set(0);
    }
}

