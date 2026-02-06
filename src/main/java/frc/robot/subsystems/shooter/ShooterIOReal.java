package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.ShooterIO.ShooterIOInputs;

public class ShooterIOReal implements ShooterIO {
    private final TalonFX flywheelLeftMotor = new TalonFX(Constants.FLYWHEELLEFT_ID);
    private final TalonFX flywheelRightMotor = new TalonFX(Constants.FLYWHEELRIGHT_ID);
    private final TalonFX hoodMotor = new TalonFX(Constants.HOOD_ID);

    private final MotionMagicVelocityVoltage flywheelLeftControl = new MotionMagicVelocityVoltage(0);
    private final MotionMagicVelocityVoltage flywheelRightControl = new MotionMagicVelocityVoltage(0);
    private final MotionMagicVoltage hoodControl = new MotionMagicVoltage(0);

    private final StatusSignal<AngularVelocity> flywheelLeftVelocity;
    private final StatusSignal<Voltage> flywheelLeftVoltage;
    private final StatusSignal<Current> flywheelLeftCurrent;

    private final StatusSignal<AngularVelocity> flywheelRightVelocity;
    private final StatusSignal<Voltage> flywheelRightVoltage;
    private final StatusSignal<Current> flywheelRightCurrent;

    private final StatusSignal<AngularVelocity> hoodVelocity;
    private final StatusSignal<Voltage> hoodVoltage;
    private final StatusSignal<Current> hoodCurrent;

    public ShooterIOReal () { 
        //constructor go brrrrrrr
        var flywheelLeftConfig = new TalonFXConfiguration();
        var flywheelRightConfig = new TalonFXConfiguration();
        var hoodConfig = new TalonFXConfiguration();

        flywheelLeftConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        flywheelLeftConfig.CurrentLimits.SupplyCurrentLimit = Constants.FLYWHEELS_SUPPLY_CURRENT_LIMIT;

        hoodConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        hoodConfig.CurrentLimits.SupplyCurrentLimit = Constants.HOOD_SUPPLY_CURRENT_LIMIT;

        flywheelLeftConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        flywheelLeftConfig.CurrentLimits.StatorCurrentLimit = Constants.FLYWHEELS_STATOR_CURRENT_LIMIT;

        flywheelLeftConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        
        var flywheelLeftSlot0Config = flywheelLeftConfig.Slot0;
        flywheelLeftSlot0Config.kS = 0.5; // Add 0.25 V output to overcome static friction
        flywheelLeftSlot0Config.kV = 0.15; // A velocity target of 1 rps results in 0.12 V output
        flywheelLeftSlot0Config.kA = 0.02; // An acceleration of 1 rps/s requires 0.01 V output
        flywheelLeftSlot0Config.kP = 0.4; // An error of 1 rps results in 0.11 V output
        flywheelLeftSlot0Config.kI = 0.0; // no output for integrated error
        flywheelLeftSlot0Config.kD = 0.0; // no output for error derivative

        var flywheelMotionMagicConfig = flywheelLeftConfig.MotionMagic;
        flywheelMotionMagicConfig.MotionMagicAcceleration = 10; // Target acceleration of 400 rps/s (0.25 seconds to max)
        flywheelMotionMagicConfig.MotionMagicJerk = 4000; // Target jerk of 4000 rps/s/s (0.1 seconds)

        flywheelLeftMotor.getConfigurator().apply(flywheelLeftConfig);
        flywheelRightMotor.getConfigurator().apply(flywheelLeftConfig);

        flywheelLeftVelocity = flywheelLeftMotor.getVelocity();
        flywheelLeftVoltage = flywheelLeftMotor.getMotorVoltage();
        flywheelLeftCurrent = flywheelLeftMotor.getStatorCurrent();

        flywheelRightVelocity = flywheelRightMotor.getVelocity();
        flywheelRightVoltage = flywheelRightMotor.getMotorVoltage();
        flywheelRightCurrent = flywheelRightMotor.getStatorCurrent();

        hoodVelocity = hoodMotor.getVelocity();
        hoodVoltage = hoodMotor.getMotorVoltage();
        hoodCurrent = hoodMotor.getStatorCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(50.0, 
            flywheelLeftVelocity,
            flywheelLeftVoltage,
            flywheelLeftCurrent,
            flywheelRightVelocity,
            flywheelRightVoltage,
            flywheelRightCurrent,
            hoodVelocity,
            hoodVoltage,
            hoodCurrent
            );

        ParentDevice.optimizeBusUtilizationForAll(flywheelLeftMotor);
        ParentDevice.optimizeBusUtilizationForAll(flywheelRightMotor);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        var flywheelLeftStatus = BaseStatusSignal.refreshAll(
            flywheelLeftVelocity,
            flywheelLeftVoltage,
            flywheelLeftCurrent
        );

        var flywheelRightStatus = BaseStatusSignal.refreshAll(
            flywheelRightVelocity,
            flywheelRightVoltage,
            flywheelRightCurrent
        );


        var hoodStatus = BaseStatusSignal.refreshAll(
             hoodVelocity,
             hoodVoltage,
             hoodCurrent );

        inputs.flywheelLeftConnected = flywheelLeftStatus.isOK();
        inputs.flywheelRightConnected = flywheelRightStatus.isOK();
        
        inputs.hoodConnected = hoodStatus.isOK();

        //yiiiiipppppppppppppppppppppeeeeeeeeeeeeeeeeee - tyler
        inputs.flywheelLeftVelocity = flywheelLeftVelocity.getValueAsDouble();
        inputs.flywheelLeftVoltage = flywheelLeftVoltage.getValueAsDouble();
        inputs.flywheelLeftCurrent = flywheelLeftCurrent.getValueAsDouble();

        inputs.flywheelRightVelocity = flywheelRightVelocity.getValueAsDouble();
        inputs.flywheelRightVoltage = flywheelRightVoltage.getValueAsDouble();
        inputs.flywheelRightCurrent = flywheelRightCurrent.getValueAsDouble();

        inputs.hoodVelocity = hoodVelocity.getValueAsDouble();
        inputs.hoodVoltage = hoodVoltage.getValueAsDouble();
        inputs.hoodCurrent = hoodCurrent.getValueAsDouble();
    }
        public void setflywheelLeftSpeed(double velocity){
        // VELOCITY IN MPS
        velocity = velocity/Constants.FLYWHEELS_CIRCUMFERENCE;
        flywheelLeftMotor.setControl(flywheelLeftControl.withVelocity(velocity));
        }

        public void setflywheelRightSpeed(double velocity){
        // VELOCITY IN MPS
        velocity = velocity/Constants.FLYWHEELS_CIRCUMFERENCE;
        flywheelRightMotor.setControl(flywheelRightControl.withVelocity(velocity));
        }

        public void setHoodAngle(double angle){
        // ANGLE IN SMTH
        hoodMotor.setControl(hoodControl.withPosition(angle));
        }

        public void stopflywheelLeft(){
        flywheelLeftMotor.set(0); 
    }

        public void stopflywheelRight(){
        flywheelRightMotor.set(0); 
    }
}