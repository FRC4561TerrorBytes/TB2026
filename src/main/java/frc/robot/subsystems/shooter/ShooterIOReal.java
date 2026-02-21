package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;

public class ShooterIOReal implements ShooterIO {
    private final TalonFX flywheelLeftTopMotor = new TalonFX(Constants.FLYWHEEL_TOP_LEFT_ID);
    private final TalonFX flywheelLeftBottomMotor = new TalonFX(Constants.FLYWHEEL_BOTTOM_LEFT_ID);
    private final TalonFX flywheelRightTopMotor = new TalonFX(Constants.FLYWHEEL_TOP_RIGHT_ID);
    private final TalonFX flywheelRightBottomMotor = new TalonFX(Constants.FLYWHEEL_BOTTOM_RIGHT_ID);
    private final TalonFX hoodMotor = new TalonFX(Constants.HOOD_ID);

    private final MotionMagicVelocityVoltage flywheelLeftControl = new MotionMagicVelocityVoltage(0);
    private final MotionMagicVelocityVoltage flywheelRightControl = new MotionMagicVelocityVoltage(0);
    private final MotionMagicVoltage hoodControl = new MotionMagicVoltage(0);

    private final StatusSignal<AngularVelocity> flywheelLeftTopVelocity;
    private final StatusSignal<Voltage> flywheelLeftTopVoltage;
    private final StatusSignal<Current> flywheelLeftTopCurrent;
    private final StatusSignal<AngularVelocity> flywheelLeftBottomVelocity;
    private final StatusSignal<Voltage> flywheelLeftBottomVoltage;
    private final StatusSignal<Current> flywheelLeftBottomCurrent;

    private final StatusSignal<AngularVelocity> flywheelRightTopVelocity;
    private final StatusSignal<Voltage> flywheelRightTopVoltage;
    private final StatusSignal<Current> flywheelRightTopCurrent;
    private final StatusSignal<AngularVelocity> flywheelRightBottomVelocity;
    private final StatusSignal<Voltage> flywheelRightBottomVoltage;
    private final StatusSignal<Current> flywheelRightBottomCurrent;

    private final StatusSignal<AngularVelocity> hoodVelocity;
    private final StatusSignal<Voltage> hoodVoltage;
    private final StatusSignal<Current> hoodCurrent;
    private final StatusSignal<Angle> hoodRelativePosition;

    public ShooterIOReal () { 
        //constructor go brrrrrrr
        var flywheelConfig = new TalonFXConfiguration();

        flywheelConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        flywheelConfig.CurrentLimits.SupplyCurrentLimit = Constants.FLYWHEELS_SUPPLY_CURRENT_LIMIT;

        flywheelConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        flywheelConfig.CurrentLimits.StatorCurrentLimit = Constants.FLYWHEELS_STATOR_CURRENT_LIMIT;

        flywheelConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        
        var flywheelLeftSlot0Config = flywheelConfig.Slot0;
        flywheelLeftSlot0Config.kS = 0.5; // Add 0.25 V output to overcome static friction
        flywheelLeftSlot0Config.kV = 0.15; // A velocity target of 1 rps results in 0.12 V output
        flywheelLeftSlot0Config.kA = 0.02; // An acceleration of 1 rps/s requires 0.01 V output
        flywheelLeftSlot0Config.kP = 0.4; // An error of 1 rps results in 0.11 V output
        flywheelLeftSlot0Config.kI = 0.0; // no output for integrated error
        flywheelLeftSlot0Config.kD = 0.0; // no output for error derivative

        var flywheelMotionMagicConfig = flywheelConfig.MotionMagic;
        flywheelMotionMagicConfig.MotionMagicAcceleration = 10; // Target acceleration of 400 rps/s (0.25 seconds to max)
        flywheelMotionMagicConfig.MotionMagicJerk = 4000; // Target jerk of 4000 rps/s/s (0.1 seconds)

        flywheelLeftTopMotor.getConfigurator().apply(flywheelConfig.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive));
        flywheelLeftBottomMotor.getConfigurator().apply(flywheelConfig.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive));

        flywheelRightTopMotor.getConfigurator().apply(flywheelConfig.MotorOutput.withInverted(InvertedValue.Clockwise_Positive));
        flywheelRightBottomMotor.getConfigurator().apply(flywheelConfig.MotorOutput.withInverted(InvertedValue.Clockwise_Positive));

        flywheelLeftTopVelocity = flywheelLeftTopMotor.getVelocity();
        flywheelLeftTopVoltage = flywheelLeftTopMotor.getMotorVoltage();
        flywheelLeftTopCurrent = flywheelLeftTopMotor.getStatorCurrent();
        flywheelLeftBottomVelocity = flywheelLeftBottomMotor.getVelocity();
        flywheelLeftBottomVoltage = flywheelLeftBottomMotor.getMotorVoltage();
        flywheelLeftBottomCurrent = flywheelLeftBottomMotor.getStatorCurrent();

        flywheelRightTopVelocity = flywheelRightTopMotor.getVelocity();
        flywheelRightTopVoltage = flywheelRightTopMotor.getMotorVoltage();
        flywheelRightTopCurrent = flywheelRightTopMotor.getStatorCurrent();
        flywheelRightBottomVelocity = flywheelRightBottomMotor.getVelocity();
        flywheelRightBottomVoltage = flywheelRightBottomMotor.getMotorVoltage();
        flywheelRightBottomCurrent = flywheelRightBottomMotor.getStatorCurrent();


        var hoodConfig = new TalonFXConfiguration();

        hoodConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        hoodConfig.CurrentLimits.StatorCurrentLimit = Constants.HOOD_STATOR_CURRENT_LIMIT;

        hoodConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        hoodConfig.CurrentLimits.SupplyCurrentLimit = Constants.HOOD_SUPPLY_CURRENT_LIMIT;

        hoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        var hoodSlot0Config = hoodConfig.Slot0;
        hoodSlot0Config.kS = 0.5; // Add 0.25 V output to overcome static friction
        hoodSlot0Config.kV = 0.15; // A velocity target of 1 rps results in 0.12 V output
        hoodSlot0Config.kA = 0.02; // An acceleration of 1 rps/s requires 0.01 V output
        hoodSlot0Config.kP = 0.4; // An error of 1 rps results in 0.11 V output
        hoodSlot0Config.kI = 0.0; // no output for integrated error
        hoodSlot0Config.kD = 0.0; // no output for error derivative

        var hoodMotionMagicConfig = hoodConfig.MotionMagic;
        hoodMotionMagicConfig.MotionMagicAcceleration = 10; // Target acceleration of 400 rps/s (0.25 seconds to max)
        hoodMotionMagicConfig.MotionMagicJerk = 4000; // Target jerk of 4000 rps/s/s (0.1 seconds)

        hoodMotor.getConfigurator().apply(hoodConfig);

        hoodVelocity = hoodMotor.getVelocity();
        hoodVoltage = hoodMotor.getMotorVoltage();
        hoodCurrent = hoodMotor.getStatorCurrent();
        hoodRelativePosition = hoodMotor.getPosition();

        BaseStatusSignal.setUpdateFrequencyForAll(50.0, 
            flywheelLeftTopVelocity,
            flywheelLeftTopVoltage,
            flywheelLeftTopCurrent,
            flywheelLeftBottomVelocity,
            flywheelLeftBottomVoltage,
            flywheelLeftBottomCurrent,
            flywheelRightTopVelocity,
            flywheelRightTopVoltage,
            flywheelRightTopCurrent,
            flywheelRightBottomVelocity,
            flywheelRightBottomVoltage,
            flywheelRightBottomCurrent,
            hoodVelocity,
            hoodVoltage,
            hoodCurrent,
            hoodRelativePosition
            );

        ParentDevice.optimizeBusUtilizationForAll(flywheelLeftTopMotor);
        ParentDevice.optimizeBusUtilizationForAll(flywheelLeftBottomMotor);
        ParentDevice.optimizeBusUtilizationForAll(flywheelRightTopMotor);
        ParentDevice.optimizeBusUtilizationForAll(flywheelRightBottomMotor);
        ParentDevice.optimizeBusUtilizationForAll(hoodMotor);

        flywheelLeftBottomMotor.setControl(new Follower(flywheelLeftTopMotor.getDeviceID(), MotorAlignmentValue.Aligned));
        flywheelRightBottomMotor.setControl(new Follower(flywheelRightTopMotor.getDeviceID(), MotorAlignmentValue.Aligned));
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        var flywheelLeftTopStatus = BaseStatusSignal.refreshAll(
            flywheelLeftTopVelocity,
            flywheelLeftTopVoltage,
            flywheelLeftTopCurrent
        );
        var flywheelLeftBottomStatus = BaseStatusSignal.refreshAll(
            flywheelLeftBottomVelocity,
            flywheelLeftBottomVoltage,
            flywheelLeftBottomCurrent
        );

        var flywheelRightTopStatus = BaseStatusSignal.refreshAll(
            flywheelRightTopVelocity,
            flywheelRightTopVoltage,
            flywheelRightTopCurrent
        );
        var flywheelRightBottomStatus = BaseStatusSignal.refreshAll(
            flywheelRightBottomVelocity,
            flywheelRightBottomVoltage,
            flywheelRightBottomCurrent
        );

        //AHHHHHHHHHHHHHHHHH SO MUCH COPY AND PASTE AHHHHHHHHHHHHHHHHHHHHH

        var hoodStatus = BaseStatusSignal.refreshAll(
            hoodVelocity,
            hoodVoltage,
            hoodCurrent,
            hoodRelativePosition);

        inputs.flywheelLeftTopConnected = flywheelLeftTopStatus.isOK();
        inputs.flywheelLeftBottomConnected = flywheelLeftBottomStatus.isOK();
        inputs.flywheelRightTopConnected = flywheelRightTopStatus.isOK();
        inputs.flywheelRightBottomConnected = flywheelRightBottomStatus.isOK();
        inputs.hoodConnected = hoodStatus.isOK();

        //yiiiiipppppppppppppppppppppeeeeeeeeeeeeeeeeee - tyler
        inputs.flywheelLeftTopVelocity = flywheelLeftTopVelocity.getValueAsDouble();
        inputs.flywheelLeftTopVoltage = flywheelLeftTopVoltage.getValueAsDouble();
        inputs.flywheelLeftTopCurrent = flywheelLeftTopCurrent.getValueAsDouble();
        inputs.flywheelLeftBottomVelocity = flywheelLeftBottomVelocity.getValueAsDouble();
        inputs.flywheelLeftBottomVoltage = flywheelLeftBottomVoltage.getValueAsDouble();
        inputs.flywheelLeftBottomCurrent = flywheelLeftBottomCurrent.getValueAsDouble();

        inputs.flywheelRightTopVelocity = flywheelRightTopVelocity.getValueAsDouble();
        inputs.flywheelRightTopVoltage = flywheelRightTopVoltage.getValueAsDouble();
        inputs.flywheelRightTopCurrent = flywheelRightTopCurrent.getValueAsDouble();
        inputs.flywheelRightBottomVelocity = flywheelRightBottomVelocity.getValueAsDouble();
        inputs.flywheelRightBottomVoltage = flywheelRightBottomVoltage.getValueAsDouble();
        inputs.flywheelRightBottomCurrent = flywheelRightBottomCurrent.getValueAsDouble();

        inputs.hoodVelocity = hoodVelocity.getValueAsDouble();
        inputs.hoodVoltage = hoodVoltage.getValueAsDouble();
        inputs.hoodCurrent = hoodCurrent.getValueAsDouble();
        inputs.hoodRelativePosition = hoodRelativePosition.getValueAsDouble();
    }

    public void setLeftFlywheelVoltage(double voltage){
        flywheelLeftTopMotor.set(voltage);
    }

    public void setRightFlywheelVoltage(double voltage){
        flywheelRightTopMotor.set(voltage);
    }

    public void setLeftFlywheelSpeed(double velocityRPS){
        flywheelLeftTopMotor.setControl(flywheelLeftControl.withVelocity(velocityRPS));
    }

    public void setRightFlywheelSpeed(double velocityRPS){
        flywheelRightTopMotor.setControl(flywheelRightControl.withVelocity(velocityRPS));
    }

    public void setHoodAngle(double position){
        hoodMotor.setControl(hoodControl.withPosition(position));
    }

    public void stopflywheelLeft(){
        flywheelLeftTopMotor.set(0); 
        flywheelLeftBottomMotor.set(0);
    }

    public void stopflywheelRight(){
        flywheelRightTopMotor.set(0); 
        flywheelRightBottomMotor.set(0);
    }
}