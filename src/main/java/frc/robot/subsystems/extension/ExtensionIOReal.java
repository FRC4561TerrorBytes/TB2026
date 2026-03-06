// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.extension;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.Constants;

/** Add your docs here. */ 
public class ExtensionIOReal implements ExtensionIO{
  private CANcoder extensionEncoder = new CANcoder(Constants.EXTENSION_CANCODER_ID);
  private TalonFX extensionMotor = new TalonFX(Constants.EXTENSION_ID);

  private final StatusSignal<Angle> extensionAngle;
  private final StatusSignal<Current> extensionStatorCurrent;
  private final StatusSignal<Current> extensionSupplyCurrent;
  private final StatusSignal<AngularVelocity> extensionSpeed;
  private final StatusSignal<Voltage> extensionVoltage;
  private final StatusSignal<Temperature> extensionTemp;

  private double extensionSetpoint = 0.0;

  private final MotionMagicVoltage m_request_extension = new MotionMagicVoltage(0);
  private final Alert extensionAlert = 
      new Alert( "Extension Motor Disconnected.", AlertType.kWarning );
  private final Alert extensionEncoderAlert =
      new Alert("Extension Encoder Disconnected.", AlertType.kWarning);

  public ExtensionIOReal() {

    var extensionPIDConfig = new Slot0Configs();
    extensionPIDConfig.GravityType = GravityTypeValue.Arm_Cosine;
    extensionPIDConfig.kS = 0.28;
    extensionPIDConfig.kV = 3.2;
    extensionPIDConfig.kA = 0.04;
    extensionPIDConfig.kP = 10; 
    extensionPIDConfig.kI = 0;
    extensionPIDConfig.kD = 0;

    var cancoderConfig = new CANcoderConfiguration();
    cancoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    cancoderConfig.MagnetSensor.MagnetOffset = -0.333496;
    tryUntilOk(5, () -> extensionEncoder.getConfigurator().apply(cancoderConfig, 0.25));

    var extensionConfig = new TalonFXConfiguration();
    extensionConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    extensionConfig.Slot0 = extensionPIDConfig;
    extensionConfig.Feedback.FeedbackRemoteSensorID = extensionEncoder.getDeviceID();
    extensionConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    extensionConfig.Feedback.RotorToSensorRatio = Constants.EXTENSION_GEAR_RATIO;
    extensionConfig.MotionMagic.MotionMagicCruiseVelocity = 100 / Constants.EXTENSION_GEAR_RATIO;
    extensionConfig.MotionMagic.MotionMagicAcceleration =
    extensionConfig.MotionMagic.MotionMagicCruiseVelocity / 0.030;
    extensionConfig.MotionMagic.MotionMagicExpo_kV = 0.12 * Constants.EXTENSION_GEAR_RATIO;
    extensionConfig.MotionMagic.MotionMagicExpo_kA = 0.1;
    extensionConfig.ClosedLoopGeneral.ContinuousWrap = false;
    extensionConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    extensionConfig.CurrentLimits.StatorCurrentLimit = Constants.EXTENSION_STATOR_CURRENT_LIMIT;
    extensionConfig.CurrentLimits.SupplyCurrentLimit = Constants.EXTENSION_SUPPLY_CURRENT_LIMIT;
    extensionConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    extensionConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    extensionConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = !true;
    extensionConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.32;
    tryUntilOk(5, () -> extensionMotor.getConfigurator().apply(extensionConfig, 0.25));

    extensionAngle = extensionEncoder.getPosition();
    extensionStatorCurrent = extensionMotor.getStatorCurrent();
    extensionSupplyCurrent = extensionMotor.getSupplyCurrent();
    extensionSpeed = extensionMotor.getVelocity();
    extensionVoltage = extensionMotor.getMotorVoltage();
    extensionTemp = extensionMotor.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        extensionAngle,
        extensionStatorCurrent,
        extensionSupplyCurrent,
        extensionSpeed,
        extensionVoltage,
        extensionTemp);

    ParentDevice.optimizeBusUtilizationForAll(extensionMotor);

  }

@Override
  public void updateInputs(ExtensionIOInputs inputs) {
    var extensionEncoderStatus = 
        BaseStatusSignal.refreshAll(
          extensionAngle
        );
    var extensionStatus =
        BaseStatusSignal.refreshAll(
            extensionStatorCurrent,
            extensionSupplyCurrent,
            extensionSpeed,
            extensionVoltage,
            extensionTemp);


    inputs.extensionEncoderConnected = extensionEncoderStatus.isOK();
    inputs.extensionAngle = extensionEncoder.getPosition().getValueAsDouble();
    inputs.extensionMotorConnected = extensionStatus.isOK();
    inputs.extensionStatorCurrent = extensionStatorCurrent.getValueAsDouble();
    inputs.extensionSupplyCurrent = extensionSupplyCurrent.getValueAsDouble();
    inputs.extensionSpeed = extensionMotor.getVelocity().getValueAsDouble();
    inputs.extensionVoltage = extensionMotor.getMotorVoltage().getValueAsDouble();
    inputs.extensionSetpoint = extensionSetpoint;

    extensionAlert.set(!inputs.extensionMotorConnected);
    extensionEncoderAlert.set(!inputs.extensionEncoderConnected);

  }


   @Override
  public void setExtensionSetpoint(double position) {
    extensionSetpoint = position;
    extensionMotor.setControl(m_request_extension.withPosition(extensionSetpoint));
  }
}
