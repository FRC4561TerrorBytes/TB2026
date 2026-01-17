// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.Constants;

/** Add your docs here. */
public class IntakeIOReal implements IntakeIO{
    private TalonFX intakeMotor = new TalonFX(Constants.INTAKE_ID);
  private CANcoder extensionEncoder = new CANcoder(Constants.EXTENSION_CANCODER_ID);
  private TalonFX extensionMotor = new TalonFX(Constants.EXTENSION_ID);

  private final StatusSignal<Current> intakeStatorCurrent;
  private final StatusSignal<Current> intakeSupplyCurrent;
  private final StatusSignal<AngularVelocity> intakeSpeed;
  private final StatusSignal<Voltage> intakeVoltage;
  private final StatusSignal<Temperature> intakeTemp;   

 private final StatusSignal<Angle> extensionAngle;
  private final StatusSignal<Current> extensionStatorCurrent;
  private final StatusSignal<Current> extensionSupplyCurrent;
  private final StatusSignal<AngularVelocity> extensionSpeed;
  private final StatusSignal<Voltage> extensionVoltage;
  private final StatusSignal<Temperature> extensionTemp;

  private double extensionSetpoint = 0.0;
  private double extensionFeedForward = 0.0;

  private final MotionMagicVoltage m_request_extension = new MotionMagicVoltage(0);

  private final Alert intakeAlert = new Alert("Extension Disconnected.", AlertType.kWarning);
  private final Alert intakeEncoderAlert =
      new Alert("Extension Encoder Disconnected.", AlertType.kWarning);

  public IntakeIOReal() {
    var ExtensionPIDConfig = new Slot0Configs();
    ExtensionPIDConfig.GravityType = GravityTypeValue.Arm_Cosine;
    // pivotPIDConfig.kS = 0.28;
    ExtensionPIDConfig.kV = 0;
    ExtensionPIDConfig.kA = 0;
    ExtensionPIDConfig.kP = 75; // 75
    ExtensionPIDConfig.kI = 0;
    ExtensionPIDConfig.kD = 0;

    var cancoderConfig = new CANcoderConfiguration();
    cancoderConfig.MagnetSensor.withMagnetOffset(0);
    cancoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    tryUntilOk(5, () -> extensionEncoder.getConfigurator().apply(cancoderConfig, 0.25));

    var extensionConfig = new TalonFXConfiguration();
    extensionConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    extensionConfig.Slot0 = ExtensionPIDConfig;
    extensionConfig.Feedback.RotorToSensorRatio = Constants.EXTENSION_GEAR_RATIO;
    extensionConfig.Feedback.FeedbackRemoteSensorID = extensionEncoder.getDeviceID();
    extensionConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    extensionConfig.MotionMagic.MotionMagicCruiseVelocity = 100 / Constants.EXTENSION_GEAR_RATIO;
    extensionConfig.MotionMagic.MotionMagicAcceleration =
        extensionConfig.MotionMagic.MotionMagicCruiseVelocity / 0.050;
    extensionConfig.MotionMagic.MotionMagicExpo_kV = 0.12 * Constants.EXTENSION_GEAR_RATIO;
    extensionConfig.MotionMagic.MotionMagicExpo_kA = 0.1;
    extensionConfig.ClosedLoopGeneral.ContinuousWrap = false;
    extensionConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    extensionConfig.CurrentLimits.StatorCurrentLimit = Constants.EXTENSION_STATOR_CURRENT_LIMIT;
    extensionConfig.CurrentLimits.SupplyCurrentLimit = Constants.EXTENSION_SUPPLY_CURRENT_LIMIT;
    extensionConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    extensionConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
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


//dont change stuff below


     var intakeConfig = new TalonFXConfiguration();
    intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    intakeConfig.Feedback.RotorToSensorRatio = Constants.INTAKE_GEAR_RATIO;
    intakeConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    intakeConfig.ClosedLoopGeneral.ContinuousWrap = false;
    intakeConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    intakeConfig.CurrentLimits.StatorCurrentLimit = Constants.INTAKE_STATOR_CURRENT_LIMIT;
    intakeConfig.CurrentLimits.SupplyCurrentLimit = Constants.INTAKE_SUPPLY_CURRENT_LIMIT;
    intakeConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    intakeConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    tryUntilOk(5, () -> intakeMotor.getConfigurator().apply(intakeConfig, 0.25));

    intakeStatorCurrent = intakeMotor.getStatorCurrent();
    intakeSupplyCurrent = intakeMotor.getSupplyCurrent();
    intakeSpeed = intakeMotor.getVelocity();
    intakeVoltage = intakeMotor.getMotorVoltage();
    intakeTemp = intakeMotor.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        intakeStatorCurrent,
        intakeSupplyCurrent,
        intakeSpeed,
        intakeVoltage,
        intakeTemp);

    ParentDevice.optimizeBusUtilizationForAll(intakeMotor);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    var IntakeStatus =
        BaseStatusSignal.refreshAll(
            intakeStatorCurrent,
            intakeSupplyCurrent,
            intakeSpeed,
            intakeVoltage,
            intakeTemp);


    inputs.intakeMotorConnected = IntakeStatus.isOK();
    inputs.intakeStatorCurrent = intakeStatorCurrent.getValueAsDouble();
    inputs.intakeSupplyCurrent = intakeSupplyCurrent.getValueAsDouble();
    inputs.intakeSpeed = intakeMotor.getVelocity().getValueAsDouble();
    inputs.intakeVoltage = intakeMotor.getMotorVoltage().getValueAsDouble();
    inputs.extensionSetpoint = extensionSetpoint;

    intakeEncoderAlert.set(!inputs.intakeEncoderConnected);
    intakeAlert.set(!inputs.intakeMotorConnected);
  }

  @Override
  public void setExtensionSetpoint(double position) {
    extensionSetpoint = Units.degreesToRotations(position);
    intakeMotor.setControl(m_request_extension.withPosition(extensionSetpoint));
  }

  @Override
  public void setOutput(double speed) {
    intakeMotor.set(speed);
  }

}
