// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

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

public class ElevatorIOReal implements ElevatorIO {
  private TalonFX elevatorMotor1 = new TalonFX(Constants.ELEVATOR_1_ID);
  private TalonFX elevatorMotor2 = new TalonFX(Constants.ELEVATOR_2_ID);
  private CANcoder elevatorEncoder1 = new CANcoder(Constants.ELEVATOR_CANCODER_1_ID);
  private CANcoder elevatorEncoder2 = new CANcoder(Constants.ELEVATOR_CANCODER_2_ID);

  private final StatusSignal<Current> elevator1StatorCurrent;
  private final StatusSignal<Current> elevator1SupplyCurrent;
  private final StatusSignal<AngularVelocity> elevator1Speed;
  private final StatusSignal<Voltage> elevator1Voltage;
  private final StatusSignal<Temperature> elevator1Temp;
  private final StatusSignal<Angle> cancoder1Position;
  private final StatusSignal<Angle> cancoder2Position;

  private final StatusSignal<Current> elevator2StatorCurrent;
  private final StatusSignal<Current> elevator2SupplyCurrent;
  private final StatusSignal<AngularVelocity> elevator2Speed;
  private final StatusSignal<Voltage> elevator2Voltage;
  private final StatusSignal<Temperature> elevator2Temp;

  private double elevatorSetpoint = 0.0;
  private final MotionMagicVoltage m_request_elevator = new MotionMagicVoltage(0);
  
  private final Alert elevator1Alert = new Alert("Elevator Motor 1 Disconnected.", AlertType.kWarning);
  private final Alert elevator1EncoderAlert = new Alert("Elevator Encoder 1 Disconnected.", AlertType.kWarning);
  private final Alert elevator2Alert = new Alert("Elevator Motor 2 Disconnected.", AlertType.kWarning);
  private final Alert elevator2EncoderAlert = new Alert("Elevator Encoder 2 Disconnected.", AlertType.kWarning);

  public ElevatorIOReal() {
    var ElevatorPIDConfig = new Slot0Configs();
    ElevatorPIDConfig.GravityType = GravityTypeValue.Elevator_Static;
    ElevatorPIDConfig.kV = 0;
    ElevatorPIDConfig.kA = 0;
    ElevatorPIDConfig.kP = 75; 
    ElevatorPIDConfig.kI = 0;
    ElevatorPIDConfig.kD = 0;

    var cancoderConfig = new CANcoderConfiguration();
    cancoderConfig.MagnetSensor.withMagnetOffset(0);
    cancoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    tryUntilOk(5, () -> elevatorEncoder1.getConfigurator().apply(cancoderConfig, 0.25));
    tryUntilOk(5, () -> elevatorEncoder2.getConfigurator().apply(cancoderConfig, 0.25));

    var elevatorConfig = new TalonFXConfiguration();
    elevatorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    elevatorConfig.Slot0 = ElevatorPIDConfig;
    
    // Configured to use the native Rotor Sensor for high-speed tracking
    elevatorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    
    elevatorConfig.MotionMagic.MotionMagicCruiseVelocity = 100;
    elevatorConfig.MotionMagic.MotionMagicAcceleration = elevatorConfig.MotionMagic.MotionMagicCruiseVelocity / 0.050;
    elevatorConfig.MotionMagic.MotionMagicExpo_kV = 0.12;
    elevatorConfig.MotionMagic.MotionMagicExpo_kA = 0.1;
    elevatorConfig.ClosedLoopGeneral.ContinuousWrap = false;
    elevatorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    elevatorConfig.CurrentLimits.StatorCurrentLimit = Constants.ELEVATOR_STATOR_CURRENT_LIMIT;
    elevatorConfig.CurrentLimits.SupplyCurrentLimit = Constants.ELEVATOR_SUPPLY_CURRENT_LIMIT;
    elevatorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    elevatorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    
    tryUntilOk(5, () -> elevatorMotor1.getConfigurator().apply(elevatorConfig, 0.25));
    tryUntilOk(5, () -> elevatorMotor2.getConfigurator().apply(elevatorConfig, 0.25));

    cancoder1Position = elevatorEncoder1.getPosition();
    cancoder2Position = elevatorEncoder2.getPosition();

    elevator1StatorCurrent = elevatorMotor1.getStatorCurrent();
    elevator1SupplyCurrent = elevatorMotor1.getSupplyCurrent();
    elevator1Speed = elevatorMotor1.getVelocity();
    elevator1Voltage = elevatorMotor1.getMotorVoltage();
    elevator1Temp = elevatorMotor1.getDeviceTemp();

    elevator2StatorCurrent = elevatorMotor2.getStatorCurrent();
    elevator2SupplyCurrent = elevatorMotor2.getSupplyCurrent();
    elevator2Speed = elevatorMotor2.getVelocity();
    elevator2Voltage = elevatorMotor2.getMotorVoltage();
    elevator2Temp = elevatorMotor2.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        cancoder1Position,
        cancoder2Position,
        elevator1StatorCurrent,
        elevator1SupplyCurrent,
        elevator1Speed,
        elevator1Voltage,
        elevator1Temp,
        elevator2StatorCurrent,
        elevator2SupplyCurrent,
        elevator2Speed,
        elevator2Voltage,
        elevator2Temp
    );

    ParentDevice.optimizeBusUtilizationForAll(elevatorMotor1, elevatorMotor2);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        cancoder1Position, cancoder2Position,
        elevator1StatorCurrent, elevator1SupplyCurrent, elevator1Speed, elevator1Voltage, elevator1Temp,
        elevator2StatorCurrent, elevator2SupplyCurrent, elevator2Speed, elevator2Voltage, elevator2Temp
    );

    inputs.cancoder1Deg = Units.rotationsToDegrees(cancoder1Position.getValueAsDouble());
    inputs.cancoder2Deg = Units.rotationsToDegrees(cancoder2Position.getValueAsDouble());

    inputs.elevatorSetpoint = elevatorSetpoint;
    inputs.elevatorMotor1Connected = elevator1StatorCurrent.getStatus().isOK();
    inputs.elevatorMotor2Connected = elevator2StatorCurrent.getStatus().isOK();
    inputs.elevatorMotorConnected = inputs.elevatorMotor1Connected && inputs.elevatorMotor2Connected;

    elevator1Alert.set(!inputs.elevatorMotor1Connected);
    elevator2Alert.set(!inputs.elevatorMotor2Connected);
    elevator1EncoderAlert.set(!cancoder1Position.getStatus().isOK());
    elevator2EncoderAlert.set(!cancoder2Position.getStatus().isOK());
  }

  @Override
  public void updateAbsolutePosition(double absoluteInches) {

    double physicalRotations = absoluteInches * Constants.ELEVATOR_GEAR_RATIO;

    elevatorMotor1.setPosition(physicalRotations);
    elevatorMotor2.setPosition(physicalRotations);
  }

  @Override
  public void setElevatorSetpoint(double positionInches) {
    this.elevatorSetpoint = positionInches;

    double targetRotations = positionInches * Constants.ELEVATOR_GEAR_RATIO;
    
    elevatorMotor1.setControl(m_request_elevator.withPosition(targetRotations));
    elevatorMotor2.setControl(m_request_elevator.withPosition(targetRotations));
  }
}