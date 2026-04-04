// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.Constants;

/** Add your docs here. */
public class IntakeIOReal implements IntakeIO{
    private TalonFX intakeMotorLeft = new TalonFX(Constants.INTAKE_ID_LEFT);
    private TalonFX intakeMotorRight = new TalonFX(Constants.INTAKE_ID_RIGHT);

  private final StatusSignal<Current> intakeLeftStatorCurrent;
  private final StatusSignal<Current> intakeLeftSupplyCurrent;
  private final StatusSignal<AngularVelocity> intakeLeftSpeed;
  private final StatusSignal<Voltage> intakeLeftVoltage;
  private final StatusSignal<Temperature> intakeLeftTemp;
  
  private final StatusSignal<Current> intakeRightStatorCurrent;
  private final StatusSignal<Current> intakeRightSupplyCurrent;
  private final StatusSignal<AngularVelocity> intakeRightSpeed;
  private final StatusSignal<Voltage> intakeRightVoltage;
  private final StatusSignal<Temperature> intakeRightTemp;  

 

  private final Alert intakeLeftAlert = new Alert("Intake Disconnected.", AlertType.kWarning);
  private final Alert intakeRightAlert = new Alert("Intake Right Disconnected", AlertType.kError);
  

  public IntakeIOReal() {


     var intakeConfig = new TalonFXConfiguration();
    intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    intakeConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    intakeConfig.CurrentLimits.StatorCurrentLimit = Constants.INTAKE_STATOR_CURRENT_LIMIT;
    intakeConfig.CurrentLimits.SupplyCurrentLimit = Constants.INTAKE_SUPPLY_CURRENT_LIMIT;
    intakeConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    intakeConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    tryUntilOk(5, () -> intakeMotorLeft.getConfigurator().apply(intakeConfig, 0.25));
    tryUntilOk(5, () -> intakeMotorRight.getConfigurator().apply(intakeConfig, 0.25));

    intakeLeftStatorCurrent = intakeMotorLeft.getStatorCurrent();
    intakeLeftSupplyCurrent = intakeMotorLeft.getSupplyCurrent();
    intakeLeftSpeed = intakeMotorLeft.getVelocity();
    intakeLeftVoltage = intakeMotorLeft.getMotorVoltage();
    intakeLeftTemp = intakeMotorLeft.getDeviceTemp();

    intakeRightStatorCurrent = intakeMotorRight.getStatorCurrent();
    intakeRightSupplyCurrent = intakeMotorRight.getSupplyCurrent();
    intakeRightSpeed = intakeMotorRight.getVelocity();
    intakeRightVoltage = intakeMotorRight.getMotorVoltage();
    intakeRightTemp = intakeMotorRight.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        intakeLeftStatorCurrent,
        intakeLeftSupplyCurrent,
        intakeLeftSpeed,
        intakeLeftVoltage,
        intakeLeftTemp,

        intakeRightStatorCurrent,
        intakeRightSupplyCurrent,
        intakeRightSpeed,
        intakeRightVoltage,
        intakeRightTemp);

    ParentDevice.optimizeBusUtilizationForAll(intakeMotorLeft);
    ParentDevice.optimizeBusUtilizationForAll(intakeMotorRight);

    intakeMotorLeft.setControl(new Follower(intakeMotorRight.getDeviceID(), MotorAlignmentValue.Opposed));
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    var IntakeLeftStatus =
        BaseStatusSignal.refreshAll(
            intakeLeftStatorCurrent,
            intakeLeftSupplyCurrent,
            intakeLeftSpeed,
            intakeLeftVoltage,
            intakeLeftTemp);

      var IntakeRightStatus = 
        BaseStatusSignal.refreshAll(
            intakeRightStatorCurrent,
        intakeRightSupplyCurrent,
        intakeRightSpeed,
        intakeRightVoltage,
        intakeRightTemp);

    inputs.intakeLeftMotorConnected = IntakeLeftStatus.isOK();
    inputs.intakeLeftStatorCurrent = intakeLeftStatorCurrent.getValueAsDouble();
    inputs.intakeLeftSupplyCurrent = intakeLeftSupplyCurrent.getValueAsDouble();
    inputs.intakeLeftSpeed = intakeMotorLeft.getVelocity().getValueAsDouble();
    inputs.intakeLeftVoltage = intakeMotorLeft.getMotorVoltage().getValueAsDouble();

    inputs.intakeRightMotorConnected = IntakeRightStatus.isOK();
    inputs.intakeRightStatorCurrent = intakeRightStatorCurrent.getValueAsDouble();
    inputs.intakeRightSupplyCurrent = intakeRightSupplyCurrent.getValueAsDouble();
    inputs.intakeRightSpeed = intakeMotorRight.getVelocity().getValueAsDouble();
    inputs.intakeRightVoltage = intakeMotorRight.getMotorVoltage().getValueAsDouble();

    intakeLeftAlert.set(!inputs.intakeLeftMotorConnected);
    intakeRightAlert.set(!inputs.intakeRightMotorConnected);
  }

 

  @Override
  public void setOutput(double speed) {
    intakeMotorRight.set(speed);
  }

}
