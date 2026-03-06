// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
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
    private TalonFX intakeMotor = new TalonFX(Constants.INTAKE_ID);

  private final StatusSignal<Current> intakeStatorCurrent;
  private final StatusSignal<Current> intakeSupplyCurrent;
  private final StatusSignal<AngularVelocity> intakeSpeed;
  private final StatusSignal<Voltage> intakeVoltage;
  private final StatusSignal<Temperature> intakeTemp;   

 

  private final Alert intakeAlert = new Alert("Intake Disconnected.", AlertType.kWarning);
  

  public IntakeIOReal() {


     var intakeConfig = new TalonFXConfiguration();
    intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
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

    intakeAlert.set(!inputs.intakeMotorConnected);
  }

 

  @Override
  public void setOutput(double speed) {
    intakeMotor.set(speed);
  }

}
