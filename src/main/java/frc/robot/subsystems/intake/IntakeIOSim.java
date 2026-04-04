// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/** Add your docs here. */
public class IntakeIOSim implements IntakeIO{
    private static final double LOOP_PERIOD_SECS = 0.02;
    
    private double intakeAppliedVolts = 0.0;

    private DCMotorSim intakeLeftMotor = 
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX44(1), 0.01, 1.0),
          DCMotor.getKrakenX44(1));
    private DCMotorSim intakeRightMotor = 
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX44(1), 0.01, 1.0),
          DCMotor.getKrakenX44(1));

    @Override
    public void updateInputs(IntakeIOInputs inputs){
        intakeLeftMotor.update(LOOP_PERIOD_SECS);
        inputs.intakeLeftSpeed = Units.radiansToDegrees(intakeLeftMotor.getAngularVelocityRadPerSec());
        inputs.intakeLeftVoltage = intakeAppliedVolts;
        inputs.intakeLeftStatorCurrent = Math.abs(intakeLeftMotor.getCurrentDrawAmps());
        
        intakeRightMotor.update(LOOP_PERIOD_SECS);
        inputs.intakeRightSpeed = Units.radiansToDegrees(intakeRightMotor.getAngularVelocityRadPerSec());
        inputs.intakeRightVoltage = intakeAppliedVolts;
        inputs.intakeRightStatorCurrent = Math.abs(intakeRightMotor.getCurrentDrawAmps());
        inputs.intakeLeftMotorConnected = true;
        inputs.intakeRightMotorConnected = true;

    }
    public void setOutput(double speed){
        intakeAppliedVolts = MathUtil.clamp(speed * 12, -12, 12);
        intakeLeftMotor.setInputVoltage(intakeAppliedVolts);
        intakeRightMotor.setInputVoltage(intakeAppliedVolts);
    }
}
