// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.extension;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/** Add your docs here. */
public class ExtensionIOSim implements ExtensionIO{
    private static final double LOOP_PERIOD_SECS = 0.02;

    private static final double EXTENSION_KP = 1.8;
    private static final double EXTENSION_KD = 0;

    private static final DCMotor EXTENSION_MOTOR = DCMotor.getKrakenX60(1);

    private DCMotorSim extensionMotorSim;
    
    private boolean closedLoop = true;
    private ProfiledPIDController extensionController = 
        new ProfiledPIDController(EXTENSION_KP, 0.0, EXTENSION_KD, new Constraints(3, 3));
    
    private double extensionSetpoint;
    private double extensionAppliedVolts;

    public ExtensionIOSim() {
        extensionMotorSim = 
            new DCMotorSim(LinearSystemId.createDCMotorSystem(EXTENSION_MOTOR, 0.000001, 1), EXTENSION_MOTOR);
    }

     @Override
  public void updateInputs(ExtensionIOInputs inputs) {
    if (closedLoop) {
      extensionAppliedVolts = extensionController.calculate(inputs.extensionAngle, extensionSetpoint);
    }

    extensionMotorSim.setInputVoltage(MathUtil.clamp(extensionAppliedVolts, -12.0, 12.0));
    extensionMotorSim.update(LOOP_PERIOD_SECS);

    inputs.extensionAngle = extensionMotorSim.getAngularPositionRotations() / 1.0;
    inputs.extensionMotorConnected = true;
    inputs.extensionSetpoint = this.extensionSetpoint;
    inputs.extensionVoltage = this.extensionAppliedVolts;
  }

  @Override
  public void setExtensionSetpoint(double position) {
    closedLoop = true;
    this.extensionSetpoint = Units.degreesToRotations(position);
  }
}
