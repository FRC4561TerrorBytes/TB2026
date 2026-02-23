package frc.robot.subsystems.shooter;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ShooterIOSim implements ShooterIO{
private static final double LOOP_PERIOD_SECS = 0.02;

  private double flywheelRightAppliedVolts = 0.0;
  private double hoodAppliedVolts = 0.0;
  private double flywheelLeftAppliedVolts = 0.0;

  private DCMotorSim flywheelRightMotor =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX44(2), 0.01, 1.0),
          DCMotor.getKrakenX44(2));

  private DCMotorSim hoodMotor =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0.01, 1.0),
          DCMotor.getKrakenX60(1));

  private DCMotorSim flywheelLeftMotor =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX44(2), 0.01, 1.0),
          DCMotor.getKrakenX44(2));

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    flywheelRightMotor.update(LOOP_PERIOD_SECS);
    hoodMotor.update(LOOP_PERIOD_SECS);
    flywheelLeftMotor.update(LOOP_PERIOD_SECS);
  }

  public void setflywheelLeftVoltage(double voltage) {
    flywheelLeftMotor.setInputVoltage(voltage);
    this.flywheelLeftAppliedVolts = voltage;
  }
  public void setFlywheelRightVoltage(double voltage) {
    flywheelRightMotor.setInputVoltage(voltage);
    this.flywheelRightAppliedVolts = voltage;
  }
    public void setHoodPosition(double voltage) {
    hoodMotor.setInputVoltage(voltage);
    this.hoodAppliedVolts = voltage;
}
}
