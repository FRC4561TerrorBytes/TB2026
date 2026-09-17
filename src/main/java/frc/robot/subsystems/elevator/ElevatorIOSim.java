package frc.robot.subsystems.elevator;

import com.google.gson.internal.TroubleshootingGuide;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/** Add your docs here. */
public class ElevatorIOSim implements ElevatorIO{
    private static final double LOOP_PERIOD_SECS = 0.02;

    private static final double ELEVATOR_KP = 1.8;
    private static final double ELEVATOR_KD = 0;

    private static final DCMotor ELEVATOR_MOTOR = DCMotor.getKrakenX60(1);

    private DCMotorSim elevatorMotorSim;
    
    private boolean closedLoop = true;
    private ProfiledPIDController elevatorController = 
        new ProfiledPIDController(ELEVATOR_KP, 0.0, ELEVATOR_KD, new Constraints(3, 3));
    
    private double elevatorSetpoint;
    private double elevatorAppliedVolts;

    public ElevatorIOSim() {
        elevatorMotorSim = 
            new DCMotorSim(LinearSystemId.createDCMotorSystem(ELEVATOR_MOTOR, 0.000001, 1), ELEVATOR_MOTOR);
    }

     @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    if (closedLoop) {
      elevatorAppliedVolts = elevatorController.calculate(inputs.elevatorAngle, elevatorSetpoint);
    }

    inputs.elevatorMotorConnected = true;

    elevatorMotorSim.setInputVoltage(MathUtil.clamp(elevatorAppliedVolts, -12.0, 12.0));
    elevatorMotorSim.update(LOOP_PERIOD_SECS);

    inputs.elevatorAngle = elevatorMotorSim.getAngularPositionRotations() / 1.0;
    inputs.elevatorSetpoint = this.elevatorSetpoint;
    inputs.elevatorVoltage = this.elevatorAppliedVolts;
  }

  @Override
  public void setElevatorSetpoint(double position) {
    closedLoop = true;
    this.elevatorSetpoint = (position);
  }
}