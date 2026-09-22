package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.google.gson.internal.TroubleshootingGuide;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

/** Add your docs here. */
public class ElevatorIOSim implements ElevatorIO{
    private static final double LOOP_PERIOD_SECS = 0.02;

    private static final double ELEVATOR_KP = 1.8;
    private static final double ELEVATOR_KD = 0;

    private static final DCMotor ELEVATOR_MOTOR1 = DCMotor.getKrakenX60(1);
    private static final DCMotor ELEVATOR_MOTOR2 = DCMotor.getKrakenX60(1);

    private static final CANcoder CANCODER1 = new CANcoder(Constants.ELEVATOR_CANCODER_1_ID);
    private static final CANcoder CANCODER2 = new CANcoder(Constants.ELEVATOR_CANCODER_2_ID);

    private DCMotorSim elevator1MotorSim;
    private DCMotorSim elevator2MotorSim;
    private CANcoderSimState cancoder1Sim;
    private CANcoderSimState cancoder2Sim;
    
    private boolean closedLoop = true;
    private ProfiledPIDController elevatorController = 
        new ProfiledPIDController(ELEVATOR_KP, 0.0, ELEVATOR_KD, new Constraints(3, 3));
    
    private double elevatorSetpoint;
    private double elevatorAppliedVolts;

    public ElevatorIOSim() {
        elevator1MotorSim = 
            new DCMotorSim(LinearSystemId.createDCMotorSystem(ELEVATOR_MOTOR1, 0.000001, 1), ELEVATOR_MOTOR1);
        elevator2MotorSim = 
            new DCMotorSim(LinearSystemId.createDCMotorSystem(ELEVATOR_MOTOR2, 0.000001, 1), ELEVATOR_MOTOR2);
        cancoder1Sim =
            new CANcoderSimState(CANCODER1);
        cancoder2Sim =
            new CANcoderSimState(CANCODER2);
    }

     @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    if (closedLoop) {
      elevatorAppliedVolts = elevatorController.calculate(inputs.elevatorAngle, elevatorSetpoint);
    }

    inputs.elevatorMotorConnected = true;

    elevator1MotorSim.setInputVoltage(MathUtil.clamp(elevatorAppliedVolts, -12.0, 12.0));
    elevator1MotorSim.update(LOOP_PERIOD_SECS);
    elevator2MotorSim.setInputVoltage(MathUtil.clamp(elevatorAppliedVolts, -12.0, 12.0));
    elevator2MotorSim.update(LOOP_PERIOD_SECS);

    inputs.elevatorAngle = elevator1MotorSim.getAngularPositionRotations() / 1.0;
    inputs.elevatorSetpoint = this.elevatorSetpoint;
    inputs.elevatorVoltage = this.elevatorAppliedVolts;
  }

  @Override
  public void setElevatorSetpoint(double position) {
    closedLoop = true;
    this.elevatorSetpoint = (position);
  }
}