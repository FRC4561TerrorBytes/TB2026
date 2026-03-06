package frc.robot.subsystems.indexer;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IndexerIOSim implements IndexerIO {

  private static final double LOOP_PERIOD_SECS = 0.02;

  private double leftIndexerAppliedVolts = 0.0;
  private double rightIndexerAppliedVolts = 0.0;
  private double kickerAppliedVolts = 0.0;

  private DCMotorSim leftIndexerMotor =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX44(1), 0.01, 1.0),
          DCMotor.getKrakenX44(1));

  private DCMotorSim rightIndexerMotor =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX44(1), 0.01, 1.0),
          DCMotor.getKrakenX44(1));

  private DCMotorSim kickerMotor =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0.01, 1.0),
          DCMotor.getKrakenX60(1));

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    leftIndexerMotor.update(LOOP_PERIOD_SECS);
    rightIndexerMotor.update(LOOP_PERIOD_SECS);
    kickerMotor.update(LOOP_PERIOD_SECS);
  }

  public void setThroughput(double indexerSpeed, double kickerSpeed) {
    leftIndexerAppliedVolts = MathUtil.clamp(indexerSpeed * 12, -12, 12);
    leftIndexerMotor.setInputVoltage(leftIndexerAppliedVolts);
    rightIndexerAppliedVolts = MathUtil.clamp(-indexerSpeed * 12, -12, 12);
    rightIndexerMotor.setInputVoltage(rightIndexerAppliedVolts);
    kickerAppliedVolts = MathUtil.clamp(kickerSpeed * 12, -12, 12);
    kickerMotor.setInputVoltage(kickerAppliedVolts);
  }
}