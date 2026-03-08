// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DropSet extends Command {

  Command test;
  Command backUp;
  int index;
  boolean backUpMode;
  double[] times = new double[4];
  double lastTime;
  int count = 0;
  double timeSince;
  /** Creates a new dropSet. */
  public DropSet(Command test, Command backUp) {
    this.test = test;
    this.backUp = backUp;
    index = 0;
    backUpMode = false;
    lastTime = System.currentTimeMillis();
    timeSince = System.currentTimeMillis();
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    times[index] = System.currentTimeMillis() - lastTime;

    if(count >= 4){
      double sum = 0;
      for(int i = 0; i < times.length; i++){
        sum += times[i];
      }

      if(sum < 1000 && System.currentTimeMillis() - timeSince >= 3000){
      backUpMode = !backUpMode;
      timeSince = System.currentTimeMillis();
      }
    }

    Logger.recordOutput("Dropped-"+test.getName(), backUpMode);

    lastTime = System.currentTimeMillis();

    index++;
    if(index >= times.length)
      index = 0;
    
    count++;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!backUpMode){
      test.schedule();
    } else {
      backUp.schedule();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}