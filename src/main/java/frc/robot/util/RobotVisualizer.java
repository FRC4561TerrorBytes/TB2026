package frc.robot.util;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.subsystems.extension.*;
import frc.robot.subsystems.shooter.*;

import org.littletonrobotics.junction.Logger;

public class RobotVisualizer {
  private static Extension extension;
  private static Shooter shooter;

  public static void initialize(Extension extension, Shooter shooter) {
    RobotVisualizer.extension = extension;
    RobotVisualizer.shooter = shooter;
  }

  public static void update() {
    Logger.recordOutput(
        "FinalComponentPoses",
        new Pose3d[] {
          extension.getPivotPose(),
          new Pose3d(-0.275,0,0.5, new Rotation3d(0,0,0)),
          shooter.getHoodPose()
        });
  }
}