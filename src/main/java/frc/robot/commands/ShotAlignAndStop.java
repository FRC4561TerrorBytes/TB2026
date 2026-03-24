package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

public class ShotAlignAndStop extends Command {

    public Drive drive;
    public double distanceToHub;
    public boolean robotMoving;
    public double targetAngle;
    PIDController controller;
    public ShotAlignAndStop(Drive drive) {
        this.drive = drive;
        addRequirements(drive);
        controller = new PIDController(0.1, 0, 0, 0.02);
        controller.setTolerance(1.5);
        controller.enableContinuousInput(-180, 180);
        robotMoving = Math.abs(drive.getChassisSpeeds().vxMetersPerSecond) < 0.1 && Math.abs(drive.getChassisSpeeds().vyMetersPerSecond) < 0.1;
    }

    @Override
    public void execute() {
        double targetAngle = drive.getRotationToHub().getDegrees();
        Logger.recordOutput("TargetAngleToFace", targetAngle);
        double rotationSpeed = MathUtil.clamp(controller.calculate(drive.getPose().getRotation().getDegrees(), targetAngle), -30, 30);
        if(controller.atSetpoint()){
            drive.stopWithX();
        }
        else if(!controller.atSetpoint() && !robotMoving){
            drive.runVelocity(new ChassisSpeeds(0, 0, rotationSpeed));
            robotMoving = false;
        }

        Logger.recordOutput("DriveTrainFacingHub", controller.atSetpoint());
 }

    @Override
    public void end(boolean interrupted) {
    }


    @Override
    public boolean isFinished() {
        return controller.atSetpoint();
    }
}
