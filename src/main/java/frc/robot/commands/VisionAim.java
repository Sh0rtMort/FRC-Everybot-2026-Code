package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class VisionAim extends Command{
    private VisionSubsystem vision;
    private CANDriveSubsystem drivetrain;

    private PIDController visionPID = new PIDController(0, 0, 0);
    
    public VisionAim(VisionSubsystem vision, CANDriveSubsystem drivetrain) {
        this.vision = vision;
        this.drivetrain = drivetrain;
        addRequirements(vision);

        visionPID.setTolerance(0.5);
    }

    @Override
    public void initialize() {
        visionPID.reset();
    }

    @Override
    public void execute() {
        //theoretically the center of the april tag
        double setPoint = 0;
        double speed = visionPID.calculate(vision.getTXValue(), setPoint);

        drivetrain.driveArcade(0, speed);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.driveArcade(0, 0);        
    }

    @Override
    public boolean isFinished() {
        return isFinished();
    }
}
