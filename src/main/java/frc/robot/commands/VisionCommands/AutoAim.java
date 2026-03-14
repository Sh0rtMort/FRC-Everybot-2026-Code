package frc.robot.commands.VisionCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class AutoAim extends Command{

    private CANDriveSubsystem drivetrain;
    private VisionSubsystem vision;
    private PIDController pidController;

    private double tolerance = 5;
    
    public AutoAim(VisionSubsystem vision, CANDriveSubsystem drivetrain) {
        this.drivetrain = drivetrain;
        this.vision = vision;

        addRequirements(vision, drivetrain);

        pidController = new PIDController(0, 0, 0); //these need to be tuned
        pidController.setTolerance(tolerance); //this unit becomes whatever the unit the limelight is in
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        //theoretically the center of the april tag
        double setPoint = 0;
        //make a speed variable that changes as the vision measurments change
        double speed = pidController.calculate(vision.getTXValue(), setPoint);
        //then apply the variable to the drive command
        drivetrain.driveArcade(0, speed);
        //then the PID does the rest, assuming you have the proper values tuned, then the calculator does the math to give the 
        //proper motor inputs 
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.driveArcade(0, 0);
    }

    @Override
    public boolean isFinished() {
        return (vision.getTXValue() <= tolerance);
    }
}
