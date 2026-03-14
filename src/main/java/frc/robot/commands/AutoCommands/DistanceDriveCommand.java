package frc.robot.commands.AutoCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANDriveSubsystem;

public class DistanceDriveCommand extends Command{

    private CANDriveSubsystem drivetrain;

    private PIDController pidController;

    private double distanceTargetMeters;

    public DistanceDriveCommand(CANDriveSubsystem drivetrain, double distanceTargetMeters) {
        this.drivetrain = drivetrain;
        this.distanceTargetMeters = distanceTargetMeters;

        pidController = new PIDController(0, 0, 0); //forward/back PID constants
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        double speed = pidController.calculate(drivetrain.getEncoderMeters(), distanceTargetMeters);

        drivetrain.driveArcade(speed, 0);

        SmartDashboard.putNumber("Encoder Distance Meter", drivetrain.getEncoderMeters());
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.driveArcade(0, 0);
    }

    @Override
    public boolean isFinished() {
        return pidController.atSetpoint();
    }
    
}
