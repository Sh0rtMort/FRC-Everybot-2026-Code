package frc.robot.commands.VisionCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANDriveSubsystem;

public class AimAtPoint extends Command{

    private final CANDriveSubsystem drive;
    private Translation2d target;
    private final DoubleSupplier forward;

    private double fieldWidth = 8.07;

    private Translation2d redHub = new Translation2d(12.51, fieldWidth/2);
    private Translation2d blueHub = new Translation2d(4.03, fieldWidth/2);

    public AimAtPoint(CANDriveSubsystem drive, DoubleSupplier forward) {
        this.drive = drive;
        // this.target = target;
        this.forward = forward;

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        target = getHub();
        
    }

    @Override
    public void execute() {
        drive.aimAtPoint(target, forward.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        drive.driveArcade(0, 0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    Translation2d getHub() {
    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
        return redHub;
    }
        return blueHub;
}
    
}
