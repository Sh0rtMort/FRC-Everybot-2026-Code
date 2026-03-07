package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.CANFuelSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class VisionAimAndShoot extends SequentialCommandGroup{

    public VisionAimAndShoot(CANFuelSubsystem shooter, CANDriveSubsystem drivetrain, VisionSubsystem vision) {
        addCommands(
            new VisionAim(vision, drivetrain).withTimeout(2),
            new LaunchSequence(shooter)
        );

    }
    
}
