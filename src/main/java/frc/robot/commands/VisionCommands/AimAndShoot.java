package frc.robot.commands.VisionCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.LimelightConstants;
import frc.robot.commands.FuelCommands.LaunchSequence;
import frc.robot.commands.FuelCommands.SpinUp;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.CANFuelSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class AimAndShoot extends SequentialCommandGroup{

    public AimAndShoot(CANDriveSubsystem drivetrain, CANFuelSubsystem fuel, VisionSubsystem vision) {
        addCommands(
            new ParallelRaceGroup(
                new AutoAim(vision, drivetrain),
                new SpinUp(fuel)
            ).withTimeout(LimelightConstants.visionAimTimeout),
            new ParallelCommandGroup(
                new AutoAim(vision, drivetrain),
                new LaunchSequence(fuel)
            )
        );
    }
    
}
