package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.LimelightConstants;
import frc.robot.commands.FuelCommands.Launch;
import frc.robot.commands.FuelCommands.LaunchSequence;
import frc.robot.commands.VisionCommands.AutoAim;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.CANFuelSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class SecondExample extends SequentialCommandGroup{

    public SecondExample(CANDriveSubsystem drivetrain, CANFuelSubsystem fuel, VisionSubsystem vision) {
        addCommands(
            new DistanceDriveCommand(drivetrain, -0.4), //negative to move in reverse, 0.4 represents the desired shooting zone
            new AutoAim(vision, drivetrain),
            new LaunchSequence(fuel)
        );
    }
}
