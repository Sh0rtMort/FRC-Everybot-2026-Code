// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import static frc.robot.Constants.OperatorConstants.*;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import frc.robot.commands.Drive;
import frc.robot.commands.AutoCommands.ExampleAuto;
import frc.robot.commands.AutoCommands.SecondExample;
import frc.robot.commands.ClimberCommands.ClimbDown;
import frc.robot.commands.ClimberCommands.ClimbUp;
import frc.robot.commands.FuelCommands.Eject;
import frc.robot.commands.FuelCommands.Intake;
import frc.robot.commands.FuelCommands.LaunchSequence;
import frc.robot.commands.VisionCommands.AimAndShoot;
import frc.robot.commands.VisionCommands.AimAtPoint;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.CANFuelSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final CANDriveSubsystem driveSubsystem = new CANDriveSubsystem();
  private final CANFuelSubsystem fuelSubsystem = new CANFuelSubsystem();
  private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();
  private final VisionSubsystem visionSubsystem = new VisionSubsystem();

  // The driver's controller
  private final CommandXboxController driverController = new CommandXboxController(
      DRIVER_CONTROLLER_PORT);

  // The operator's controller, by default it is setup to use a single controller
  private final CommandXboxController operatorController = new CommandXboxController(
      OPERATOR_CONTROLLER_PORT);

  // The autonomous chooser
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    autoChooser.addOption("Time Based Example Auto", new ExampleAuto(driveSubsystem, fuelSubsystem));
    autoChooser.addOption("PID Based Example Auto", new SecondExample(driveSubsystem, fuelSubsystem, visionSubsystem));

    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the {@link Trigger#Trigger(java.util.function.BooleanSupplier)}
   * constructor with an arbitrary predicate, or via the named factories in
   * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses
   * for {@link CommandXboxController Xbox}/
   * {@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    // While the left bumper on operator controller is held, intake Fuel
    driverController.leftBumper().whileTrue(new Intake(fuelSubsystem));
    // While the right bumper on the operator controller is held, spin up for 1
    // second, then launch fuel. When the button is released, stop.
    driverController.rightBumper().whileTrue(new LaunchSequence(fuelSubsystem));
    // While the A button is held on the operator controller, eject fuel back out
    // the intake
    driverController.a().whileTrue(new Eject(fuelSubsystem));
   // While the down arrow on the directional pad(d-pad) is held it will unclimb the robot
    driverController.povDown().whileTrue(new ClimbDown(climberSubsystem));
    // While the up arrow on the directional pad(d-pad) is held it will cimb the robot
    driverController.povUp().whileTrue(new ClimbUp(climberSubsystem));

    driverController.back().whileTrue(new AimAndShoot(driveSubsystem, fuelSubsystem, visionSubsystem));

    driverController.rightTrigger().whileTrue(new AimAtPoint(driveSubsystem, () -> -driverController.getLeftY() * DRIVE_SCALING));

    // Set the default command for the drive subsystem to the command provided by
    // factory with the values provided by the joystick axes on the driver
    // controller. The Y axis of the controller is inverted so that pushing the
    // stick away from you (a negative value) drives the robot forwards (a positive
    // value)
    driveSubsystem.setDefaultCommand(new Drive(driveSubsystem, driverController));

    //setting default commands allows a command to be ran if the required subsystem is not being used
    //by another command. I.E the fuel subsystem will be defaulted to stop when not applying a command.
    fuelSubsystem.setDefaultCommand(fuelSubsystem.run(() -> fuelSubsystem.stop()));
    //stops climber motors if not running climber command
    climberSubsystem.setDefaultCommand(climberSubsystem.run(() -> climberSubsystem.stop()));
    //turns LEDs off if not using limelight
    visionSubsystem.setDefaultCommand(visionSubsystem.run(() -> visionSubsystem.setLEDmode(0)));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }
}
