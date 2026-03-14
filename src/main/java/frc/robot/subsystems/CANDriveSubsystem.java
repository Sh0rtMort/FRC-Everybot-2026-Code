// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.utility.WheelForceCalculator.Feedforwards;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPLTVController;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.kinematics.Kinematics;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.DriveConstants.*;

public class CANDriveSubsystem extends SubsystemBase {
  private final SparkMax leftLeader;
  private final SparkMax leftFollower;
  private final SparkMax rightLeader;
  private final SparkMax rightFollower;

  private RelativeEncoder leftEncoder;
  private RelativeEncoder rightEncoder; //set these to the lead encoders

  public Pigeon2 gyro;


  private final DifferentialDrive drive;

  // private final DifferentialDriveOdometry odometry;
  // private final DifferentialDriveKinematics kinematics;


  public CANDriveSubsystem() {
    // create brushed motors for drive
    leftLeader = new SparkMax(LEFT_LEADER_ID, MotorType.kBrushed);
    leftFollower = new SparkMax(LEFT_FOLLOWER_ID, MotorType.kBrushed);
    rightLeader = new SparkMax(RIGHT_LEADER_ID, MotorType.kBrushed);
    rightFollower = new SparkMax(RIGHT_FOLLOWER_ID, MotorType.kBrushed);

    // set up differential drive class
    drive = new DifferentialDrive(leftLeader, rightLeader);

    leftEncoder = leftLeader.getEncoder();
    rightEncoder = rightLeader.getEncoder();

    // gyro = new Pigeon2(pigeon2_ID);

    

    // odometry = new DifferentialDriveOdometry(
    //   gyro.getRotation2d(),
    //    leftEncoder.getPosition(),
    //     rightEncoder.getPosition()
    //   );

    // kinematics = new DifferentialDriveKinematics(trackWidth); //Track width in meters

    // Set can timeout. Because this project only sets parameters once on
    // construction, the timeout can be long without blocking robot operation. Code
    // which sets or gets parameters during operation may need a shorter timeout.
    leftLeader.setCANTimeout(250);
    rightLeader.setCANTimeout(250);
    leftFollower.setCANTimeout(250);
    rightFollower.setCANTimeout(250);

    // Create the configuration to apply to motors. Voltage compensation
    // helps the robot perform more similarly on different
    // battery voltages (at the cost of a little bit of top speed on a fully charged
    // battery). The current limit helps prevent tripping
    // breakers.
    SparkMaxConfig config = new SparkMaxConfig();
    config.voltageCompensation(12);
    config.smartCurrentLimit(DRIVE_MOTOR_CURRENT_LIMIT);
    config.idleMode(IdleMode.kBrake);

    // Set configuration to follow each leader and then apply it to corresponding
    // follower. Resetting in case a new controller is swapped
    // in and persisting in case of a controller reset due to breaker trip
    config.follow(leftLeader);
    leftFollower.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    config.follow(rightLeader);
    rightFollower.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    

    // Remove following, then apply config to right leader
    config.disableFollowerMode();
    rightLeader.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    // Set config to inverted and then apply to left leader. Set Left side inverted
    // so that postive values drive both sides forward
    config.inverted(true);
    leftLeader.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // RobotConfig rConfig;

    // rConfig = new RobotConfig(
    //   robotMass,
    //   (robotMass * (trackWidth * trackWidth)),
    //   null,
    //   trackWidth
    // );

    // try{
    //   rConfig = RobotConfig.fromGUISettings();
    // } catch (Exception e) {
    //   // Handle exception as needed
    //   e.printStackTrace();
    // }

  //   AutoBuilder.configure(
  //     this::getPose,
  //      this::resetPose,
  //       this::getChassisSpeeds,
  //         (speeds, feedforwards) -> driveRobotRelative(speeds),
  //         new PPLTVController(0.02),
  //          rConfig,
  //           () -> {
  //             var alliance = DriverStation.getAlliance();
  //             if (alliance.isPresent()) {
  //               return alliance.get() == DriverStation.Alliance.Red;
  //             }
  //             return false;
  //           },
  //            this
  //   );

  }

  //this is here if needed for the encoders
  private double rpmToMeters(double rotations) {
    double wheelRotations = rotations / gearRatio;
    return wheelRotations * Math.PI * wheelDiameter;
  }

  private double rpmToMetersPerSecond(double rpm) {
    double wheelRPM = rpm / gearRatio;
    double wheelCircumference = Math.PI * wheelRPM;
    return (wheelRPM * wheelCircumference) / 60.0;
}

  public void driveArcade(double xSpeed, double zRotation) {
    drive.arcadeDrive(xSpeed, zRotation);
  }

  // public Pose2d getPose() {
  //   return odometry.getPoseMeters();
  // }

  // public void resetPose(Pose2d pose2d) {
  //   odometry.resetPose(pose2d);
  // }

  // public ChassisSpeeds getChassisSpeeds() {
  //   double leftSpeed = rpmToMetersPerSecond(leftEncoder.getVelocity());
  //   double rightSpeed = rpmToMetersPerSecond(rightEncoder.getVelocity());

  //   return kinematics.toChassisSpeeds(
  //       new DifferentialDriveWheelSpeeds(leftSpeed, rightSpeed)
  //   );
  // }

//   public void driveRobotRelative(ChassisSpeeds speeds) {

//     DifferentialDriveWheelSpeeds wheelSpeeds =
//         kinematics.toWheelSpeeds(speeds);

//     double leftOutput = wheelSpeeds.leftMetersPerSecond;
//     double rightOutput = wheelSpeeds.rightMetersPerSecond;

//     leftLeader.set(leftOutput / 0.95);
//     rightLeader.set(rightOutput / 0.95);
// }

  

  @Override
  public void periodic() {

  }

}
