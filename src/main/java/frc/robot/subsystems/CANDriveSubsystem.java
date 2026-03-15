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

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.kinematics.Kinematics;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

import static frc.robot.Constants.DriveConstants.*;

import java.lang.constant.Constable;

public class CANDriveSubsystem extends SubsystemBase {
  private final SparkMax leftLeader;
  private final SparkMax leftFollower;
  private final SparkMax rightLeader;
  private final SparkMax rightFollower;

  private RelativeEncoder leftEncoder;
  private RelativeEncoder rightEncoder; //set these to the lead encoders

  private Pigeon2 gyro = new Pigeon2(31);

  private DifferentialDriveKinematics kinematics = 
    new DifferentialDriveKinematics(Constants.DriveConstants.trackWidth);

  private final DifferentialDriveOdometry odometry;

  private final PIDController headingPID =
    new PIDController(10, 0.0, 3);

  private final Field2d field = new Field2d();

  private final DifferentialDrive drive;

  private DifferentialDrivetrainSim driveSim;

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

    // leftEncoder.setPosition(0);
    // rightEncoder.setPosition(0);

    gyro.getConfigurator().apply(new Pigeon2Configuration());

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

    odometry = new DifferentialDriveOdometry(getHeading(), leftEncoder.getPosition(), rightEncoder.getPosition());

    SmartDashboard.putData("Field", field);

    driveSim = new DifferentialDrivetrainSim(
    DCMotor.getNEO(2),       // motors per side
    Constants.DriveConstants.gearRatio,                   // gearing
    7.5,                     // moment of inertia
    Constants.DriveConstants.robotMass,                    // robot mass (kg)
    Constants.DriveConstants.wheelDiameter / 2, // wheel radius
    Constants.DriveConstants.trackWidth,        // track width
    null
  );
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

  public double getEncoderMeters() {
    return (leftEncoder.getPosition() + -rightEncoder.getPosition()) / 2 * DriveConstants.encoderTick2Meters;
  }

  public double getLeftDistanceMeters() {
    return leftEncoder.getPosition() * encoderTick2Meters;
  }

  public double getRightDistanceMeters() {
    return -rightEncoder.getPosition() * encoderTick2Meters;
  }

  public void driveArcade(double xSpeed, double zRotation) {
    drive.arcadeDrive(xSpeed, zRotation);
  }

  public Rotation2d getHeading() {
  return Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble());
  }

  @Override
  public void periodic() {

  Pose2d pose = odometry.update(
    getHeading(),
    getLeftDistanceMeters(),
    getRightDistanceMeters()
  );

  field.setRobotPose(pose);

  SmartDashboard.putNumber("Left Encoder", leftEncoder.getPosition());
  SmartDashboard.putNumber("Right Encoder", rightEncoder.getPosition());

  }

  public void aimAtPoint(Translation2d target, double forwardSpeed) {

    Pose2d pose = odometry.getPoseMeters();

    Rotation2d targetAngle =
        target.minus(pose.getTranslation()).getAngle();

    double omega =
        headingPID.calculate(
            pose.getRotation().getRadians(),
            targetAngle.getRadians()
        );

    ChassisSpeeds speeds =
        new ChassisSpeeds(forwardSpeed, 0, omega);

    DifferentialDriveWheelSpeeds wheelSpeeds =
        kinematics.toWheelSpeeds(speeds);

    drive.tankDrive(
        wheelSpeeds.leftMetersPerSecond,
        wheelSpeeds.rightMetersPerSecond
    );

}

@Override
public void simulationPeriodic() {

    // Send motor outputs to the sim
    driveSim.setInputs(
        leftLeader.get() * 12.0,
        rightLeader.get() * 12.0
    );

    driveSim.update(0.02); // 20ms loop

    // Update encoders
    leftEncoder.setPosition(
        driveSim.getLeftPositionMeters()
    );

    rightEncoder.setPosition(
        driveSim.getRightPositionMeters()
    );

    // Update gyro
    gyro.getSimState().setRawYaw(
        driveSim.getHeading().getDegrees()
    );
  }

}
