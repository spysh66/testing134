// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {

  private final double kEncoderCPR = 2048;
  private final double kGearReduction = 10.17;
  private final double kDistancePerWheelRevolutionMeters = 0.481;
  //double check
  private final double encoderConstant = (1 / 8.45) * (1 / kEncoderCPR);


  WPI_TalonFX backLeft = new WPI_TalonFX(1);
  WPI_TalonFX frontLeft = new WPI_TalonFX(3);
  WPI_TalonFX backRight = new WPI_TalonFX(2);
  WPI_TalonFX frontRight = new WPI_TalonFX(4);
  

  final TalonFXInvertType kInvertType = TalonFXInvertType.CounterClockwise;
  


  // The motors on the left side of the drive.
  private final MotorControllerGroup m_leftMotors =
      new MotorControllerGroup(
          new WPI_TalonFX(DriveConstants.kLeftMotor1Port),
          new WPI_TalonFX(DriveConstants.kLeftMotor2Port));

  // The motors on the right side of the drive.
  private final MotorControllerGroup m_rightMotors =
      new MotorControllerGroup(
          new WPI_TalonFX(DriveConstants.kRightMotor1Port),
          new WPI_TalonFX(DriveConstants.kRightMotor2Port));

  // The robot's drive
  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);
  

  // // The left-side drive encoder
  // private final Encoder m_leftEncoder =
  //     new Encoder(
  //         DriveConstants.kLeftEncoderPorts[1],
  //         DriveConstants.kLeftEncoderPorts[3],
  //         DriveConstants.kLeftEncoderReversed);

  // // The right-side drive encoder
  // private final Encoder m_rightEncoder =
  //     new Encoder(
  //         DriveConstants.kRightEncoderPorts[2],
  //         DriveConstants.kRightEncoderPorts[4],
  //         DriveConstants.kRightEncoderReversed);

  // The gyro sensor
  private final Gyro m_gyro = new AHRS(SPI.Port.kMXP);
  // Odometry class for tracking robot pose
  private final DifferentialDriveOdometry m_odometry;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
  //  frontRight.setInverted(kInvertType);
  //  backRight.setInverted(kInvertType);
  TalonFXConfiguration configs = new TalonFXConfiguration();

    configs.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;

    frontLeft.configAllSettings(configs);
    frontRight.configAllSettings(configs);

    frontLeft.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20);
    frontRight.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 20);

    //Determines which motors will be inverted
    frontLeft.setInverted(InvertType.InvertMotorOutput);
   // backLeft.setInverted(InvertType.FollowMaster);
    frontRight.setInverted(InvertType.None);
  //  backRight.setInverted(InvertType.FollowMaster);

    //Sets the motors to brake mode
    frontLeft.setNeutralMode(NeutralMode.Brake);
   // backLeft.setNeutralMode(NeutralMode.Brake);
    frontRight.setNeutralMode(NeutralMode.Brake);
   // backRight.setNeutralMode(NeutralMode.Brake);

    //  backLeft.set(ControlMode.Follower, frontLeft.getDeviceID());
    //  backRight.set(ControlMode.Follower, frontRight.getDeviceID());

    // myRobot = new DifferentialDrive(m_leftFront, m_rightFront);
    // myRobot.setDeadband(0);
    // myRobot.setSafetyEnabled(false);

    zeroHeading();


    // TalonFXConfiguration configs = new TalonFXConfiguration();
	
		// 	configs.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
	
		// 	backLeft.configAllSettings(configs);
    //   frontLeft.configAllSettings(configs);
    //   backRight.configAllSettings(configs);
    //   frontRight.configAllSettings(configs);

    //   m_rightMotors.setInverted(true);
    //   m_leftMotors.setInverted(true);

    // Sets the distance per pulse for the encoders
    // m_leftEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
    // m_rightEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);

    resetEncoders();
    m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());
  }


  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    double heading = getHeading();
    double leftDist = getLeftPosition();
    double rightDist = getRightPosition();
    m_odometry.update(
      Rotation2d.fromDegrees(heading),
      leftDist,
      rightDist
);
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(
        getLeftVelocity(),
        getRightVelocity());
   // return new DifferentialDriveWheelSpeeds(((double) frontLeft.getSelectedSensorVelocity(0) / 2048 * 10), ((double) frontRight.getSelectedSensorVelocity(0) / 2048 * 10));
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, m_gyro.getRotation2d());
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    m_drive.arcadeDrive(fwd, rot);
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts the commanded left output
   * @param rightVolts the commanded right output
   */
  public synchronized void tankDriveVolts(double leftVolts, double rightVolts) {
    frontLeft.setVoltage(-leftVolts);
    frontRight.setVoltage(-rightVolts);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    frontRight.getSensorCollection().setIntegratedSensorPosition(0, 10);
    frontLeft.getSensorCollection().setIntegratedSensorPosition(0, 10);
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  // public double getAverageEncoderDistance() {
  //   return (((double) frontLeft.getSelectedSensorPosition(0) / 2048) + ((double) frontRight.getSelectedSensorPosition(0) / 2048)) / 2.0;
  // }

  //  public double getLeftDistance() {
  //   return -frontLeft.getSelectedSensorPosition(0) * (((Math.PI * Units.inchesToMeters(6)) / 2048) / 10.71);
  //   //return -m_leftFront.getSelectedSensorPosition(0) * encoderConstant;
  // }
  // public double getRightDistance() {
  //   return -frontRight.getSelectedSensorPosition(0) * (((Math.PI * Units.inchesToMeters(6)) / 2048) / 10.71);
  //   //return -m_rightFront.getSelectedSensorPosition(0) * encoderConstant;
  // }

  double getLeftPosition() {
    // Native units are encoder ticks (2048 ticks per revolution)
   return frontLeft.getSelectedSensorPosition() * kDistancePerWheelRevolutionMeters * kGearReduction / kEncoderCPR;
 }

 /**
  * Returns the distance in Meters the right wheel has travelled
  *
  * @return distance in meters
  */
 double getRightPosition() {
   // Native units are encoder ticks (2048 ticks per revolution)
   return frontRight.getSelectedSensorPosition() * kDistancePerWheelRevolutionMeters * kGearReduction / kEncoderCPR;
 }

 /**
  * Returns the velocity of the left wheel in meters per second
  *
  * @return velocity in meters/second
  */
 double getLeftVelocity() {
   // Native units are encoder ticks per 100ms
   return frontLeft.getSelectedSensorVelocity() * kDistancePerWheelRevolutionMeters * kGearReduction * 10.0 / kEncoderCPR ;
 }

 /**
  * Returns the velocity of the right wheel in meters per second
  *
  * @return velocity in meters/second
  */
 double getRightVelocity() {
   // Native units are encoder ticks per 100ms
   return frontRight.getSelectedSensorVelocity() * kDistancePerWheelRevolutionMeters * kGearReduction * 10.0 / kEncoderCPR ;
 }

  

  // /**
  //  * Gets the right drive encoder.
  //  *
  //  * @return the right drive encoder
  //  */
  // public Encoder getRightEncoder() {
  //   return m_rightEncoder;
  // }

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return -m_gyro.getRate();
  }
}
