// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.I2C;
// import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
// import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ChasisConstants;

public class DriveTrain extends SubsystemBase {
  /** Creates a new DriveTrain. */
  private WPI_TalonFX frontLeft = new WPI_TalonFX(ChasisConstants.frontLeftID);
  private WPI_TalonFX frontRigth = new WPI_TalonFX(ChasisConstants.frontRightID);
  private WPI_TalonFX rearLeft = new WPI_TalonFX(ChasisConstants.rearLeftID);
  private WPI_TalonFX rearRight = new WPI_TalonFX(ChasisConstants.rearRightID);

  private final double kEncoderTick2Meter = 1 / 2048 * 4 * Math.PI * 1 / 39.37;

  private final MotorControllerGroup m_leftMotors = new MotorControllerGroup(frontLeft, rearLeft);
  private final MotorControllerGroup m_rightMotors = new MotorControllerGroup(frontRigth, rearRight);

  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors,m_rightMotors);


  // private final Gyro m_gyro = new ADXRS450_Gyro();
  private final AHRS navx = new AHRS(I2C.Port.kMXP);
  

  private final DifferentialDriveOdometry m_odometry;

  public DriveTrain() {
    m_rightMotors.setInverted(true);

    // double leftEncoder = frontLeft.getSelectedSensorPosition()*kEncoderTick2Meter;
    // double rightEncoder = frontRigth.getSelectedSensorPosition()*kEncoderTick2Meter;

    resetEncoders();
    m_odometry = new DifferentialDriveOdometry(navx.getRotation2d());


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_odometry.update(
      navx.getRotation2d(), 
      frontLeft.getSelectedSensorPosition()*kEncoderTick2Meter, 
      frontRigth.getSelectedSensorPosition()*kEncoderTick2Meter);

  }

  public Pose2d getPose(){
    return m_odometry.getPoseMeters();
  }

    /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeed(){
    return new DifferentialDriveWheelSpeeds(
      frontLeft.getSelectedSensorVelocity()*kEncoderTick2Meter, 
      frontRigth.getSelectedSensorVelocity()*kEncoderTick2Meter);
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, navx.getRotation2d());
  }

  public void arcadeDrive(double fwd, double rot){
    m_drive.arcadeDrive(fwd, rot);
  }

  public void tankDriveVolts(double leftVolts, double rightVolts){
    m_leftMotors.setVoltage(leftVolts);
    m_rightMotors.setVoltage(rightVolts);
    m_drive.feed();
  }

  public void resetEncoders(){
    frontLeft.getSensorCollection().setIntegratedSensorPosition(0, 10);
    frontRigth.getSensorCollection().setIntegratedSensorPosition(0, 10);
  }

  public double getAverageEncoderDistance(){
    return (frontLeft.getSelectedSensorPosition()+frontRigth.getSelectedSensorPosition() / 2.0);
  }

  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  public void zeroHeading(){
    navx.reset();
  }

  public double getHeading(){
    return navx.getRotation2d().getDegrees();
  }

  public double getTurnRate(){
    return -navx.getRate();
  }
}
