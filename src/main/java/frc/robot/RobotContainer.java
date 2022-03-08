// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveTrain;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  String trajectoryJSON = "paths/output/test.wpilib.json";
  Trajectory trajectory = new Trajectory();

  private final DriveTrain m_driveTrain = new DriveTrain();
  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    
    try{
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex){
      DriverStation.reportError("Unable to open trajectory" + trajectoryJSON, ex.getStackTrace());
    }


    // An ExampleCommand will run in autonomous
    var autoVoltageConstraint = 
      new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(
          DriveConstants.ksVolts, 
          DriveConstants.ksVoltSecondsPerMeter, 
          DriveConstants.kaVoltSecondsSquaredPerMeter), 
        DriveConstants.kDriveKinematics, 
        10);

    TrajectoryConfig config = new TrajectoryConfig(
      AutoConstants.kMaxSpeedMetersPerSecond, 
      AutoConstants.kMaxAccelerationMetersPerSecond)
      .setKinematics(DriveConstants.kDriveKinematics)
      .addConstraint(autoVoltageConstraint);

    Trajectory exampleTrajectory =
      TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)), 
        List.of(new Translation2d(0.2,0), new Translation2d(0.5,0)), 
        new Pose2d(0.7, 0, new Rotation2d(30)), 
        config);
    
    RamseteCommand ramseteCommand = 
      new RamseteCommand(
        trajectory, 
        m_driveTrain::getPose, 
        new RamseteController(
          AutoConstants.kRamseteB, 
          AutoConstants.kRamseteZeta), 
        new SimpleMotorFeedforward(
          DriveConstants.ksVolts, 
          DriveConstants.ksVoltSecondsPerMeter, 
          DriveConstants.kaVoltSecondsSquaredPerMeter), 
        DriveConstants.kDriveKinematics, 
        m_driveTrain::getWheelSpeed, 
        new PIDController(DriveConstants.kPDriveVel, 0, 0), 
        new PIDController(DriveConstants.kPDriveVel, 0, 0), 
        m_driveTrain::tankDriveVolts, 
        m_driveTrain);
    
    m_driveTrain.resetOdometry(exampleTrajectory.getInitialPose());

    return ramseteCommand.andThen(() -> m_driveTrain.tankDriveVolts(0, 0));
 
  }
}
