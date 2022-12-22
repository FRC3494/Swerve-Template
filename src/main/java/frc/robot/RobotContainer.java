// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.List;

import javax.naming.Reference;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.spline.Spline.ControlVector;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.subsystems.DrivetrainSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DrivetrainSubsystem m_drivetrainSubsystem;

  private final XboxController m_controller = new XboxController(0);

 
  private final HolonomicDriveController test;
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() 
  {
    m_drivetrainSubsystem = new DrivetrainSubsystem();
   
    // Set up the default command for the drivetrain.
    // The controls are for field-oriented driving:
    // Left stick Y axis -> forward and backwards movement
    // Left stick X axis -> left and right movement
    // Right stick X axis -> rotation
  
    m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
            m_drivetrainSubsystem,
            () -> -modifyAxis(m_controller.getLeftY()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            () -> -modifyAxis(m_controller.getLeftX()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            () -> -modifyAxis(m_controller.getRightX()) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
    ));

    PIDController move = new PIDController(.15, 0.00002, 0.0001);
    ProfiledPIDController rotate = new ProfiledPIDController(.15, 0.00002, 0.0001, new TrapezoidProfile.Constraints(6.28, 3.14));
    test = new HolonomicDriveController(move, move, rotate);

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() 
  {
    
    // Back button zeros the gyroscope
    new Button(m_controller::getXButton)
            // No requirements because we don't need to interrupt anything
            .whenPressed(m_drivetrainSubsystem::zeroGyroscope);

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
   
  //Pose2d togo = new Pose2d(0, 0, new Rotation2d(Math.toRadians(45)));

  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    startTime = System.currentTimeMillis();

    return new DefaultDriveCommand( m_drivetrainSubsystem, 
      () -> -grabSpeeds().vyMetersPerSecond,
      () -> -grabSpeeds().vxMetersPerSecond,
      () -> -grabSpeeds().omegaRadiansPerSecond);    
  }

  /*Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
    /*new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
    List.of(),
    new Pose2d(0.2, 0, Rotation2d.fromDegrees(0)),
    new TrajectoryConfig(1, 1)).concatenate(
      TrajectoryGenerator.generateTrajectory(
        new Pose2d(0.2, 0, Rotation2d.fromDegrees(0)),
        List.of(),
        new Pose2d(0.2, 0.2, Rotation2d.fromDegrees(0)),
        new TrajectoryConfig(1, 1))
    // Start at the origin facing the +X direction
    new Pose2d(0, 0, new Rotation2d(0)),
    // Pass through these two interior waypoints, making an 's' curve path
    List.of(),
    // End 3 meters straight ahead of where we started, facing forward
    new Pose2d(3, 0, new Rotation2d(0))
    );*/
    TrajectoryConfig config = new TrajectoryConfig(1,1);


// An example trajectory to follow.  All units in meters.

Trajectory trajectory =
    TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(0.5, 0, new Rotation2d(0)),
        // Pass config
        config);
  long startTime = -1;

  private ChassisSpeeds grabSpeeds()
  {
     //return test.calculate(m_drivetrainSubsystem.getOdometry().getPoseMeters(), togo,  0.5, new Rotation2d(0) );
    double time = (System.currentTimeMillis() - startTime) / 1000.0;
    
    Trajectory.State goal = trajectory.sample(time);

    return test.calculate(m_drivetrainSubsystem.getOdometry().getPoseMeters(), goal, Rotation2d.fromDegrees(90.0));
  }

  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) 
    {
      if (value > 0.0) 
      {
        return (value - deadband) / (1.0 - deadband);
      } 
      
      return (value + deadband) / (1.0 - deadband);
      
    } 

    return 0.0;
    
  }

  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, 0.05);

    // Square the axis
    value = Math.copySign(value * value* value, value);

    return value*.5;
  }

 
}
