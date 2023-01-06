// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.subsystems.Drivetrain;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
	private final Drivetrain drivetrain = new Drivetrain();

	XboxController primaryController = new XboxController(0);

	String autoFile = "TestPath.wpilib.json";

	public RobotContainer() {
		// Configure the button bindings
		configureButtonBindings();

		// Configure default commands
		drivetrain.setDefaultCommand(
			new RunCommand(
				() -> drivetrain.drive(
					modifyAxis(primaryController.getLeftY()),
					modifyAxis(primaryController.getLeftX()),
					modifyAxis(primaryController.getRightX()),
					true
				),
				drivetrain
			)
		);
	}

	private void configureButtonBindings() {
	}

    public static final TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
        Constants.MAX_VELOCITY_METERS_PER_SECOND,
        Constants.maxAcceleration)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(Constants.kDriveKinematics);

	
	public Command getAutonomousCommand() {
		Trajectory trajectory;

		
		// An example trajectory to follow. All units in meters.
		trajectory = TrajectoryGenerator.generateTrajectory(
				// Start at the origin facing the +X direction
				new Pose2d(0, 0, new Rotation2d(0)),
				// Pass through these two interior waypoints, making an 's' curve path
				List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
				// End 3 meters straight ahead of where we started, facing forward
				new Pose2d(3, 0, new Rotation2d(0)),
				trajectoryConfig);
		
		
		/*try { 
			Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(AutoConstants.expandAutoFile(autoName));
			
			trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
		} catch (IOException e) {
			e.printStackTrace();
			return null;
		}*/

		var thetaController = new ProfiledPIDController(1, 0, 0, Constants.kThetaControllerConstraints);
		thetaController.enableContinuousInput(-Math.PI, Math.PI);

		SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
			trajectory,
				drivetrain::getPose, // Functional interface to feed supplier
				Constants.kDriveKinematics,

				// Position controllers
				new PIDController(1, 0, 0),
				new PIDController(1, 0, 0),
				thetaController,
				drivetrain::setModuleStates,
				drivetrain);

		// Reset odometry to the starting pose of the trajectory.
		drivetrain.resetOdometry(trajectory.getInitialPose());

		// Run path following command, then stop at the end.
		return swerveControllerCommand.andThen(() -> drivetrain.drive(0, 0, 0, false));
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
