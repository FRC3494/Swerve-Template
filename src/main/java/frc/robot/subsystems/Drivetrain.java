package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

import com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SwerveModule;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;


public class Drivetrain extends SubsystemBase {
			
	// Robot swerve modules
	SwerveModule frontLeft = Mk4iSwerveModuleHelper.createNeo(
            // This parameter is optional, but will allow you to see the current state of the module on the dashboard.
            Shuffleboard.getTab("Drivetrain").getLayout("Front Left Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(0, 0),
            // This can either be Mk4i or FAST depending on your gear configuration
            Mk4iSwerveModuleHelper.GearRatio.L2,
            // This is the ID of the drive motor
            FRONT_LEFT_MODULE_DRIVE_MOTOR,
            // This is the ID of the steer motor
            FRONT_LEFT_MODULE_STEER_MOTOR,
            // This is the ID of the steer encoder
            FRONT_LEFT_MODULE_STEER_ENCODER,
            // This is how much the steer encoder is offset from true zero (In our case, zero is facing straight forward)
            FRONT_LEFT_MODULE_STEER_OFFSET
    );

    // We will do the same for the other modules
    SwerveModule frontRight = Mk4iSwerveModuleHelper.createNeo(
			Shuffleboard.getTab("Drivetrain").getLayout("Front Right Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(2, 0),
            Mk4iSwerveModuleHelper.GearRatio.L2,
            FRONT_RIGHT_MODULE_DRIVE_MOTOR,
            FRONT_RIGHT_MODULE_STEER_MOTOR,
            FRONT_RIGHT_MODULE_STEER_ENCODER,
            FRONT_RIGHT_MODULE_STEER_OFFSET
    );

    SwerveModule rearLeft = Mk4iSwerveModuleHelper.createNeo(
			Shuffleboard.getTab("Drivetrain").getLayout("Back Left Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(4, 0),
            Mk4iSwerveModuleHelper.GearRatio.L2,
            BACK_LEFT_MODULE_DRIVE_MOTOR,
            BACK_LEFT_MODULE_STEER_MOTOR,
            BACK_LEFT_MODULE_STEER_ENCODER,
            BACK_LEFT_MODULE_STEER_OFFSET
    );

    SwerveModule rearRight = Mk4iSwerveModuleHelper.createNeo(
			Shuffleboard.getTab("Drivetrain").getLayout("Back Right Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(6, 0),
            Mk4iSwerveModuleHelper.GearRatio.L2,
            BACK_RIGHT_MODULE_DRIVE_MOTOR,
            BACK_RIGHT_MODULE_STEER_MOTOR,
            BACK_RIGHT_MODULE_STEER_ENCODER,
            BACK_RIGHT_MODULE_STEER_OFFSET
    );

	// The gyro sensor
	private final NavX m_navx = new NavX();

	

	// Odometry class for tracking robot pose
	SwerveDriveOdometry odometry = new SwerveDriveOdometry(kDriveKinematics, getGyroscopeRotation());

	/** Creates a new DriveSubsystem. */
	public Drivetrain() {
	}

	@Override
	public void periodic() {
		// Update the odometry in the periodic block
		odometry.update(
				getGyroscopeRotation(),
				frontLeft.getState(),
				frontRight.getState(),
				rearLeft.getState(),
				rearRight.getState());

		//System.out.print(frontLeft.getState().angle + " | " + frontRight.getState().angle + " | " + rearLeft.getState().angle + " | " + rearRight.getState().angle);
	}

	/**
	 * Returns the currently-estimated pose of the robot.
	 *
	 * @return The pose.
	 */
	public Pose2d getPose() {
		return odometry.getPoseMeters();
	}

	/**
	 * Resets the odometry to the specified pose.
	 *
	 * @param pose The pose to which to set the odometry.
	 */
	public void resetOdometry(Pose2d pose) {
		odometry.resetPosition(pose, getGyroscopeRotation());
	}

	/**
	 * Method to drive the robot using joystick info.
	 *
	 * @param xSpeed        Speed of the robot in the x direction (forward).
	 * @param ySpeed        Speed of the robot in the y direction (sideways).
	 * @param rot           Angular rate of the robot.
	 * @param fieldRelative Whether the provided x and y speeds are relative to the
	 *                      field.
	 */
	@SuppressWarnings("ParameterName")
	public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
		var swerveModuleStates = kDriveKinematics.toSwerveModuleStates(
				fieldRelative
						? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getGyroscopeRotation())
						: new ChassisSpeeds(xSpeed, ySpeed, rot));
		
		SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, MAX_VELOCITY_METERS_PER_SECOND);
		
		if(Math.abs(xSpeed)<.001 
    	   && Math.abs(ySpeed)<.001
    	   && Math.abs(rot)<.001)
    	{
    	      frontLeft.set(0, frontLeft.getState().angle.getRadians());
    	      frontRight.set(0, frontRight.getState().angle.getRadians());
    	      rearLeft.set(0, rearLeft.getState().angle.getRadians());
    	      rearRight.set(0, rearRight.getState().angle.getRadians());
    	      return;
    	}

    	setModuleStates(swerveModuleStates);
	}

	/**
	 * Sets the swerve ModuleStates.
	 *
	 * @param desiredStates The desired SwerveModule states.
	 */
	public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(
                desiredStates, MAX_VELOCITY_METERS_PER_SECOND);
        frontLeft.set(desiredStates[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, desiredStates[0].angle.getRadians());
        frontRight.set(desiredStates[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, desiredStates[1].angle.getRadians());
        rearLeft.set(desiredStates[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, desiredStates[2].angle.getRadians());
        rearRight.set(desiredStates[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, desiredStates[3].angle.getRadians());
	}


	public Rotation2d getGyroscopeRotation() {
		// FIXME Remove if you are using a Pigeon
		//Rotation2d heading = Rotation2d.fromDegrees(m_pigeon.getYaw());
		//System.out.println(heading);
		//return Rotation2d.fromDegrees(0); //This line is for testing purposes DO NOT UNCOMMENT unless you want to test without your IMU
		//return heading;
	
		// FIXME Uncomment if you are using a NavX
		if (m_navx.getNavX().isMagnetometerCalibrated()) {
		  // We will only get valid fused headings if the magnetometer is calibrated
		  return Rotation2d.fromDegrees(m_navx.getYaw());
		}
	
		// We have to invert the angle of the NavX so that rotating the robot counter-clockwise makes the angle increase.
	   return Rotation2d.fromDegrees(360.0 - m_navx.getYaw());
	  }
}
