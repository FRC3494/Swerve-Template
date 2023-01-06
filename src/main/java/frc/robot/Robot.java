package frc.robot;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.stream.Stream;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
	private Command autonomousCommand;

	private RobotContainer robotContainer;

	private SendableChooser<String> chooser;

	@Override
	public void robotInit() {
		robotContainer = new RobotContainer();

		Path autoPath = Filesystem.getDeployDirectory().toPath().resolve(Constants.fullPathToAuto(""));

		chooser = new SendableChooser<>();

		try (Stream<Path> list = Files.list(autoPath)) {
			list
					.filter(Files::isRegularFile)
					.map(Path::getFileName)
					.map(Path::toString)
					.forEach((String autoFileName) -> {
						chooser.addOption(autoFileName, autoFileName);
					});
		} catch (IOException e) {
			e.printStackTrace();
		}

		Shuffleboard.getTab("Autonomous").add(chooser);
	}

	@Override
	public void robotPeriodic() {
		CommandScheduler.getInstance().run();
	}


	@Override
	public void disabledInit() {
	}

	@Override
	public void disabledPeriodic() {
	}


	@Override
	public void autonomousInit() {
		//autonomousCommand = robotContainer.getAutonomousCommand(chooser.getSelected());
    autonomousCommand = robotContainer.getAutonomousCommand();

		// schedule the autonomous command (example)
		if (autonomousCommand != null) {
			autonomousCommand.schedule();
		}
	}

	@Override
	public void autonomousPeriodic() {
	}


	@Override
	public void teleopInit() {
		if (autonomousCommand != null) {
			autonomousCommand.cancel();
		}
	}

	@Override
	public void teleopPeriodic() {
	}


	@Override
	public void testInit() {
		CommandScheduler.getInstance().cancelAll();
	}

	@Override
	public void testPeriodic() {
	}
}
