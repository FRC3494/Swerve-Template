package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Drivetrain;

public class TeleopDrive extends CommandBase {
	Drivetrain drivetrain;
	Intake intake;

	public TeleopDrive(Drivetrain drivetrain, Intake intake) {
		this.drivetrain = drivetrain;
		this.intake = intake;
		addRequirements(drivetrain);
		addRequirements(intake);
	}

	@Override
	public void execute() {
		drivetrain.drive(OI.getTeleopXVelocity(), OI.getTeleopYVelocity(), OI.getTeleopTurnVelocity(), true);
		intake.setPower(OI.getIntakePower());
		intake.setIntakePower(OI.getIntakeSpeed());
	}

	@Override
	public void end(boolean interrupted) {
		drivetrain.drive(0, 0, 0, false);
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
