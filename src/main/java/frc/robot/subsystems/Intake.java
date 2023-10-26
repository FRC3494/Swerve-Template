package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
    CANSparkMax wristMotor;
    CANSparkMax intakeMotor;

    public Intake() {
        wristMotor = new CANSparkMax(Constants.Subsystems.Intake.WRIST_MOTOR_PORT, MotorType.kBrushless);
        wristMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        intakeMotor = new CANSparkMax(Constants.Subsystems.Intake.INTAKE_MOTOR_PORT, MotorType.kBrushless);
    }

    @Override
    public void periodic() {

    }

    public void setPower(double wristPower) {
        System.out.println("POWERING" + wristPower);
        wristMotor.set(wristPower);
    }

    public void setIntakePower(double intakePower) {
        intakeMotor.set(intakePower);
    }
    
}
