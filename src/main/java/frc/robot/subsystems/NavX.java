package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.util.sendable.SendableBuilder;


public class NavX{
    private AHRS ahrs = new AHRS();

    private double pitchOffset = 0;

    public void initialize() {
        //this.ahrs.calibrate();
        this.pitchOffset = this.ahrs.getRoll();
    }

    public double getYaw() {
        return this.ahrs.getFusedHeading();
    }

    public double getPitch() {
        //System.out.println(-this.ahrs.getRoll() + this.pitchOffset);
        return -this.ahrs.getRoll() + this.pitchOffset;
    }
    public void zeroYaw(){
        this.pitchOffset = this.ahrs.getRoll();
    }
    //@Override FIXEME make this work with ShuffleBoard
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Gyro");
        builder.addDoubleProperty("Value", this::getYaw, null);
    }
    public AHRS getNavX(){
        return this.ahrs;
    }
}