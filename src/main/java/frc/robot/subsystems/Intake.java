package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private double speed = 0;
    private String extendStatus = "Retracted";
    private Timer extendTimer = new Timer();

    public void in() {
        speed = 0.75;
    }

    public void out() {
        speed = -0.75;
    }

    // intended for prototyping time
    public void setSpeedRaw(double speed) {
        this.speed = speed;
    }

    public void stop() {
        speed = 0;
    }

    public void extend() {
        extendStatus = "Extending";
        extendTimer.restart();
    }

    public void retract() {
        extendStatus = "Retracting";
        extendTimer.restart();
    }

    public Boolean isExtended() {
        return extendStatus == "Extended";
    }

    public Boolean isRetracted() {
        return extendStatus == "Retracted";
    }

    @Override
    public void periodic() {
        if (extendStatus == "Extending" && extendTimer.hasElapsed(2)) {
            extendStatus = "Extended";
            extendTimer.stop();
            extendTimer.reset();
        }
        else if (extendStatus == "Retracting" && extendTimer.hasElapsed(2)) {
            extendStatus = "Retracted";
            extendTimer.stop();
            extendTimer.reset();
        }

        SmartDashboard.putNumber("Intake/speed", speed);
        SmartDashboard.putString("Intake/extendStatus", extendStatus);
    }
}
