package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PlaceholderCode.*;


public class Shooter extends SubsystemBase{
    FakeMotor shootMotor = new FakeMotor(41);
    FakeMotor hoodMotor = new FakeMotor(42);
    FakeMotor feedMotor = new FakeMotor(43);
    FakeEncoder hoodEncoder = new FakeEncoder(50);
    PIDController shooterPID = new PIDController(0, 0, 0);
    PIDController hoodPID = new PIDController(0, 0, 0);
    double motorSpeedGoal = 0;
    double hoodGoal = 0;

    public Shooter() {
        super();

        SmartDashboard.putData("Shooter/shootController", shooterPID);
        SmartDashboard.putData("Shooter/hoodController", hoodPID);
    }
    
    public void setSpeedGoal(double speed) {
        shooterPID.setSetpoint(speed);
    }
    
    public void inFeed() {
        feedMotor.set(0.2);
    }

    public void backFeed() {
        feedMotor.set(-0.2);
    }

    public void stopFeed() {
        feedMotor.set(0);
    }

    public void angleHood(double angle) {
        hoodPID.setSetpoint(angle);
    }

    public void lowerHood() {
        hoodPID.setSetpoint(0);
    }

    public void aim(double dist) {
        setSpeedGoal(0);
        angleHood(0);
    }

    public boolean isAimed() {
        return shooterPID.atSetpoint() && hoodPID.atSetpoint();
    }



    @Override
    public void periodic() {
        double motorSpeed = shooterPID.calculate(shootMotor.getSpeed());
        shootMotor.set(motorSpeed);
        double hoodSpeed = hoodPID.calculate(hoodEncoder.getPosition());
        hoodMotor.set(hoodSpeed);

        SmartDashboard.putNumber("Shooter/shootActual", shootMotor.getSpeed());
        SmartDashboard.putNumber("Shooter/hoodActual", hoodEncoder.getPosition());
    
    }
}
