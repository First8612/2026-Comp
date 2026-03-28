package frc.robot.controls;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import java.util.OptionalDouble;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer;

public class Controls {
    protected final CommandXboxController driver = new CommandXboxController(0);
    protected final CommandXboxController operator = new CommandXboxController(1);
    protected final Trigger noButton = new Trigger(() -> false);
    protected final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(RobotContainer.MaxSpeed * 0.05)
            .withRotationalDeadband(RobotContainer.MaxAngularRate * 0.05) // Add a 5% deadband
            .withDriveRequestType(DriveRequestType.Velocity); // Use open-loop control for drive motors

    // Driver ******************************
    public SwerveRequest.FieldCentric getDriveRequest() {
        return drive
                .withVelocityX(-driver.getLeftY() * RobotContainer.MaxSpeed
                        * (RobotContainer.prescisionMode == true ? 0.25 : 1)) // Drive forward with negative Y
                // (forward)
                .withVelocityY(-driver.getLeftX() * RobotContainer.MaxSpeed
                        * (RobotContainer.prescisionMode == true ? 0.25 : 1)) // Drive left with negative X (left)
                .withRotationalRate(-driver.getRightX() * RobotContainer.MaxAngularRate
                        * (RobotContainer.prescisionMode == true ? 0.25 : 1)); // Drive counterclockwise with
        // negative X (left)
    }

    public Trigger driveAndFaceTarget() {
        // This is the same event as pressing M2
        return driver.a();
    }
    public Trigger goToClimb() {
        return driver.b();
    }
    
    public Trigger horn() {
        return driver.rightStick();
    }

    public Trigger fieldReset() {
        return driver.back();
    }

    public Trigger trenchRun() {
        return driver.rightBumper();
    }

    public Trigger prescisionMode() {
        return driver.leftBumper();
    }

    // Operator ******************************
    public Trigger intakeRetract() {
        return operator.axisGreaterThan(XboxController.Axis.kRightTrigger.value, 0.8);
    }

    public Trigger intakeExtend() {
        return operator.axisGreaterThan(XboxController.Axis.kLeftTrigger.value, 0.8);
    }

    public Trigger shoot() {
        return operator.x();
    }

    public Trigger shootSimiple() {
        return operator.y();
    }

    public Trigger intake() {
        return operator.rightBumper();
    }

    public Trigger conveyIn() {
        return operator.povUp();
    }

    public Trigger conveyOut() {
        return operator.povDown();
    }

    public Trigger feedOut() {
        return operator.a();
    }

    public Trigger lowerClimb() {
        return operator.axisGreaterThan(XboxController.Axis.kLeftY.value, 0.8);
    }

    public Trigger raiseClimb() {
        return operator.axisLessThan(XboxController.Axis.kLeftY.value, -0.8);
    }

    public Trigger useClimb() {
        return operator.leftStick();
    }

    public Trigger manualClimb() {
        return operator.axisGreaterThan(XboxController.Axis.kLeftY.value, 0.1)
                .or(operator.axisLessThan(XboxController.Axis.kLeftY.value, -0.1));
    }

    public Trigger manualReset() {
        return operator.leftBumper().and(operator.a());
    }

    public double getClimbManual() {
        return operator.getLeftY();
    }

    public Trigger zeroHood() {
        return operator.b();
    }

    public Trigger changeColor() {
        return operator.povRight();
    }

    // Events
    public void setRumble(double value) {
        driver.setRumble(RumbleType.kBothRumble, value);
        operator.setRumble(RumbleType.kBothRumble, value);
    }

    // No controls
    public Trigger sysIdDrivetrainDynamicForward() {
        return noButton;
    }

    public Trigger sysIdDrivetrainDynamicReverse() {
        return noButton;
    }

    public Trigger sysIdDrivetrainQuasistaticForward() {
        return noButton;
    }

    public Trigger sysIdDrivetrainQuasistaticReverse() {
        return noButton;
    }

    public Trigger startSignalLogger() {
        return noButton;
    }

    public Trigger stopSignalLogger() {
        return noButton;
    }

    public OptionalDouble getTestJogValue() {
        return OptionalDouble.empty();
    }
}
