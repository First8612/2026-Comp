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
            .withDeadband(RobotContainer.MaxSpeed * 0.1)
            .withRotationalDeadband(RobotContainer.MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    // Driver ******************************
    public SwerveRequest.FieldCentric getDriveRequest() {
        return drive.withVelocityX(-driver.getLeftY() * RobotContainer.MaxSpeed) // Drive forward with negative Y
                                                                                 // (forward)
                .withVelocityY(-driver.getLeftX() * RobotContainer.MaxSpeed) // Drive left with negative X (left)
                .withRotationalRate(-driver.getRightX() * RobotContainer.MaxAngularRate); // Drive counterclockwise with
                                                                                          // negative X (left)
    }

    public Trigger driveAndFaceTarget() {
        return driver.x();
    }

    public Trigger horn() {
        return driver.rightStick();
    }

    public Trigger fieldReset() {
        return driver.leftBumper();
    }

    // Operator ******************************
    public Trigger intakeRetract() {
        return operator.axisGreaterThan(XboxController.Axis.kLeftY.value, 0.8);
    }

    public Trigger intakeExtend() {
        return operator.axisGreaterThan(XboxController.Axis.kLeftY.value, -0.8);
    }

    public Trigger shoot() {
        return operator.x();
    }

    public Trigger shootSimiple() {
        return operator.y();
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

    public OptionalDouble getTestJogValue() {
        return OptionalDouble.empty();
    }
}
