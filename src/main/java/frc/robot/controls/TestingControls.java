package frc.robot.controls;

import java.util.OptionalDouble;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer;

public class TestingControls extends Controls {
    // same as Controls unless overridden

    // @Override
    // public FieldCentric getDriveRequest() {
    //     return new FieldCentric(); // no drive!
    // }

    @Override
    public FieldCentric getDriveRequest() {
        if (driver.povUp().getAsBoolean()) {
            return drive.withVelocityX(.1 * RobotContainer.MaxSpeed);
        }

        if (driver.povDown().getAsBoolean()) {
            return drive.withVelocityX(-.1 * RobotContainer.MaxSpeed);
        }

        // TODO Auto-generated method stub
        return super.getDriveRequest();
    }

    @Override
    public Trigger driveAndFaceTarget() {
        return noButton;
    }

    @Override
    public Trigger sysIdDrivetrainDynamicForward() {
        return driver.back().and(driver.y());
    }

    @Override
    public Trigger sysIdDrivetrainDynamicReverse() {
        return driver.back().and(driver.x());
    }

    @Override
    public Trigger sysIdDrivetrainQuasistaticForward() {
        return driver.start().and(driver.y());
    }

    @Override
    public Trigger sysIdDrivetrainQuasistaticReverse() {
        return driver.start().and(driver.x());
    }

    @Override
    public Trigger startSignalLogger() {
        return driver.start().and(driver.a());
    }

    @Override
    public Trigger stopSignalLogger() {
        return driver.back().and(driver.b());
    }

    @Override
    public OptionalDouble getTestJogValue() {
        return OptionalDouble.of(-operator.getRightY());
    }
}
