package frc.robot.controls;

import java.util.OptionalDouble;

import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public class TestingControls extends Controls {
    // same as Controls unless overridden

    // @Override
    // public FieldCentric getDriveRequest() {
    //     return new FieldCentric(); // no drive!
    // }

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
    public OptionalDouble getTestJogValue() {
        return OptionalDouble.of(-operator.getRightY());
    }
}
