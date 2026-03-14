package frc.robot.subsystems;

import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

public class Drivetrain extends CommandSwerveDrivetrain {
    private SwerveDriveState currentState;

    public Drivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, modules);

        currentState = getState();
    }

    @Override
    public void periodic() {
        super.periodic();

        currentState = getState();
    }

    public SwerveDriveState getCachedState() {
        return currentState;
    }
}
