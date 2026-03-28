package frc.robot.subsystems;

import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

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
