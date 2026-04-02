package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PositionAccuracyEstimator extends SubsystemBase {
    private Pose2d robotPose;
    private List<Pose2d> lastVisionPoses = new ArrayList<>();
    private double estimation = 0;
    private double distanceAvg = 0;
    private Timer visionReadingTimer = new Timer();
    private Supplier<SwerveDriveState> robotStateSupplier;

    public PositionAccuracyEstimator(
            Supplier<SwerveDriveState> robotStateSupplier) {
        super();
        this.robotStateSupplier = robotStateSupplier;
        robotPose = robotStateSupplier.get().Pose;

        visionReadingTimer.start();
    }

    public void addVisionReading(Pose2d pose, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs) {
        visionReadingTimer.restart();

        var thisDist = robotPose.getTranslation().minus(pose.getTranslation()).getNorm();

        distanceAvg = 
            ((distanceAvg * 20)
            + thisDist) / 21;

        lastVisionPoses.add(pose);
    }

    public double getEstimation() {
        return estimation;
    }

    @Override
    public void periodic() {
        robotPose = robotStateSupplier.get().Pose;

        estimation = 100;
        estimation -= visionReadingTimer.get() * 10;
        estimation -= distanceAvg * 15;

        var visionDiffDist = 0.0;
        if (lastVisionPoses.size() > 1) {
            var pose1 = lastVisionPoses.get(0);
            var pose2 = lastVisionPoses.get(1);

            visionDiffDist = 
                pose1.getTranslation()
                .minus(pose2.getTranslation())
                .getNorm();

            estimation -= visionDiffDist * 30;
        }
       
        if (estimation < 0) {
            estimation = 0;
        }

        SmartDashboard.putNumber("PositionAccuracyEstimator/estimate", estimation);
        lastVisionPoses.clear();
    }
}
