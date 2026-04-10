package frc.robot.utils;

/**
 * MAPLE-SIM 2026 QUICK START CHECKLIST
 * 
 * This is a documentation reference file. See MAPLE_SIM_SETUP.md for full details.
 * 
 * ==============================================================================
 * QUICK SETUP (3 STEPS)
 * ==============================================================================
 * 
 * Step 1: Add to RobotContainer constructor
 *   if (RobotBase.isSimulation()) {
 *       initializeMapleSimulation();
 *   }
 * 
 * Step 2: Add method to RobotContainer
 *   private void initializeMapleSimulation() {
 *       final DriveTrainSimulationConfig config = 
 *           DriveTrainSimulationConfig.Default()
 *           .withTrackLengthTrackWidth(Inches.of(24), Inches.of(24))
 *           .withBumperSize(Inches.of(30), Inches.of(30));
 *       
 *       SwerveDriveSimulation sim = new SwerveDriveSimulation(
 *           config, new Pose2d(0, 0, new Rotation2d()));
 *       
 *       MapleSimManager.initialize(sim);
 *   }
 * 
 * Step 3: Add to Shocky.robotPeriodic()
 *   if (RobotBase.isSimulation()) {
 *       MapleSimManager.getInstance().update();
 *   }
 * 
 * ==============================================================================
 * CUSTOMIZATION
 * ==============================================================================
 * Update your robot dimensions in initializeMapleSimulation():
 *   Inches.of(24) -> Your trackLength (front to back centers)
 *   Inches.of(24) -> Your trackWidth (left to right centers)
 *   Inches.of(30) -> Your bumperLengthX (total length)
 *   Inches.of(30) -> Your bumperLengthY (total width)
 * 
 * ==============================================================================
 * RUN SIMULATION
 * ==============================================================================
 * Command line:  ./gradlew simulateJava
 * VS Code:       Ctrl+Shift+P -> "Simulate Robot Code"
 * 
 * Documentation: https://shenzhen-robotics-alliance.github.io/maple-sim/
 */
public class MAPLE_SIM_QUICK_REFERENCE {
    // Documentation only - no implementation needed
}

