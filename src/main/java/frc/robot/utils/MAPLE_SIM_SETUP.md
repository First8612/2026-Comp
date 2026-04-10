# Maple-Sim 2026 Integration Guide

This guide helps you set up Maple-Sim for your 2026 robot simulation.

## Quick Start

### 1. Verify Dependencies

Check that you have the Maple-Sim vendor dependency installed:
- File: `vendordeps/maple-sim-0.4.0-beta.json` ✓ (already present)

### 2. Create SwerveDriveSimulation in RobotContainer

In your `RobotContainer` constructor, create the simulation when in sim mode:

```java
package frc.robot;

import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.swerve.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.COTS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import static edu.wpi.first.units.Units.*;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.utils.MapleSimManager;

public class RobotContainer {
    public final Drivetrain drivetrain = TunerConstants.createDrivetrain();
    
    // ... other subsystems ...

    public RobotContainer() {
        // Initialize Maple-Sim simulation if in simulation mode
        if (RobotBase.isSimulation()) {
            initializeMapleSimulation();
        }
        
        // ... rest of initialization ...
    }

    /**
     * Initializes the Maple-Sim physics simulation.
     * Configure with your actual robot dimensions.
     */
    private void initializeMapleSimulation() {
        // Create drivetrain simulation configuration
        final DriveTrainSimulationConfig driveTrainConfig = DriveTrainSimulationConfig.Default()
            .withGyro(COTS.ofPigeon2())
            .withSwerveModule(COTS.ofMark4(
                DCMotor.getFalcon500(1),      // Drive motor
                DCMotor.getFalcon500(1),      // Steer motor
                COTS.WHEELS.COLSONS.cof,      // Wheel COF
                2))                            // L2 gear ratio (adjust if using L3, L4, etc.)
            // TODO: Update these dimensions to match your robot
            .withTrackLengthTrackWidth(Inches.of(24), Inches.of(24))
            .withBumperSize(Inches.of(30), Inches.of(30));
        
        // Create the swerve drive simulation
        SwerveDriveSimulation drivetrainSimulation = new SwerveDriveSimulation(
            driveTrainConfig,
            new Pose2d(0, 0, new Rotation2d())  // Starting position
        );
        
        // Initialize the MapleSimManager
        MapleSimManager.initialize(drivetrainSimulation);
    }
}
```

### 3. Update Shocky.robotPeriodic()

In `Shocky.java`, add the simulation update in `robotPeriodic()`:

```java
@Override
public void robotPeriodic() {
    // Update Maple-Sim simulation
    if (RobotBase.isSimulation()) {
        MapleSimManager.getInstance().update();
    }
    
    CommandScheduler.getInstance().run();
    m_robotContainer.robotPeriodic();
}
```

## Configuration Details

### Robot Dimensions

Update these values to match your actual 2026 robot:

```java
// Track length: Distance between front and back modules (X-axis)
.withTrackLengthTrackWidth(Inches.of(24), Inches.of(24))

// Bumper size: Total robot dimensions including bumpers
.withBumperSize(Inches.of(30), Inches.of(30))
```

### Motor Configuration

The default configuration uses:
- **Drive Motors**: Falcon 500
- **Steer Motors**: Falcon 500
- **Gyro**: Pigeon 2
- **Swerve Modules**: Mark 4 L2

If your robot uses different modules (L3, L4, etc.), adjust the gear ratio number:
```java
.withSwerveModule(COTS.ofMark4(
    DCMotor.getFalcon500(1),
    DCMotor.getFalcon500(1),
    COTS.WHEELS.COLSONS.cof,
    3))  // Change to 3 for L3, 4 for L4, etc.
```

## Accessing Simulation Data

### Get Arena Instance
```java
SimulatedArena arena = MapleSimManager.getInstance().getSimulatedArena();
```

### Get Drivetrain Simulation
```java
SwerveDriveSimulation driveSim = MapleSimManager.getInstance().getDrivetrainSimulation();
```

### Get Simulated Battery
```java
SimulatedBattery battery = MapleSimManager.getInstance().getSimulatedBattery();
```

## Running in Simulation

1. **In VS Code**: Use the "Simulate Robot Code" option from WPILib commands
2. **Robot Program**: Runs normally but uses simulated physics instead of real hardware
3. **SmartDashboard**: Shows simulated sensor values and robot state

## Troubleshooting

### Simulation Not Running
- Ensure `RobotBase.isSimulation()` returns true (check console output)
- Verify `MapleSimManager.initialize()` was called in `RobotContainer`
- Check that `update()` is called in `robotPeriodic()`

### Physics Seems Wrong
- Verify robot dimensions are correct in `withTrackLengthTrackWidth()` and `withBumperSize()`
- Check gear ratios match your actual modules
- Ensure motor types match your hardware (Falcon 500 vs Kraken vs others)

### Performance Issues
- Maple-Sim physics simulation is CPU-intensive
- Consider running on a powerful machine
- Check simulation update frequency (default is robot loop frequency)

## Documentation References

- **Maple-Sim**: https://shenzhen-robotics-alliance.github.io/maple-sim/
- **CTRE Phoenix 6**: https://v6.docs.ctr-electronics.com/
- **WPILib Units**: https://docs.wpilib.org/en/stable/docs/software/general-conventions/code-organization.html

## Next Steps

1. ✓ Add `MapleSimManager` and `MapleSimFactory` to your utils
2. ✓ Update `RobotContainer` to initialize simulation
3. ✓ Update `Shocky.robotPeriodic()` to call `update()`
4. Run simulation and test!

---

For more advanced features like intake simulation, projectile simulation, or opponent robot simulation, see the [Maple-Sim documentation](https://shenzhen-robotics-alliance.github.io/maple-sim/).
