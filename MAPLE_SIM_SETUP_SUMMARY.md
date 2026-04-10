# Maple-Sim 2026 Setup Summary

## What I Created

I've set up the foundation for Maple-Sim integration in your 2026 robot code. Here's what was created:

### 1. **MapleSimManager.java** (Core Manager)
- **Location**: `src/main/java/frc/robot/utils/MapleSimManager.java`
- **Purpose**: Manages the Maple-Sim physics engine lifecycle
- **Key Methods**:
  - `initialize(SwerveDriveSimulation)` - Call once in RobotContainer
  - `update()` - Call every robotPeriodic() in Shocky
  - `getInstance()` - Access the singleton manager
  - `getSimulatedArena()` - Access the physics world
  - `getDrivetrainSimulation()` - Access the simulated drivetrain
  - `getSimulatedBattery()` - Access the virtual battery

### 2. **MapleSimFactory.java** (Helper Utilities)
- **Location**: `src/main/java/frc/robot/utils/MapleSimFactory.java`
- **Purpose**: Provides utility methods for simulation components
- **Methods**: Battery creation, arena access, simulation check

### 3. **MAPLE_SIM_SETUP.md** (Integration Guide)
- **Location**: `src/main/java/frc/robot/utils/MAPLE_SIM_SETUP.md`
- **Content**: Step-by-step setup instructions with code examples

## What You Need to Do

### Step 1: Update RobotContainer.java

Add this in your `RobotContainer` constructor (after `drivetrain` initialization):

```java
if (RobotBase.isSimulation()) {
    initializeMapleSimulation();
}
```

And add this method to `RobotContainer`:

```java
private void initializeMapleSimulation() {
    final org.ironmaple.simulation.drivesims.swerve.DriveTrainSimulationConfig config = 
        org.ironmaple.simulation.drivesims.swerve.DriveTrainSimulationConfig.Default()
        .withTrackLengthTrackWidth(Inches.of(24), Inches.of(24))
        .withBumperSize(Inches.of(30), Inches.of(30));
    
    org.ironmaple.simulation.drivesims.SwerveDriveSimulation sim = 
        new org.ironmaple.simulation.drivesims.SwerveDriveSimulation(
            config,
            new Pose2d(0, 0, new Rotation2d())
        );
    
    frc.robot.utils.MapleSimManager.initialize(sim);
}
```

> **TODO**: Replace `Inches.of(24)` and `Inches.of(30)` with your actual robot dimensions

### Step 2: Update Shocky.java

Add this to `robotPeriodic()`:

```java
@Override
public void robotPeriodic() {
    if (RobotBase.isSimulation()) {
        MapleSimManager.getInstance().update();
    }
    
    CommandScheduler.getInstance().run();
    m_robotContainer.robotPeriodic();
}
```

### Step 3: Add Imports to RobotContainer

```java
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.swerve.DriveTrainSimulationConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import static edu.wpi.first.units.Units.*;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.utils.MapleSimManager;
```

## What Happens When You Run Simulation

1. **Physics Engine Starts**: Maple-Sim initializes the physics world
2. **Virtual Robot Spawned**: Your drivetrain appears in the simulated arena
3. **CAN Bus Simulation**: Your CTRE motors respond as if running on the robot
4. **Sensor Simulation**: Gyro, encoders, and other sensors provide realistic data
5. **Real Drive Code**: Your actual drive code runs unchanged - it just talks to simulated hardware instead

## Testing Your Setup

After making these changes:

1. Open terminal and run: `./gradlew simulateJava`
2. Or use VS Code WPILib extension: Open Command Palette → "Simulate Robot Code"
3. SmartDashboard will show simulated robot state

## Customization Options

### Robot Dimensions
In the `.withTrackLengthTrackWidth()` call:
- First value (24 inches): Distance front-to-back between modules (X-axis)
- Second value (24 inches): Distance left-to-right between modules (Y-axis)

Measure from module center to module center on your actual robot.

### Bumper Size
In the `.withBumperSize()` call:
- First value (30 inches): Total length including bumpers
- Second value (30 inches): Total width including bumpers

### Motor Types
If using different motors (currently Falcon 500):
```java
DCMotor.getKrakenX60(1)  // For Kraken X60
DCMotor.getNEO(1)        // For REV NEO
DCMotor.getNeo550(1)     // For REV NEO 550
```

### Gear Ratios
Adjust the last parameter in `.withSwerveModule()`:
- `2` = L2 (6:1 drive)
- `3` = L3 (6.33:1 drive)
- `4` = L4 (8.14:1 drive)

## Next Steps

1. Measure your robot dimensions
2. Update the dimension values in `initializeMapleSimulation()`
3. Run the simulation
4. Test your autonomous and teleop code in simulation
5. Iterate and refine!

---

**Reference**: Full Maple-Sim documentation at https://shenzhen-robotics-alliance.github.io/maple-sim/
