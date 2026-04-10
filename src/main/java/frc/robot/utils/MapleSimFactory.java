// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.motorsims.SimulatedBattery;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * Factory class for creating Maple-Sim components for your 2026 robot.
 * 
 * This class provides helper methods to access Maple-Sim components.
 * 
 * For creating custom SwerveDriveSimulation instances, reference the Maple-Sim documentation:
 * https://shenzhen-robotics-alliance.github.io/maple-sim/swerve-simulation-overview/
 * 
 * Example:
 * <pre>
 * final DriveTrainSimulationConfig config = DriveTrainSimulationConfig.Default();
 * SwerveDriveSimulation drivetrainSim = new SwerveDriveSimulation(
 *     config,
 *     new Pose2d(0, 0, new Rotation2d())
 * );
 * MapleSimManager.initialize(drivetrainSim);
 * </pre>
 */
public class MapleSimFactory {

    /**
     * Gets the default simulated battery voltage.
     * 
     * @return 12.0 volts (typical FRC battery voltage)
     */
    public static double getDefaultBatteryVoltage() {
        return 12.0;
    }

    /**
     * Creates a simulated battery with default characteristics.
     * 
     * @return A SimulatedBattery instance
     */
    public static SimulatedBattery createSimulatedBattery() {
        return new SimulatedBattery();
    }

    /**
     * Gets the singleton SimulatedArena instance for accessing the physics simulation.
     * 
     * @return The SimulatedArena instance
     */
    public static SimulatedArena getSimulatedArena() {
        return SimulatedArena.getInstance();
    }

    /**
     * Checks if the simulation is running.
     * 
     * @return true if in simulation mode, false otherwise
     */
    public static boolean isSimulation() {
        return RobotBase.isSimulation();
    }
}
