// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.wpilibj.RobotBase;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.motorsims.SimulatedBattery;

/**
 * Manages Maple-Sim initialization and periodic updates for robot simulation.
 * This class handles the simulation of the drivetrain and other mechanisms.
 * 
 * Usage in RobotContainer:
 * <pre>
 * SwerveDriveSimulation drivetrainSim = MapleSimFactory.createSwerveDriveSimulation(
 *     1.0,    // drivetrainLength
 *     1.0,    // drivetrainWidth  
 *     0.0508, // wheelRadius
 *     5.04    // maxModuleSpeed
 * );
 * MapleSimManager.initialize(drivetrainSim);
 * </pre>
 * 
 * Then in Shocky.robotPeriodic():
 * <pre>
 * if (RobotBase.isSimulation()) {
 *     MapleSimManager.getInstance().update();
 * }
 * </pre>
 */
public class MapleSimManager {
    private static MapleSimManager m_instance;

    private final SimulatedArena m_simulatedArena;
    private final SwerveDriveSimulation m_drivetrainSimulation;
    private final SimulatedBattery m_battery;

    private MapleSimManager(SwerveDriveSimulation drivetrainSimulation) {
        m_simulatedArena = SimulatedArena.getInstance();
        m_drivetrainSimulation = drivetrainSimulation;
        m_battery = new SimulatedBattery();

        // Add the drivetrain to the simulated arena
        m_simulatedArena.addDriveTrainSimulation(m_drivetrainSimulation);
    }

    /**
     * Initializes Maple-Sim with the given drivetrain simulation.
     * Call this once during robot initialization (recommended in RobotContainer constructor).
     *
     * @param drivetrainSimulation The SwerveDriveSimulation instance
     */
    public static void initialize(SwerveDriveSimulation drivetrainSimulation) {
        if (m_instance == null) {
            m_instance = new MapleSimManager(drivetrainSimulation);
        }
    }

    /**
     * Gets the singleton instance of MapleSimManager.
     * Must call initialize() first.
     *
     * @return The MapleSimManager instance
     */
    public static MapleSimManager getInstance() {
        if (m_instance == null) {
            throw new IllegalStateException("MapleSimManager not initialized. Call initialize() first.");
        }
        return m_instance;
    }

    /**
     * Updates the simulation. Call this periodically in robotPeriodic().
     * This should only be called when in simulation mode.
     */
    public void update() {
        if (RobotBase.isSimulation()) {
            m_simulatedArena.simulationPeriodic();
        }
    }

    /**
     * Gets the simulated arena instance.
     *
     * @return The SimulatedArena
     */
    public SimulatedArena getSimulatedArena() {
        return m_simulatedArena;
    }

    /**
     * Gets the drivetrain simulation instance.
     *
     * @return The SwerveDriveSimulation
     */
    public SwerveDriveSimulation getDrivetrainSimulation() {
        return m_drivetrainSimulation;
    }

    /**
     * Gets the simulated battery instance.
     *
     * @return The SimulatedBattery
     */
    public SimulatedBattery getSimulatedBattery() {
        return m_battery;
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
