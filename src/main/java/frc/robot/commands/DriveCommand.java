package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.controls.Controls;
import frc.robot.subsystems.Drivetrain;

public class DriveCommand extends Command {
    private Drivetrain drivetrain;
    private Controls controls;

    public DriveCommand(Drivetrain drivetrain, Controls controls) {
        super();
        this.drivetrain = drivetrain;
        this.controls = controls;

        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        var request = controls.getDriveRequest();
        drivetrain.setControl(request);
    }
}
