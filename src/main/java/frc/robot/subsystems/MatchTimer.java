package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.utils.NetworkTableGroup;

/**
 * Tracks match time and fires controller rumble cues at scheduled game events.
 *
 * <p>Call {@link #start()} at teleop init and {@link #stop()} at teleop exit.
 */
public class MatchTimer extends SubsystemBase {
    private final Timer gameTime = new Timer();
    private final double[] gameEvents = {
        /*Start 1st Shift*/  10,
        /*2nd Shift*/        35,
        /*3rd Shift*/        60,
        /*4th Shift*/        85,
        /*Start Endgame*/   110,
        /*End of Game*/     140
    };
    private final EventLoop loop = new EventLoop();
    private final NetworkTableGroup nt = new NetworkTableGroup("MatchTimer", true);

    private boolean onChange = false;
    private boolean aboutToChange = false;
    private boolean gameFinished = false;

    public final Trigger onChangeTrigger = new Trigger(() -> onChange);
    public final Trigger aboutToChangeTrigger = new Trigger(() -> aboutToChange);
    public final Trigger gameFinishedTrigger = new Trigger(() -> gameFinished);

    @Override
    public void periodic() {
        double t = gameTime.get();
        onChange = false;
        aboutToChange = false;
        gameFinished = false;
        for (double event : gameEvents) {
            if (t >= event - 3 && t < event - 2) aboutToChange = true;
            if (t >= event && t < event + 1) onChange = true;
        }

        if (t > 140) {
            gameFinished = true;
        }

        loop.poll();

        nt.putBoolean("onChange", onChange);
        nt.putBoolean("aboutToChange", aboutToChange);
        nt.putBoolean("gameFinished", gameFinished);
        nt.putNumber("time", t);
    }

    /** Start the match timer (call at teleop init). */
    public void start() {
        gameTime.restart();
    }

    /** Stop and reset the match timer (call at teleop exit). */
    public void stop() {
        gameTime.reset();
        gameTime.stop();
    }
}
