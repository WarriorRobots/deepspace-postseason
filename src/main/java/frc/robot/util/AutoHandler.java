package frc.robot.util;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.commands.autonomous.paths.AutoDrive;

/**
 * Add your docs here.
 */
public class AutoHandler {

    // object of this class
    private static AutoHandler instance = null;

    // Name of auto to run
    private String autoname = null;
    // The command to run for auto
    private Command autocommand = null;

    /**
     * @return The only instance of AutoHandler
     */
    public static AutoHandler getInstance() {
        if (instance == null) {
            instance = new AutoHandler();
        }
        return instance;
    }

    /**
     * Have AutoHandler select a case based on dashboard values
    */
    public void selectCase() {
        autoname = "TESTLeft90"; // XXX temporary, have the value based on inputs from the dashboard
        autocommand = new AutoDrive(autoname);
    }

    /**
     * @return The command the AutoHandler has chosen
     */
    public Command getCase() {
        return autocommand;
    }

    //public void startAuto() {}

    //public void stopAuto() {}

    /**
     * Set AutoHandler data from dashbaord
     */
    public void getDashboardData () {}

    /**
     * Resets the data in the AutoHandler
     */
    public void reset() {
        autoname = null;
        autocommand = null;
    }
}
