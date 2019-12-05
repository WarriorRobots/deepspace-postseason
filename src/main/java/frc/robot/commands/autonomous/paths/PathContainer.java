package frc.robot.commands.autonomous.paths;

import frc.lib.control.Path;

/**
 * Interface containing all information necessary for a path including the Path itself, the Path's starting pose, and
 * whether or not the robot should drive in reverse along the path.
 */
public interface PathContainer {
    Path buildPath();

    boolean isReversed();
}
