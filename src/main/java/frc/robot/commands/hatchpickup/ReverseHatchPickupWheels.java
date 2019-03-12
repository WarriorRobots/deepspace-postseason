package frc.robot.commands.hatchpickup;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

@Deprecated
public class ReverseHatchPickupWheels extends Command {

  /**
   * Runs the hatch pickup wheels in reverse. This can cause mechanical damage, so
   * do not use unless for debugging purposes.
   */
  public ReverseHatchPickupWheels() {
    requires(Robot.hatchPickupWheels);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.hatchPickupWheels.runPickup(-1);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  @Override
  protected void end() {
    Robot.hatchPickupWheels.stopPickup();
  }

}
