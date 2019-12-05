/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autonomous.pathsWW.routines;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.autonomous.pathsWW.AutoStraightDistance;
import frc.robot.commands.autonomous.CameraStopAtDistance;

public class Rocket extends CommandGroup {
  public Rocket(String autoname) {
    addSequential(new RocketDrive(autoname)); // drive to the target and turn the correct direction
    addSequential(new CameraStopAtDistance(true),1.5); // if it overshoots and doesn't backup, it should just shoot
    addSequential(new AutoStraightDistance(-26));
  }

  @Override
  public void initialize() {
    // debug for drivers to know when the auto is running
    SmartDashboard.putBoolean("Auto Running", true);
  }

  @Override
  public void end() {
    // debug for drivers to know when the auto is stopped
    SmartDashboard.putBoolean("Auto Running", false);
  }

  @Override
  public void interrupted() {
    end();
  }
}
