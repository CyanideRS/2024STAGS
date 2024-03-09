// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

// import frc.robot.subsystems.CANLauncher;

public class BothClimbersDown extends Command {
  ClimberSubsystem m_climber;

  // CANLauncher m_launcher;

  /** Creates a new PrepareLaunch. */
  public BothClimbersDown(ClimberSubsystem climber) {
    // save the launcher system internally
    m_climber = climber;

    // indicate that this command requires the launcher system
    addRequirements(m_climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Set launch wheel to speed, keep feed wheel at 0 to let launch wheel spin up.
    m_climber.runLeftClimber(0.6);
    m_climber.runRightClimber(0.6);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // There is nothing we need this command to do on each iteration. You could remove this method
    // and the default blank method
    // of the base class will run.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Always return false so the command never ends on it's own. In this project we use a timeout
    // decorator on the command to end it.
    return false;
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Stop the wheels when the command ends.
    m_climber.stopLeftClimber();;
    m_climber.stopRightClimber();;
  }
}