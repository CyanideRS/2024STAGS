// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SuperStructure;

// import frc.robot.subsystems.CANLauncher;

/*This is an example of creating a command as a class. The base Command class provides a set of methods that your command
 * will override.
 */
public class PSIntake extends Command {
  ShooterSubsystem m_shooter;
  SuperStructure m_superstructure;

  /** Creates a new LaunchNote. */
  public PSIntake(ShooterSubsystem shooter, SuperStructure superstructure) {
    // save the launcher system internally
    m_shooter = shooter;
    m_superstructure = superstructure;

    // indicate that this command requires the launcher system
    addRequirements(m_shooter, m_superstructure);
  }

  // The initialize method is called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_superstructure.lightSensor.get()) {
      m_superstructure.setFeeder(-0.4);
      m_shooter.setShooterWheels(-0.3);
    } else {
      Timer.delay(0.5);
      m_superstructure.stop();
      m_shooter.stopShooterWheels();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Always return false so the command never ends on it's own. In this project we use the
    // scheduler to end the command when the button is released.
    return false;
  }
  @Override
  public void end(boolean interrupted) {
    // Stop the wheels when the command ends.
      m_superstructure.stop();
      m_shooter.stopShooterWheels();
  }
}
