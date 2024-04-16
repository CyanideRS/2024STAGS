// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AmpShoot;
import frc.robot.commands.BothClimbers;
import frc.robot.commands.BothClimbersDown;
import frc.robot.commands.GroundIntake;
import frc.robot.commands.PSIntake;
import frc.robot.commands.PrepareAmpShoot;
import frc.robot.commands.PrepareSpeakerShoot;
import frc.robot.commands.SpeakerShoot;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import java.io.File;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here.
  private final IntakeSubsystem m_intake = new IntakeSubsystem();
  private final SuperStructure m_superstructure = new SuperStructure();
  private final ShooterSubsystem m_shooter = new ShooterSubsystem();
  private final ClimberSubsystem m_climber = new ClimberSubsystem();
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    NamedCommands.registerCommand("prepareShoot", new PrepareSpeakerShoot(m_shooter));
    NamedCommands.registerCommand("shootSpeaker", new SpeakerShoot(m_shooter, m_superstructure));
    NamedCommands.registerCommand("intakeGround", new GroundIntake(m_intake, m_superstructure));
    NamedCommands.registerCommand("stopShooter", m_shooter.stopShooter());
    NamedCommands.registerCommand("stopFeeder", m_superstructure.stopFeeder());
    NamedCommands.registerCommand("zeroGyro", Commands.runOnce(drivebase::zeroGyro));
    configureBindings();
  }
  private void configureBindings() {
    //Main Driver Controller
    Constants.DriverController.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
    Constants.DriverController.b().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
    Constants.DriverController.y().whileTrue(
        Commands.deferredProxy(() -> drivebase.driveToPose(
                                   new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
                              ));
    Constants.DriverController.leftTrigger().whileTrue(m_climber.runLeft(0.6));
    Constants.DriverController.rightTrigger().whileTrue(m_climber.runRight(0.6));
    Constants.DriverController.leftBumper().whileTrue(m_climber.runLeft(-0.6));
    Constants.DriverController.rightBumper().whileTrue(m_climber.runRight(-0.6));
    Constants.DriverController.povUp().whileTrue(new BothClimbers(m_climber));
    Constants.DriverController.povDown().whileTrue(new BothClimbersDown(m_climber));

    //Secondary Operator Controller
    Constants.OperatorController.y().whileTrue(new PSIntake(m_shooter, m_superstructure));
    Constants.OperatorController.x().whileTrue(new GroundIntake(m_intake, m_superstructure));
    Constants.OperatorController.b().whileTrue(
                                        new PrepareSpeakerShoot(m_shooter)
                                        .andThen(new WaitCommand(1))
                                        .andThen (new SpeakerShoot(m_shooter, m_superstructure))
                                        .andThen(new WaitCommand(1))
                                        .andThen(Commands.parallel(m_shooter.stopShooter(),m_superstructure.stopFeeder()))
                                        .handleInterrupt(() -> Commands.parallel(m_shooter.stopShooter(),m_superstructure.stopFeeder())));
    Constants.OperatorController.a().whileTrue(
                                        new PrepareAmpShoot(m_shooter)
                                        .andThen(new WaitCommand(1.0))
                                        .andThen (new AmpShoot(m_shooter, m_superstructure))
                                        .handleInterrupt(() -> m_shooter.stopShooter()));    
    Constants.OperatorController.leftBumper().onTrue(m_intake.rotateIntake(-0.2)).onFalse(m_intake.stopRotate());
    Constants.OperatorController.rightBumper().onTrue(m_intake.rotateIntake(0.2)).onFalse(m_intake.stopRotate());
    Constants.OperatorController.leftTrigger().whileTrue(m_shooter.rotateShooter(-0.15));
    Constants.OperatorController.rightTrigger().whileTrue(m_shooter.rotateShooter(0.15));
    Constants.OperatorController.povUp().whileTrue(new BothClimbers(m_climber));
    Constants.OperatorController.povDown().whileTrue(new BothClimbersDown(m_climber));
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    return new PathPlannerAuto("New Auto");
  }

  public void setDriveMode()
  {
    // Applies deadbands and inverts controls because joysticks are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation, right stick controls the desired angle NOT angular rotation
    Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
        () -> -MathUtil.applyDeadband(Constants.DriverController.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> -MathUtil.applyDeadband(Constants.DriverController.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> -Constants.DriverController.getRightX(),
        () -> -Constants.DriverController.getRightY());

    Command driveFieldOrientedDirectAngleSim = drivebase.simDriveCommand(
        () -> -MathUtil.applyDeadband(Constants.DriverController.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> -MathUtil.applyDeadband(Constants.DriverController.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> -Constants.DriverController.getRawAxis(2));

    drivebase.setDefaultCommand(
        !RobotBase.isSimulation() ? driveFieldOrientedDirectAngle : driveFieldOrientedDirectAngleSim);
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}