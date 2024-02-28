package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  private final CANSparkMax IntakeDeploy;
  private final CANSparkMax IntakeRoller;

  public IntakeSubsystem() {
    IntakeDeploy = new CANSparkMax(Constants.GroundIntakeConstants.IntakeDeployMotorID, CANSparkLowLevel.MotorType.kBrushless);
    IntakeDeploy.restoreFactoryDefaults();
    IntakeDeploy.setIdleMode(CANSparkMax.IdleMode.kBrake);
    IntakeDeploy.setInverted(false);
    IntakeRoller = new CANSparkMax(Constants.GroundIntakeConstants.IntakeRollerMotorID, CANSparkLowLevel.MotorType.kBrushless);
    IntakeRoller.restoreFactoryDefaults();
    IntakeRoller.setIdleMode(CANSparkMax.IdleMode.kCoast);
    IntakeRoller.setInverted(true);
  }
  public void intake(double power) {
      IntakeRoller.set(power);
  }
  public void stop() {
      IntakeRoller.set(0);
  }
  public Command floorIntake (double power) {
      return run(() -> intake(power));
  }
  public Command stopIntake () {
      return run(() -> stop());
  }    
}

