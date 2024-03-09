package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  private final CANSparkMax IntakeDeploy;
  private final CANSparkMax IntakeRoller;
  private final DigitalInput intakeUpperLimit = new DigitalInput(0);
  private final DigitalInput intakeLowerLimit = new DigitalInput(2);

  public IntakeSubsystem() {
    IntakeDeploy = new CANSparkMax(Constants.GroundIntakeConstants.IntakeDeployMotorID, CANSparkLowLevel.MotorType.kBrushless);
    IntakeDeploy.setIdleMode(CANSparkMax.IdleMode.kBrake);
    IntakeDeploy.setInverted(false);
    IntakeRoller = new CANSparkMax(Constants.GroundIntakeConstants.IntakeRollerMotorID, CANSparkLowLevel.MotorType.kBrushless);
    IntakeRoller.setIdleMode(CANSparkMax.IdleMode.kCoast);
    IntakeRoller.setInverted(false);
  }
  public void feedIntake(double power) {
        IntakeRoller.set(power);
  }
  public void stop() {
    IntakeRoller.set(0);
  }
  public void moveIntake(double power) {
    if (power > 0) {
      if (intakeUpperLimit.get()) {
        IntakeDeploy.set(0);
      } else {
        IntakeDeploy.set(power);
      }
    } else {
      if (intakeLowerLimit.get()) {
        IntakeDeploy.set(0);
      } else {
        IntakeDeploy.set(power);
      }
    }
  }
  public void stopRotation() {
    IntakeDeploy.set(0);
  }
  public Command floorIntake (double power) {
    return run(() -> feedIntake(power));
  }
  public Command stopIntake () {
    return run(() -> stop());
  }    
  public Command rotateIntake(double power) {
    return run(() -> moveIntake(power));
  }
  public Command stopRotate() {
    return run(() -> stopRotation());
  }
}

