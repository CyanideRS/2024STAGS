package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

public class SuperStructure extends SubsystemBase {
    private final CANSparkMax feederRoller;
    public final DigitalInput lightSensor = new DigitalInput(1);
    IntakeSubsystem m_IntakeSubsystem;

    public SuperStructure() {
        feederRoller = new CANSparkMax(Constants.FeederConstants.FeederRollerMotorID, CANSparkLowLevel.MotorType.kBrushless);
        feederRoller.setIdleMode(CANSparkMax.IdleMode.kCoast);
        feederRoller.setInverted(true);
    }
    public void setFeeder(double power) {
        feederRoller.set(power);
    }
    public void stop() {
        feederRoller.set(0);
    }
    public Command stopFeeder () {
        return run(() -> stop()).withTimeout(1);
    }
}
