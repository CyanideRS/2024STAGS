package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;


public class FeederSubsystem extends SubsystemBase {
    private final CANSparkMax feederRoller;

    public FeederSubsystem() {
        feederRoller = new CANSparkMax(Constants.FeederConstants.FeederRollerMotorID, CANSparkLowLevel.MotorType.kBrushless);
        feederRoller.restoreFactoryDefaults();
        feederRoller.setIdleMode(CANSparkMax.IdleMode.kCoast);
    }
    public void feed(double power) {
        feederRoller.set(power);
    }
    public void stop() {
        feederRoller.set(0);
    }
    public Command runFeeder (double power) {
        return run(() -> feed(power));
    }
    public Command stopFeeder () {
        return run(() -> stop());
    }
}
