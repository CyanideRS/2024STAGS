package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;


public class ClimberSubsystem extends SubsystemBase {
    private final CANSparkMax leftClimber;
    private final CANSparkMax rightClimber;

    public ClimberSubsystem() {
        leftClimber = new CANSparkMax(Constants.ClimberConstants.ClimberLeftMotorID, CANSparkLowLevel.MotorType.kBrushless);
        leftClimber.restoreFactoryDefaults();
        leftClimber.setIdleMode(CANSparkMax.IdleMode.kBrake);
        rightClimber = new CANSparkMax(Constants.ClimberConstants.ClimberRightMotorID, CANSparkLowLevel.MotorType.kBrushless);
        rightClimber.restoreFactoryDefaults();
        rightClimber.setIdleMode(CANSparkMax.IdleMode.kBrake);
    }
    public void runLeftClimber(double power) {
        leftClimber.set(power);
    }
    public void stopLeftClimber() {
        leftClimber.set(0);
    }
    public void runRightClimber(double power) {
        rightClimber.set(power);
    }
    public void stopRightClimber() {
        rightClimber.set(0);
    }
    public Command runLeft(double power) {
        return this.startEnd(
            () -> {
                runLeftClimber(power);
            },
            () -> {
                stopLeftClimber();
            });
    }
    public Command runRight(double power) {
        return this.startEnd(
            () -> {
                runRightClimber(power);
            },
            () -> {
                stopRightClimber();
            });
    }
}