package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;


public class ClimberSubsystem extends SubsystemBase {
    private final TalonFX leftClimber = new TalonFX(Constants.ClimberConstants.ClimberLeftMotorID, "rio");
    private final TalonFX rightClimber = new TalonFX(Constants.ClimberConstants.ClimberRightMotorID, "rio");

    public ClimberSubsystem() {
        leftClimber.setNeutralMode(NeutralModeValue.Brake);
        rightClimber.setNeutralMode(NeutralModeValue.Brake);
        rightClimber.setInverted(true);
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