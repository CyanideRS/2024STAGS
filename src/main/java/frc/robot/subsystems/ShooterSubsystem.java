package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase{
    private final CANSparkMax shooterAngle;
    private final CANSparkMax shooterTop;
    private final CANSparkMax shooterBottom;

    public ShooterSubsystem() {
        shooterAngle = new CANSparkMax(Constants.ShooterConstants.ShooterAngleMotorID, CANSparkLowLevel.MotorType.kBrushless);
        shooterAngle.restoreFactoryDefaults();
        shooterAngle.setIdleMode(CANSparkMax.IdleMode.kBrake);
        shooterAngle.setInverted(false);
        shooterTop = new CANSparkMax(Constants.ShooterConstants.ShooterTopMotorID, CANSparkLowLevel.MotorType.kBrushless);
        shooterTop.restoreFactoryDefaults();
        shooterTop.setIdleMode(CANSparkMax.IdleMode.kBrake);
        shooterTop.setInverted(false);
        shooterBottom = new CANSparkMax(Constants.ShooterConstants.ShooterBottomMotorID, CANSparkLowLevel.MotorType.kBrushless);
        shooterBottom.restoreFactoryDefaults();
        shooterBottom.setIdleMode(CANSparkMax.IdleMode.kBrake);
        shooterBottom.setInverted(true);
    }
    public void shoot(double power) {
        shooterTop.set(power);
        shooterBottom.set(power);
    }
    public void stop() {
        shooterTop.set(0);
        shooterBottom.set(0);
    }
    public void rotateShooter(double power) {
        shooterAngle.set(power);
    }
    public Command manualShoot(double power) {
        return run(() -> shoot(power));
    }
    public Command stopShooter() {
        return run(() -> stop());
    }
}
