package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase{
    private final CANSparkMax shooterAngle;
    private final CANSparkMax shooterTop;
    private final CANSparkMax shooterBottom;
    //private final CANcoder shooterPivot;


    public ShooterSubsystem() {
        shooterAngle = new CANSparkMax(Constants.ShooterConstants.ShooterAngleMotorID, CANSparkLowLevel.MotorType.kBrushless);
        shooterAngle.setIdleMode(CANSparkMax.IdleMode.kBrake);
        shooterAngle.setInverted(false);
        shooterTop = new CANSparkMax(Constants.ShooterConstants.ShooterTopMotorID, CANSparkLowLevel.MotorType.kBrushless);
        shooterTop.setIdleMode(CANSparkMax.IdleMode.kBrake);
        shooterTop.setInverted(false);
        shooterTop.setSmartCurrentLimit(30);
        shooterBottom = new CANSparkMax(Constants.ShooterConstants.ShooterBottomMotorID, CANSparkLowLevel.MotorType.kBrushless);
        shooterBottom.setIdleMode(CANSparkMax.IdleMode.kBrake);
        shooterBottom.setInverted(true);
        shooterBottom.setSmartCurrentLimit(30);
       // shooterPivot = new CANcoder(25);
    }
    public void setShooterWheels(double power) {
        shooterTop.set(power);
        shooterBottom.set(power);
    }
    public void stopShooterWheels() {
        shooterTop.set(0);
        shooterBottom.set(0);
    }
    public void rotatePivot(double power) {
        shooterAngle.set(power);
    }
    public void stopPivot() {
        shooterAngle.set(0);
    }
   // public void getShooterPosition() {
        //var absolutePositionSignal = shooterPivot.getAbsolutePosition();
        //var absolutePositionValue = absolutePositionSignal.getUnits();
   // }

    public Command manualShoot(double power) {
        return run(() -> setShooterWheels(power));
    }
    public Command stopShooter() {
        return run(() -> stopShooterWheels()).withTimeout(1);
    }
    public Command rotateShooter(double power) {
        return this.startEnd(
            () -> {
                rotatePivot(power);
            },
            () -> {
                stopPivot();
            });
    }
    
}
