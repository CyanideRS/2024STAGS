package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase{
    public static InterpolatingDoubleTreeMap distanceToAngleMap = new InterpolatingDoubleTreeMap();
    static {
      distanceToAngleMap.put(1.25, 0.0);//test
    }

    private final TalonFX shooterAngle = new TalonFX(Constants.ShooterConstants.ShooterAngleMotorID, "rio");
    private final CANSparkMax shooterTop;
    private final CANSparkMax shooterBottom;
    private final CANcoder shooterEncoder;


    public ShooterSubsystem() {
        shooterAngle.setNeutralMode(NeutralModeValue.Brake);

        shooterTop = new CANSparkMax(Constants.ShooterConstants.ShooterTopMotorID, CANSparkLowLevel.MotorType.kBrushless);
        shooterTop.setIdleMode(CANSparkMax.IdleMode.kBrake);
        shooterTop.setInverted(false);
        shooterTop.setSmartCurrentLimit(30);

        shooterBottom = new CANSparkMax(Constants.ShooterConstants.ShooterBottomMotorID, CANSparkLowLevel.MotorType.kBrushless);
        shooterBottom.setIdleMode(CANSparkMax.IdleMode.kBrake);
        shooterBottom.setInverted(true);
        shooterBottom.setSmartCurrentLimit(30);

        shooterEncoder = new CANcoder(25, "absolute"); //Absolute Encoder for Shooter Pivot
    }
    public void setShooterWheels(double power) {
        shooterTop.set(power);
        shooterBottom.set(power);
    }
    public void setTopWheel(double power) {
        shooterTop.set(power);
    }
    public void setBottomWheel(double power) {
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
    public double getShooterPosition() {
        var absolutePositionSignal = shooterEncoder.getAbsolutePosition();
        double absolutePositionValue = absolutePositionSignal.getValueAsDouble();
        return absolutePositionValue * 360;
    }

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
    @Override
    public void periodic()
    {
        SmartDashboard.putNumber("Shooter Position", getShooterPosition());
    }
    
}
