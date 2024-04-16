package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants;
import frc.robot.Constants.LimelightConstants;
import java.util.HashMap;
import java.util.Map;

public class Limelight extends SubsystemBase{
    NetworkTable table;
    NetworkTableEntry tx;
    NetworkTableEntry ty;
    NetworkTableEntry ta;
    NetworkTableEntry tv;
    NetworkTableEntry ledMode;
    NetworkTableEntry camMode;
    NetworkTableEntry pipeline;

    ShuffleboardTab tab;
    GenericEntry distanceToTargetX;
    GenericEntry distanceToTargetY;
    GenericEntry targetVisible;

    String name;
    double limeHeight;
    double limeAngle;
    boolean enabled;
    int currentPipeline = -1;
    boolean isNetworkTableConnected;

    Translation2d offset;

    private static Limelight mainLimelight;

    public static Limelight getLimelightInstance() {
        if (mainLimelight == null) {
            mainLimelight =
                new Limelight(
                    LimelightConstants.limelightHeight,
                    LimelightConstants.limelightAngle,
                    LimelightConstants.limelightXOffsetMeters,
                    LimelightConstants.limelightYOffsetMeters,
                    LimelightConstants.limelightEnabled);
        }
        return mainLimelight;
    }

    private Limelight(
        double limelightHeightMeters,
        double limelightAngleDegrees,
        double xOffsetMeters,
        double yOffsetMeters,
        boolean enabled) {
    limeHeight = limelightHeightMeters;
    limeAngle = limelightAngleDegrees;
    offset = new Translation2d(xOffsetMeters, yOffsetMeters);
    this.enabled = enabled;

    if (enabled){
        table = NetworkTableInstance.getDefault().getTable(name);
        tx = table.getEntry("tx");
        ty = table.getEntry("ty");
        ta = table.getEntry("ta");
        tv = table.getEntry("tv");
        ledMode = table.getEntry("ledMode");
        camMode = table.getEntry("camMode");
        pipeline = table.getEntry("pipeline");

        if (Constants.debug) {
            tab = Shuffleboard.getTab(name);
            targetVisible =
                tab.add("Target Visible", false)
                    .withWidget(BuiltInWidgets.kBooleanBox)
                    .withPosition(0, 0)
                    .getEntry();
            distanceToTargetX = tab.add("Target X", 0).withPosition(0, 1).getEntry();
            distanceToTargetY = tab.add("Target Y", 0).withPosition(0, 2).getEntry();
          }
        }
    }

 /*   @Override
    public void periodic() {
        if (enabled) {
            if (Constants.debug) {
              boolean visible = getTargetVisible();
              targetVisible.setBoolean(visible);
              if (visible) {
                Translation2d targetPos = getTargetPosRobotRelative();
                distanceToTargetX.setDouble(targetPos.getX());
                distanceToTargetY.setDouble(targetPos.getY());
              }
            }
      
            if (table == null) {
              isNetworkTableConnected = false;
            } else {
              isNetworkTableConnected = true;
            }
        }
    }
    public boolean getTargetVisible() {
        if (enabled && isNetworkTableConnected) {
          return tv.getDouble(0.0) == 1.0;
        } else {
          return false;
        }
    }*/
}
