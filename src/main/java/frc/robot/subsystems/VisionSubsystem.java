package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class VisionSubsystem extends SubsystemBase{
    private String limelightName = "KPLime";
    

    public VisionSubsystem() {
        double tx = LimelightHelpers.getTX(limelightName);
        double ty = LimelightHelpers.getTY(limelightName);
        double ta = LimelightHelpers.getTA(limelightName);
        boolean hasTarget = LimelightHelpers.getTV(limelightName);

        double txnc = LimelightHelpers.getTXNC(limelightName);
        double tync = LimelightHelpers.getTYNC(limelightName);

    }


    public void setLEDmode(int ledMode) {
        // switch(ledMode) {
        //     case 0:
        //         LimelightHelpers.setLEDMode_ForceOff(limelightName);
        //     break;
        //     case 1:
        //         LimelightHelpers.setLEDMode_ForceOn(limelightName);
        //     break;
        //     case 2:
        //         LimelightHelpers.setLEDMode_ForceBlink(limelightName);
        //     break;
        //     default: LimelightHelpers.setLEDMode_ForceOff(limelightName);
        // }
        //or...
        //I prefer this one as its simplier and less redundant
        switch (ledMode) {
            case 0 -> LimelightHelpers.setLEDMode_ForceOff(limelightName);
            case 1 -> LimelightHelpers.setLEDMode_ForceOn(limelightName);
            case 2 -> LimelightHelpers.setLEDMode_ForceBlink(limelightName);
            default -> LimelightHelpers.setLEDMode_ForceOff(limelightName);
        }
    }

    //this command allows to swap between pipelines ran in the limelight
    //I.E. retoflective, full color for driving, or aprilTag Tracking pipelines
    public void setPipelineIndex(int index) {
        LimelightHelpers.setPipelineIndex(limelightName, index);
    }

    public double getTXValue() {
        return LimelightHelpers.getTX(limelightName);
    }

    public double getTYValue() {
        return LimelightHelpers.getTY(limelightName);
    }

    public double getTAValue() {
        return LimelightHelpers.getTA(limelightName);
    }

    public boolean hasTarget() {
        return LimelightHelpers.getTV(limelightName);
    }



}
