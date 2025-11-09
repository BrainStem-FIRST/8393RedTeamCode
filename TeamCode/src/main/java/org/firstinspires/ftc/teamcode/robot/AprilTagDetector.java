package org.firstinspires.ftc.teamcode.robot;

import android.annotation.SuppressLint;
import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.Arrays;

@Config
public class AprilTagDetector {
    public static int cameraResX = 1600, cameraResY = 1200;
    public int[] fieldTags = new int[] {20, 24};
    //Focals (pixels) - Fx: 628.438 Fy: 628.438
    //Optical center - Cx: 986.138 Cy: 739.836

    private final Robot robot;
    private final AprilTagProcessor tagProcessor;
    private final VisionPortal visionPortal;
    private ArrayList<AprilTagDetection> detections;
    private final ArrayList<String> detectionData;
    private double x, y, a; // predicted x y and angle from april tags
    public AprilTagDetector(Robot robot) {
        this.robot = robot;
        tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setTagLibrary(AprilTagGameDatabase.getDecodeTagLibrary())
                .setLensIntrinsics(628.438, 628.438, 986.138, 739.836)
                .build();
        visionPortal = new VisionPortal.Builder()
                .setCamera(robot.hardwareMap.get(WebcamName.class, "webcam"))
                .addProcessor(tagProcessor)
                .setCameraResolution(new Size(cameraResX, cameraResY))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();
        detections = new ArrayList<>();
        detectionData = new ArrayList<>();
    }

    public void update() {
        detections = tagProcessor.getDetections();
        detectionData.clear();
        if (!detections.isEmpty()) {
            for (int i = 0; i < detections.size(); i++) {
                AprilTagDetection tag = detections.get(i);
                if(tag.metadata != null)
                    detectionData.add(tag.id + tag.metadata.name);
                else
                    detectionData.add(tag.id + "");
            }
        }
        // 21: gpp, 22: pgp, 23: ppg
    }
    public AprilTagDetection getTag(int id) {
        for(AprilTagDetection tag : detections)
            if(tag.id == id)
                return tag;
        return null;
    }
    public String getTagData() {
        return Arrays.toString(detectionData.toArray());
    }
    @SuppressLint("DefaultLocale")
    public String getLocationData(int id) {
        StringBuilder str = new StringBuilder();
        AprilTagDetection tag = getTag(id);
        if (tag != null) {
            str.append(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", tag.ftcPose.x, tag.ftcPose.y, tag.ftcPose.z));
            str.append(String.format("PRY %6.1f %6.1f %6.1f  (deg)", tag.ftcPose.pitch, tag.ftcPose.roll, tag.ftcPose.yaw));
            str.append(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", tag.ftcPose.range, tag.ftcPose.bearing, tag.ftcPose.elevation));
        }
        return str.toString();
    }
}
