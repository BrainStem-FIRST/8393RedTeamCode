package org.firstinspires.ftc.teamcode.robot;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.utils.math.Vector3;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.Arrays;

@Config
public class WebcamAprilTagDetector {
    public static int cameraResX = 1920, cameraResY = 1200;
    public static double cx = 0, cz = 15.4331, cy = -3; // camera position relative to robot origin (x=left/right, y=up/down, z=forwards/backwards)
    public static double yaw = 0, pitch = 80, roll = 0; // camera axes orientation relative to robot heading vector (IN DEGREES)
    public final Position cameraPosition = new Position(DistanceUnit.INCH,cx, cy, cz, 0);
    public final YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, yaw, pitch, roll, 0);

    //Focals (pixels) - Fx: 599.162 Fy: 599.162
    //Optical center - Cx: 922.924 Cy: 624.331
    // error for these stats was 0.31

    //Focals (pixels) - Fx: 597.203 Fy: 597.203
    //Optical center - Cx: 923.189 Cy: 623.277
    // error for these stats was 0.27
    private final Robot robot;
    private final AprilTagProcessor tagProcessor;
    private final VisionPortal visionPortal;
    private ArrayList<AprilTagDetection> detections;
    private final ArrayList<String> detectionData;
    public final Vector3 robotPos, robotOrient, rawTagOffset;
    public WebcamAprilTagDetector(Robot robot) {
        this.robot = robot;
        tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setCameraPose(cameraPosition, cameraOrientation)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setTagLibrary(AprilTagGameDatabase.getDecodeTagLibrary())
                .setLensIntrinsics(597.203, 597.203, 923.189, 623.277)
                .build();
        tagProcessor.setPoseSolver(AprilTagProcessor.PoseSolver.OPENCV_SOLVEPNP_EPNP);
        visionPortal = new VisionPortal.Builder()
                .setCamera(robot.hardwareMap.get(WebcamName.class, "webcam"))
                .addProcessor(tagProcessor)
                .setCameraResolution(new Size(cameraResX, cameraResY))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();
        detections = new ArrayList<>();
        detectionData = new ArrayList<>();
        FtcDashboard.getInstance().startCameraStream(visionPortal, 60);
        robotPos = new Vector3(0, 0, 0);
        robotOrient = new Vector3(0, 0, 0); // pitch, yaw, and roll, respectively
        rawTagOffset = new Vector3(0, 0, 0);
    }

    public void updateDetections() {
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
    }
    public void updateLocalization() {
        AprilTagDetection tag = getTag(20);
        if(tag == null)
            return;
        rawTagOffset.set(tag.ftcPose.x, tag.ftcPose.y, tag.ftcPose.z);
        robotPos.set(tag.robotPose.getPosition().x, tag.robotPose.getPosition().y, tag.robotPose.getPosition().z);
        robotOrient.set(tag.robotPose.getOrientation().getPitch(AngleUnit.DEGREES), tag.robotPose.getOrientation().getYaw(AngleUnit.DEGREES), tag.robotPose.getOrientation().getRoll(AngleUnit.DEGREES));
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
    public double getCameraFps() {
        return visionPortal.getFps();
    }
    public double getAvgPoseCalcTime() {
        return tagProcessor.getPerTagAvgPoseSolveTime();
    }
}
