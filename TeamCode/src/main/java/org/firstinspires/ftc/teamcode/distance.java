package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "distance", group = "Autonomous")
public class distance extends LinearOpMode {

    // CHANGE FOR YOUR ROBOT
    public static double CAMERA_HEIGHT_IN = 2.5;   // lens height off floor
    public static double TARGET_HEIGHT_IN = 25.5;   // center of tag off floor
    public static double CAMERA_PITCH_DEG = 110.0;   // camera tilt angle up

    Limelight3A cam;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        cam = hardwareMap.get(Limelight3A.class, "limelight");

        waitForStart();

        while (opModeIsActive()) {

            LLResult result = cam.getLatestResult();
            if (result != null) {

                double ty = result.getTy();   // raw vertical angle offset from limelight (degrees)
                double tx = result.getTx();   // raw horizontal angle offset (degrees)
                double ta = result.getTa();   // raw target area (unitless % image size)

                double verticalAngle = CAMERA_PITCH_DEG + ty;

                // Horizontal distance to AprilTag (floor → tag plane)
                double distanceToTagInches =
                        (TARGET_HEIGHT_IN - CAMERA_HEIGHT_IN) /
                                Math.tan(Math.toRadians(verticalAngle));

                // Distance straight down to floor under the tag = tag height
                double floorDistanceUnderTag = TARGET_HEIGHT_IN;

                telemetry.addData("ty (deg)", ty);
                telemetry.addData("tx (deg)", tx);
                telemetry.addData("ta (area)", ta);
                telemetry.addData("Distance → Tag (in)", distanceToTagInches);
                telemetry.addData("Floor Height Under Tag (in)", floorDistanceUnderTag);
            } else {
                // No data frame yet
                telemetry.addLine("Waiting for Limelight data...");
            }

            telemetry.update();
        }
    }
}
