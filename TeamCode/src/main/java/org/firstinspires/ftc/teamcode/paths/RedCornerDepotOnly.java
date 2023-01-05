package org.firstinspires.ftc.teamcode.paths;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name = "RedCornerDepotOnly", group = "Auto")
public class RedCornerDepotOnly extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        final SampleMecanumDrive drivetrain = new SampleMecanumDrive(hardwareMap);

        final double halfPi = Math.toRadians(-90);
        final Pose2d initialPose = new Pose2d(36, 63, halfPi);

        drivetrain.setPoseEstimate(initialPose);
        final Trajectory depotPath = drivetrain.trajectoryBuilder(initialPose, halfPi)
                .splineToLinearHeading(new Pose2d(36, 23, halfPi), halfPi)
                .splineTo(new Vector2d(58, 12), 0)
                .build();

        waitForStart();
        if (isStopRequested()) return;

        drivetrain.followTrajectory(depotPath);
    }
}