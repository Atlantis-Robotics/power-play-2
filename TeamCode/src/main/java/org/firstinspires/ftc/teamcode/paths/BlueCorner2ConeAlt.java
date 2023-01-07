package org.firstinspires.ftc.teamcode.paths;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.claw.Claw;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.lift.Lift;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "BlueCorner2ConeAlt", group = "Auto")
public class BlueCorner2ConeAlt extends LinearOpMode{
    @Override
    public void runOpMode() throws InterruptedException {
        final SampleMecanumDrive drivetrain = new SampleMecanumDrive(hardwareMap);

        final DcMotor leftLiftMotor = hardwareMap.get(DcMotor.class, "leftLiftMotor");
        final DcMotor rightLiftMotor = hardwareMap.get(DcMotor.class, "rightLiftMotor");
        final Lift lift = new Lift(leftLiftMotor, rightLiftMotor);

        final Servo clawServo = hardwareMap.get(Servo.class, "clawServo");
        final Claw claw = new Claw(clawServo);

        final double radian90 = Math.toRadians(90);
        final Pose2d initialPose = new Pose2d(36, -63, radian90);

        drivetrain.setPoseEstimate(initialPose);
        //Robot goes up to medium junction from the initial starting position at the blue corner
        final TrajectorySequence driveUpToJunction = drivetrain.trajectorySequenceBuilder(initialPose)
                .lineToConstantHeading(new Vector2d(35, -35))
                .turn(Math.toRadians(45))
                .build();

        final TrajectorySequence driveCloserToJunction = drivetrain.trajectorySequenceBuilder(driveUpToJunction.end())
                .forward(4)//Changed from 5 to 5.5
                .build();

        final TrajectorySequence driveBackFromJunction = drivetrain.trajectorySequenceBuilder(driveCloserToJunction.end())
                .back(5.5)//Changed from 5 to 5.5
                .build();

        //Robot drives to depot from mid junction
        final TrajectorySequence driveToDepot = drivetrain.trajectorySequenceBuilder(driveBackFromJunction.end())
                .turn(Math.toRadians(-45))
                .lineToLinearHeading(new Pose2d(36, -12, Math.toRadians(90)))
                .turn(Math.toRadians(-90))
                .lineToLinearHeading(new Pose2d(59.25, -10.5, Math.toRadians(0)))//12.5 to 14.5
                .build();
        //Robot drives back to high junction from depot
        final TrajectorySequence driveBackToJunction = drivetrain.trajectorySequenceBuilder(driveToDepot.end())
                //.lineToConstantHeading(new Vector2d(50,-14.5))
                .back(17)
                .turn(Math.toRadians(-135))
                .lineToConstantHeading(new Vector2d(34, -16.5))
                .build();

        final TrajectorySequence driveBackFromSecondJunction = drivetrain.trajectorySequenceBuilder(driveBackToJunction.end())
                //Drive back from 2nd medium junction
                .back(5)
                .build();



        waitForStart();
        if (isStopRequested()) return;

        lift.resetEncoder();


        claw.clawClose();
        sleep(1000);
        lift.liftToJunction(2);
        sleep(2000);
        drivetrain.followTrajectorySequence(driveUpToJunction);

        drivetrain.followTrajectorySequence(driveCloserToJunction);
        sleep(1000);
        claw.clawOpen();
        sleep(500);

        drivetrain.followTrajectorySequence(driveBackFromJunction);
        lift.liftToConeStack(3); // 5 to 4 to 3 so that lift can come lower and pick the cone more properly

        drivetrain.followTrajectorySequence(driveToDepot);
        //lift.liftToConeStack(3); // 5 to 4 to 3 so that lift can come lower and pick the cone more properly
        sleep(1000);
        claw.clawClose();
        sleep(1000);
        lift.liftToJunction(2);
        sleep(1000);
        drivetrain.followTrajectorySequence(driveBackToJunction);
        sleep(2000);
        lift.liftToJunction(1);
        claw.clawOpen();
        sleep(500);
        drivetrain.followTrajectorySequence(driveBackFromSecondJunction);
        lift.AutoRetract();
        sleep(3000);
    }

}
