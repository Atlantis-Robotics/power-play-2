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

@Autonomous(name = "RedCorner2ConeAlt", group = "Auto")
public class RedCorner2ConeAlt extends LinearOpMode{
    @Override
    public void runOpMode() throws InterruptedException {
        final SampleMecanumDrive drivetrain = new SampleMecanumDrive(hardwareMap);

        final DcMotor leftLiftMotor = hardwareMap.get(DcMotor.class, "leftLiftMotor");
        final DcMotor rightLiftMotor = hardwareMap.get(DcMotor.class, "rightLiftMotor");
        final Lift lift = new Lift(leftLiftMotor, rightLiftMotor);

        final Servo clawServo = hardwareMap.get(Servo.class, "clawServo");
        final Claw claw = new Claw(clawServo);

        final double radian90 = Math.toRadians(90);
        final Pose2d initialPose = new Pose2d(-36, -63, radian90);

        drivetrain.setPoseEstimate(initialPose);
        //Robot goes up to high junction from the initial starting position at the blue corner
        final TrajectorySequence driveUpToJunction = drivetrain.trajectorySequenceBuilder(initialPose)
                .lineToConstantHeading(new Vector2d(-36, -34.5))
                .turn(Math.toRadians(-45))
                .build();

        final TrajectorySequence driveCloserToJunction = drivetrain.trajectorySequenceBuilder(driveUpToJunction.end())
                .forward(5.1)//Changed from 5 to 5.3
                .build();

        final TrajectorySequence driveBackFromJunction = drivetrain.trajectorySequenceBuilder(driveCloserToJunction.end())
                .back(5.4)//Changed from 5 to 5.5
                .build();

        //Robot drives to depot from mid junction
        final TrajectorySequence driveToDepot = drivetrain.trajectorySequenceBuilder(driveBackFromJunction.end())
                .turn(Math.toRadians(45))
                .lineToLinearHeading(new Pose2d(-36, -12, Math.toRadians(90)))
                .turn(Math.toRadians(90))
                .lineToLinearHeading(new Pose2d(-61, -12, Math.toRadians(180)))//12.5 to 14.5
                .build();

        //Robot drives back to mid junction from depot
        final TrajectorySequence driveBackToJunction = drivetrain.trajectorySequenceBuilder(driveToDepot.end())
                //.lineToConstantHeading(new Vector2d(50,-14.5))
                .back(18)
                .turn(Math.toRadians(135))
                .lineToConstantHeading(new Vector2d(-32, -16))
                .build();

        final TrajectorySequence backUpAfterLastJunction = drivetrain.trajectorySequenceBuilder(driveBackToJunction.end())
                 .back(3)
                .build();

        waitForStart();
        if (isStopRequested()) return;

        lift.resetEncoder();


        claw.clawClose();
        sleep(1000);
        lift.liftToJunction(2);
        sleep(1000);
        drivetrain.followTrajectorySequence(driveUpToJunction);

        drivetrain.followTrajectorySequence(driveCloserToJunction);
        sleep(1000);
        claw.clawOpen();
        sleep(500);

        drivetrain.followTrajectorySequence(driveBackFromJunction);
        lift.retract();
        sleep(1000);
        lift.liftToConeStack(3);

        drivetrain.followTrajectorySequence(driveToDepot);
        sleep(1000);
        claw.clawClose();
        sleep(1000);
        lift.liftToJunction(2);
       sleep(1000);
        drivetrain.followTrajectorySequence(driveBackToJunction);
        sleep(1000);
        claw.clawOpen();
        sleep(1000);
        drivetrain.followTrajectorySequence(backUpAfterLastJunction);
        lift.retract();
        sleep(2000);
        lift.retract();

    }

}
