package org.firstinspires.ftc.teamcode.pedroPathing.examples;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.*;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;



@Autonomous(name = "Auto Red Sample", group = "Examples")
public class RedBucket extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;




    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private int pathState;


    private final Pose startPose = new Pose(135, 50, Math.toRadians(0));

    private final Pose scorePose = new Pose(85, 70, Math.toRadians(0));

    private final Pose parkPose = new Pose(135, 25, Math.toRadians(0));



    private Path scorePreload, park;


    public void buildPaths() {

        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(scorePose)));
        scorePreload.setLinearHeadingInterpolation(Math.toRadians(180),Math.toRadians(180));

        park = new Path(new BezierLine(new Point(scorePose), new Point(parkPose)));
        park.setLinearHeadingInterpolation(Math.toRadians(180),Math.toRadians(180));

    }
    public void autonomousPathUpdate() {
        DcMotor slide = hardwareMap.dcMotor.get("slide");
        DcMotor arm = hardwareMap.dcMotor.get("arm");
        Servo twist = hardwareMap.servo.get("twist");
        Servo spin = hardwareMap.servo.get("spin");
        Servo grip = hardwareMap.servo.get("grip");

        int ap1 = 2200;
        int sp1 = 1850;
        int ap2 = 0;
        int sp2 = 100;
        double tp1 = 0.072;

        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                setPathState(1);
                break;
            case 1:
                arm.setTargetPosition(ap1);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setPower(1);


                slide.setTargetPosition(sp1);
                slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slide.setPower(1);

                grip.setPosition(1);

                twist.setPosition(tp1);

                setPathState(2);
                break;

            case 2:
                if(arm.getCurrentPosition()+ 20 < ap1 || arm.getCurrentPosition()-20 > ap1 || slide.getCurrentPosition()+ 20 < sp1 || slide.getCurrentPosition()-20 > sp1) {
                    follower.followPath(scorePreload, true);
                    setPathState(3);
                }
                break;

            case 3:

                if(follower.getPose().getX() > (scorePose.getX() - 1) && follower.getPose().getY() > (scorePose.getY() - 1)) {
                    follower.followPath(park,true);
                    grip.setPosition(0);
                    setPathState(4);
                }
                break;
            case 4:
                if(follower.getPose().getX() > (parkPose.getX() - 1) && follower.getPose().getY() > (parkPose.getY() - 1)) {
                    arm.setTargetPosition(ap2);
                    arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    arm.setPower(0.5);


                    slide.setTargetPosition(sp2);
                    slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slide.setPower(0.5);
                }
                break;

            case 5:
                if(arm.getCurrentPosition()+ 20 < ap2 || arm.getCurrentPosition()-20 > ap2 || slide.getCurrentPosition()+ 20 < sp2 || slide.getCurrentPosition()-20 > sp2) {
                    setPathState(-1);
                    break;
                }

        }
    }

    /** These change the states of the paths and actions
     * It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {

        // These loop the movements of the robot
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();

        opmodeTimer.resetTimer();

        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        buildPaths();



        // Set the claw to positions for init

    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
    }
}