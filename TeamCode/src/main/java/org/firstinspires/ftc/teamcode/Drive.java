package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Drive", group="Exercises")
public class Drive extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        // Hardware Initialization
        DcMotor hang = hardwareMap.dcMotor.get("hang");

        DcMotor fl = hardwareMap.dcMotor.get("leftFront");
        DcMotor bl = hardwareMap.dcMotor.get("leftRear");
        DcMotor fr = hardwareMap.dcMotor.get("rightFront");
        DcMotor br = hardwareMap.dcMotor.get("rightRear");
        DcMotor slide = hardwareMap.dcMotor.get("slide");
        DcMotor arm = hardwareMap.dcMotor.get("arm");

        Servo twist = hardwareMap.servo.get("twist");
        Servo spin = hardwareMap.servo.get("spin");
        Servo grip = hardwareMap.servo.get("grip");

        // Motor Configuration
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.REVERSE);

        // Encoder Reset and Initial Setup
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Variables for Positions and Limits
        int sMax = -20000;
        int sMin = 5000;
        int sPos = slide.getCurrentPosition();
        int aMax = 0;
        int aMin = 6000;
        int aPos = arm.getCurrentPosition();
        int tin = 2000;

        // Boolean States
        boolean sm = true;
        boolean sb = true;
        boolean m1 = false, m2 = false, m3 = false, m4 = false;
        boolean hc = false;

        waitForStart();

        grip.setPosition(0);

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            // Calculated Variables
            int tpa = tin / 90;
            int aa = (arm.getCurrentPosition() != 0) ? (arm.getCurrentPosition() / tpa) : 1;

            // Gamepad 2 Controls (Manual)
            if (gamepad2.x) hc = true;
            else if (gamepad2.b) hc = false;

            if (hc) {
                if (gamepad2.dpad_up) twist.setPosition(twist.getPosition() - 0.001);
                else if (gamepad2.dpad_down) twist.setPosition(twist.getPosition() + 0.001);

                if (gamepad2.dpad_left) spin.setPosition(spin.getPosition() - 0.01);
                else if (gamepad2.dpad_right) spin.setPosition(spin.getPosition() + 0.01);

                if (gamepad2.y) grip.setPosition(0);
                if (gamepad2.a) grip.setPosition(1);
                if (gamepad2.x) twist.setPosition((0.2 + (aa / 300.0)) / 5.0);

            } else {
                if (arm.getCurrentPosition() <= 500) {
                    sPos = 650;
                    twist.setPosition(0.525 / 5.0);
                } else {
                    if (sm) {
                        twist.setPosition((aa / 300.0) / 5.0);
                    } else {
                        twist.setPosition((0.5 + (aa / 300.0)) / 5.0);
                    }
                }

                if (gamepad1.y || gamepad2.y) grip.setPosition(0);
                if (gamepad1.a || gamepad2.a) grip.setPosition(1);
                if (gamepad1.dpad_down) sm = false;
                if (gamepad1.dpad_up) sm = true;
                if (gamepad1.dpad_right) sb = false;
                if (gamepad1.dpad_left) sb = true;

                spin.setPosition(sb ? 0.7 : 0);
            }

            // Slider Controls
            if (aa > 97) sMax = -6600;
            else sMax = -4000;

            if (gamepad1.right_trigger >= 0.1 && sPos < sMin) {
                slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                slide.setPower(gamepad1.right_trigger);
                sPos = slide.getCurrentPosition();
            } else if (gamepad1.left_trigger >= 0.1 && sPos > sMax) {
                slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                slide.setPower(-gamepad1.left_trigger);
                sPos = slide.getCurrentPosition();
            } else {
                slide.setPower(0);
                slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }

            // Arm Controls
            if (gamepad1.right_bumper && aPos < aMin) {
                arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                arm.setPower(1);
                aPos = arm.getCurrentPosition();
            } else if (gamepad1.left_bumper && aPos > aMax) {
                arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                arm.setPower(-1);
                aPos = arm.getCurrentPosition();
            } else {
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setTargetPosition(aPos);
                arm.setPower(1);
            }

            // Preset Positions
            if (gamepad1.left_stick_button) {
                aPos = 2500;
                m1 = true;
            }
            if (m1 && Math.abs(aPos - arm.getCurrentPosition()) <= 400) {
                sPos = -2500;
                m1 = false;
            }

            if (gamepad1.right_stick_button) {
                aPos = 4400;
                m2 = true;
            }
            if (m2 && Math.abs(aPos - arm.getCurrentPosition()) <= 400) {
                sPos = 420;
                m2 = false;
            }

            // Telemetry
            telemetry.addData("Arm Value", arm.getCurrentPosition());
            telemetry.addData("Arm Angle", aa);
            telemetry.addData("Arm Angle on a 1 scale", aa / 300.0);
            telemetry.addData("Claw Angle", twist.getPosition() * 300);
            telemetry.addData("Claw Angle on a 1 scale", twist.getPosition());
            telemetry.addData("Slider Value", sPos);
            telemetry.addData("rightFront", fr.getCurrentPosition());
            telemetry.addData("rightRear", br.getCurrentPosition());
            telemetry.addData("leftFront", fl.getCurrentPosition());
            telemetry.addData("leftRear", bl.getCurrentPosition());
            telemetry.update();

            // Drive Controls
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            fl.setPower(y + x + rx);
            bl.setPower(y - x + rx);
            fr.setPower(y - x - rx);
            br.setPower(y + x - rx);

            // Hanger Controls
            if (gamepad2.right_bumper) hang.setPower(1);
            else if (gamepad2.left_bumper) hang.setPower(-1);
            else hang.setPower(0);
        }
    }
}
