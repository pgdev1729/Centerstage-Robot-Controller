package org.firstinspires.ftc.teamcode.competition.qualifiers.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class QualifiersTeleop extends LinearOpMode {
    int pixelsPossesed = 0;
    int slideLevelOne = 0;
    int slideLevelTwo = 0;
    int slideLevelThree = 0;

    int slideLevelGround = 0;

    float slideKi = 0;
    float slideKp = 0;
    float slideKd = 0;
    float slideReference = 0;

    DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
    DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
    DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
    DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
    DcMotor intakeMotor = hardwareMap.dcMotor.get("intakeMotor");
    DcMotor leftSlideMotor = hardwareMap.dcMotor.get("leftSlideMotor");
    DcMotor rightSlideMotor = hardwareMap.dcMotor.get("rightSlideMotor");

    int leftSlideMotorTicks = leftSlideMotor.getCurrentPosition();
    int rightSlideMotorTicks = leftSlideMotor.getCurrentPosition();
    @Override
    public void runOpMode() throws InterruptedException {



        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            if (pixelsPossesed <= 2){
                if (gamepad1.right_trigger > 0){
                    intakeMotor.setPower(gamepad1.right_trigger);
                }
            }

            if (gamepad2.dpad_left){
                pixelsPossesed = 1;
            }

            if (gamepad2.dpad_right){
                pixelsPossesed = 2;
            }

            if (gamepad2.dpad_up){
                pixelsPossesed = 0;
            }

            if (gamepad2.a){
                moveSlideToGround();
            }

            if (gamepad2.x){
                moveSlideToLevelOne();
            }

            if (gamepad2.b){
                moveSlideToLevelTwo();
            }

            if (gamepad2.y){
                moveSlideToLevelThree();
            }



            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);


        }
    }

    public void moveSlideToPositionPID(DcMotor leftSlideMotor, DcMotor rightSlideMotor, float ticks, float Kp, float Ki, float Kd, float reference){

        double integralSum = 0;

        float lastError = 0;

// Elapsed timer class from SDK, please use it, it's epic
        ElapsedTime timer = new ElapsedTime();

        while (leftSlideMotor.getCurrentPosition() != ticks) {


            // obtain the encoder position
            float encoderPosition = leftSlideMotor.getCurrentPosition();
            // calculate the error
            float error = reference - encoderPosition;

            // rate of change of the error
            float derivative = (float) ((error - lastError) / timer.seconds());

            // sum of all error over time
            integralSum = integralSum + (error * timer.seconds());

            double out = (Kp * error) + (Ki * integralSum) + (Kd * derivative);

            leftSlideMotor.setPower(out);
            rightSlideMotor.setPower(out);

            lastError = error;

            // reset the timer for next time
            timer.reset();

            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            if (pixelsPossesed <= 2){
                if (gamepad1.left_trigger > 0){
                    intakeMotor.setPower(-gamepad1.left_trigger);
                }
                if (gamepad1.right_trigger > 0){
                    intakeMotor.setPower(gamepad1.right_trigger);
                }
            }

            if (gamepad1.dpad_left){
                pixelsPossesed = 1;
            }

            if (gamepad1.dpad_right){
                pixelsPossesed = 2;
            }

            if (gamepad1.dpad_up){
                pixelsPossesed = 0;
            }

            if (gamepad2.dpad_up){
                break;
            }





            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

        }


    }

    public void moveSlideToLevelOne(){
        moveSlideToPositionPID(leftSlideMotor, rightSlideMotor, slideLevelOne, slideKp, slideKi, slideKd, slideReference);
    }
    public void moveSlideToLevelTwo(){
        moveSlideToPositionPID(leftSlideMotor, rightSlideMotor, slideLevelTwo, slideKp, slideKi, slideKd, slideReference);
    }
    public void moveSlideToLevelThree(){
        moveSlideToPositionPID(leftSlideMotor, rightSlideMotor, slideLevelThree, slideKp, slideKi, slideKd, slideReference);
    }

    public void moveSlideToGround(){
        moveSlideToPositionPID(leftSlideMotor, rightSlideMotor, slideLevelGround, slideKp, slideKi, slideKd, slideReference);
    }
}