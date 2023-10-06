package org.firstinspires.ftc.teamcode.competition.qualifiers.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class QualifiersTeleop extends LinearOpMode {
    int pixelsPossesed = 0;
    int slideLevelOne = 0;
    int slideLevelTwo = 0;
    int slideLevelThree = 0;

    int slideLevelGround = 0;

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
                if (gamepad1.left_trigger > 0){
                    intakeMotor.setPower(-gamepad1.left_trigger);
                }
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

    public void moveToPositionPID(DcMotor motor, int currentTicks, int positionTicks){
        // i will write this function later
    }

    public void moveSlideToLevelOne(){
        moveToPositionPID(leftSlideMotor, leftSlideMotorTicks, slideLevelOne);
        moveToPositionPID(rightSlideMotor, rightSlideMotorTicks, slideLevelOne);
    }
    public void moveSlideToLevelTwo(){
        moveToPositionPID(leftSlideMotor, leftSlideMotorTicks, slideLevelTwo);
        moveToPositionPID(rightSlideMotor, rightSlideMotorTicks, slideLevelTwo);
    }
    public void moveSlideToLevelThree(){
        moveToPositionPID(leftSlideMotor, leftSlideMotorTicks, slideLevelTwo);
        moveToPositionPID(rightSlideMotor, rightSlideMotorTicks, slideLevelTwo);
    }

    public void moveSlideToGround(){
        moveToPositionPID(leftSlideMotor, leftSlideMotorTicks, slideLevelGround);
        moveToPositionPID(rightSlideMotor, rightSlideMotorTicks, slideLevelGround);
    }
}