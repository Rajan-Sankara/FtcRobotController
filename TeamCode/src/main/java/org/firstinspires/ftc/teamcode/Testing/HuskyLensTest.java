package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.dfrobot.HuskyLens.Block;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class HuskyLensTest extends LinearOpMode {

    // === Distance Calculation Constants ===
    // REAL measured width of the black part of your tag (in inches)
    private static final double REAL_TAG_WIDTH = 7.0;
    // Will be replaced after calibration
    private static final double FOCAL_LENGTH = (double) (70 * 28) / 7;
    public DcMotorEx leftRear, rightRear, leftFront, rightFront;
    public DcMotorEx intakeMotor, shooter1, shooter2, conveyor;
    public ServoImplEx servo;
    HuskyLens husky;
    ElapsedTime servoTimer = new ElapsedTime();
    boolean servoTriggered = false;
    boolean lastSquare = false;
    boolean lastTriangle = false;
    double power = 0.5;

    @Override
    public void runOpMode() {

        husky = hardwareMap.get(HuskyLens.class, "HuskyLens");
        leftRear = hardwareMap.get(DcMotorEx.class, "backLeft");
        rightRear = hardwareMap.get(DcMotorEx.class, "backRight");
        leftFront = hardwareMap.get(DcMotorEx.class, "frontLeft");
        rightFront = hardwareMap.get(DcMotorEx.class, "frontRight");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "CarWash");
        shooter1 = hardwareMap.get(DcMotorEx.class, "OuttakeLeft");
        shooter2 = hardwareMap.get(DcMotorEx.class, "OuttakeRight");
        conveyor = hardwareMap.get(DcMotorEx.class, "ConveyorBelt");
        servo = hardwareMap.get(ServoImplEx.class, "OuttakeServo");

        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter1.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.REVERSE);

        //Encoder Init
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Brake System
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (!husky.knock()) {
            telemetry.addData(">>", "Problem communicating with " + husky.getDeviceName());
        }
        else {
            telemetry.addData(">>", "HuskyLens Connected!");
        }

        // Use trained HuskyLens tags (NOT FTC AprilTags)
        husky.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);

        telemetry.addLine("Waiting for start...");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            double ydrive = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double xdrive = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            //Driver Math to set the speeds
            double denominator = Math.max(Math.abs(ydrive) + Math.abs(xdrive) + Math.abs(rx), 1);
            double frontLeftPower = (ydrive + xdrive + rx) / denominator;
            double backLeftPower = (ydrive - xdrive + rx) / denominator;
            double frontRightPower = (ydrive - xdrive - rx) / denominator;
            double backRightPower = (ydrive + xdrive - rx) / denominator;

            leftFront.setPower(frontLeftPower);
            leftRear.setPower(backLeftPower);
            rightFront.setPower(frontRightPower);
            rightRear.setPower(backRightPower);

            //Power set for the motors
            intakeMotor.setPower(1);
            shooter1.setPower(power);
            shooter2.setPower(power);
            conveyor.setPower(0.75);

            //DO NOT UNCOMMENT THIS OR DELETE THIS COMMENTED if statement BELOW
        /*
        if (gamepad1.cross){
            servo.setPosition(0.05);//Kick position
            servo.setPosition(1.1);//Reset Position
        }
        */

            if (gamepad1.cross && !servoTriggered) {
                //Sets servo to kick position
                servo.setDirection(Servo.Direction.REVERSE);
                servo.setPosition(0.9);
                servoTimer.reset();
                servoTriggered = true;
            }
            if (servoTriggered && servoTimer.seconds() > 0.5) {
                //Set servo to reset position after 0.75 seconds
                servo.setPosition(0.2);
                servoTriggered = false;
            }

            telemetry.addData("Position", servo.getPosition());

            // Detect rising edge: button is pressed now, but wasn't before
            /*
            boolean squareJustPressed = gamepad1.square && !lastSquare;
            boolean triangleJustPressed = gamepad1.triangle && !lastTriangle;

            if (squareJustPressed) {
                power -= 0.1;
            }
            if (triangleJustPressed) {
                power += 0.1;
            }

            // Clamp power between 0 and 1
            power = Math.max(0.0, Math.min(1.0, power));

            // Apply power
            shooter1.setPower(power);
            shooter2.setPower(power);

            // Update last button states
            lastSquare = gamepad1.square;
            lastTriangle = gamepad1.triangle;

             */

            HuskyLens.Block[] blocks = husky.blocks();

            if (blocks != null && blocks.length > 0) {

                for (int i = 0; i < blocks.length; i++) {

                    Block tag = blocks[i];

                    int tagID = tag.id;
                    int x = tag.x;
                    int y = tag.y;
                    int width = tag.width;
                    int height = tag.height;

                    telemetry.addLine("=== Block #" + i + " ===");
                    telemetry.addData("Tag ID", tagID);
                    telemetry.addData("Width (px)", width);
                    telemetry.addData("Height (px)", height);
                    telemetry.addData("Shooter1 Power", shooter1);
                    telemetry.addData("Shooter2 Power", shooter2);

                    // === DISTANCE CALCULATION ===
                    // distance = (real_width * focal_length) / pixel_width
                    double distanceInches = (REAL_TAG_WIDTH * FOCAL_LENGTH) / width;

                    telemetry.addData("Distance (in)", "%.1f", distanceInches);

                    if (tagID == 1) {
                        telemetry.addLine("★★★ Tag 20 DETECTED! ★★★");

                        if (distanceInches >= 0 && distanceInches <= 45){

                            shooter1.setPower(0.3);
                            shooter2.setPower(0.3);
                        }
                        if (distanceInches > 45 && distanceInches <= 60){

                            shooter1.setPower(0.5);
                            shooter2.setPower(0.5);
                        }
                        if (distanceInches > 60 && distanceInches <= 75){

                            shooter1.setPower(0.65);
                            shooter2.setPower(0.65);
                        }
                        if(distanceInches>112){
                            shooter1.setPower(0.5);
                            shooter2.setPower(0.5);
                        }
                        telemetry.addData("Power", shooter1.getPower());
                    }
                    else
                    {
                        telemetry.addLine("Tag detected, but NOT ID 20");
                    }

                }
            }
            else {
                telemetry.addLine("NO TAG DETECTED");
            }
            telemetry.update();
        }
    }
}
