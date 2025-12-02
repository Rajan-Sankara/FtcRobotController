package org.firstinspires.ftc.teamcode.DriverCode.RobotCentric;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
@TeleOp
public class RCLeagueMeet1 extends OpMode {
    public DcMotorEx leftRear, rightRear, leftFront, rightFront;
    public DcMotorEx intakeMotor, shooter1, shooter2, conveyor;
    ElapsedTime servoTimer = new ElapsedTime();
    boolean servoTriggered = false;
    public ServoImplEx servo;

    boolean lastSquare = false;
    boolean lastTriangle =false;

    double power = 0.5;
    @Override
    public void init() {
        //Initializes the Wheel Motors and Intake, Outtake, and ConveyorBelt
        leftRear = hardwareMap.get(DcMotorEx.class, "backLeft");
        rightRear = hardwareMap.get(DcMotorEx.class, "backRight");
        leftFront = hardwareMap.get(DcMotorEx.class, "frontLeft");
        rightFront = hardwareMap.get(DcMotorEx.class, "frontRight");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "CarWash");
        shooter1 = hardwareMap.get(DcMotorEx.class, "OuttakeLeft");
        shooter2 = hardwareMap.get(DcMotorEx.class, "OuttakeRight");
        conveyor = hardwareMap.get(DcMotorEx.class, "ConveyorBelt");
        servo = hardwareMap.get(ServoImplEx.class, "OuttakeServo");

        //Set Motor Rotation Directions
        //FORWARD = Counterclockwise and REVERSE = Clockwise
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

        telemetry.addData("status", "Ready to drive");
        telemetry.update();
    }

    @Override
    public void loop() {
        double y = -gamepad1.left_stick_y; // forward/back
        double rx = gamepad1.right_stick_x; // rotation

// Driver Math to set the speeds
        double denominator = Math.max(Math.abs(y) + Math.abs(rx), 1);
        double frontLeftPower = (y + rx) / denominator;
        double backLeftPower = (y + rx) / denominator;
        double frontRightPower = (y - rx) / denominator;
        double backRightPower = (y - rx) / denominator;

        leftFront.setPower(frontLeftPower);
        leftRear.setPower(backLeftPower);
        rightFront.setPower(frontRightPower);
        rightRear.setPower(backRightPower);

        //Power set for the motors
        intakeMotor.setPower(1);
        shooter1.setPower(power);
        shooter2.setPower(power);
        conveyor.setPower(1);

        //DO NOT UNCOMMENT THIS OR DELETE THIS COMMENTED if statement BELOW
        /*
        if (gamepad1.cross){
            servo.setPosition(0.05);//Kick position
            servo.setPosition(1.1);//Reset Position
        }
         */

        if (gamepad1.cross && !servoTriggered) {
            //Sets servo to kick position
            servo.setPosition(0.01);
            servoTimer.reset();
            servoTriggered = true;
        }
        if (servoTriggered && servoTimer.seconds() > 0.5) {
            //Set servo to reset position after 0.75 seconds
            servo.setPosition(0.9);
            servoTriggered = false;
        }

        // Detect rising edge: button is pressed now, but wasn't before
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
    }
}
