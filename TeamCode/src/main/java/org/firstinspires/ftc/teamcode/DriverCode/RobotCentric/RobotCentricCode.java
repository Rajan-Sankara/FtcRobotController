package org.firstinspires.ftc.teamcode.DriverCode.RobotCentric;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.lang.reflect.Field;
import java.util.Arrays;
import java.util.List;

@TeleOp
public class RobotCentricCode extends OpMode {

    public DcMotorEx leftRear, rightRear, leftFront, rightFront;
    public DcMotorEx intakeMotor, shooter1, shooter2, conveyor;
    public ServoImplEx servo;
    public HuskyLens huskyLens;

    ElapsedTime servoTimer = new ElapsedTime();
    boolean servoTriggered = false;

    double baseShooterPower = 0.5;

    // Constants for distance calculation
    private static final double KNOWN_TAG_WIDTH = 50.0; // mm
    private static final double FOCAL_LENGTH = 700.0;

    @Override
    public void init() {
        // Drive motors
        leftRear = hardwareMap.get(DcMotorEx.class, "backLeft");
        rightRear = hardwareMap.get(DcMotorEx.class, "backRight");
        leftFront = hardwareMap.get(DcMotorEx.class, "frontLeft");
        rightFront = hardwareMap.get(DcMotorEx.class, "frontRight");

        // Subsystems
        intakeMotor = hardwareMap.get(DcMotorEx.class, "CarWash");
        shooter1 = hardwareMap.get(DcMotorEx.class, "OuttakeLeft");
        shooter2 = hardwareMap.get(DcMotorEx.class, "OuttakeRight");
        conveyor = hardwareMap.get(DcMotorEx.class, "ConveyorBelt");
        servo = hardwareMap.get(ServoImplEx.class, "OuttakeServo");

        // HuskyLens
        huskyLens = hardwareMap.get(HuskyLens.class, "huskyLens");
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);

        // Motor directions
        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter1.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.REVERSE);

        // Encoders
        for (DcMotorEx motor : new DcMotorEx[]{leftRear, leftFront, rightRear, rightFront}) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        telemetry.addLine("Ready to drive");
        telemetry.update();
    }

    @Override
    public void loop() {
        // --- Robot-Centric Drive ---
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        leftFront.setPower(frontLeftPower);
        leftRear.setPower(backLeftPower);
        rightFront.setPower(frontRightPower);
        rightRear.setPower(backRightPower);

        // --- HuskyLens Tag Detection ---
        double shooterPower = baseShooterPower;
        List<?> blocks = Arrays.asList(huskyLens.blocks());

        if (blocks != null && !blocks.isEmpty()) {
            Object block = blocks.get(0);
            try {
                Field widthField = block.getClass().getField("width");
                Field idField = block.getClass().getField("id");

                double width = Math.max((double) widthField.get(block), 1.0);
                int id = (int) idField.get(block);
                double distance = (KNOWN_TAG_WIDTH * FOCAL_LENGTH) / width;

                // Adjust shooter power based on distance
                if (distance >= 800) {
                    shooterPower = 0.8;
                } else if (distance >= 400 && distance < 800) {
                    shooterPower = 0.6;
                } else if (distance >= 200 && distance < 400) {
                    shooterPower = 0.4;
                } else {
                    shooterPower = 0.0;
                }

                telemetry.addData("Tag ID", id);
                telemetry.addData("Tag Width", width);
                telemetry.addData("Distance (mm)", distance);
                telemetry.addData("Shooter Power", shooterPower);
            } catch (Exception e) {
                telemetry.addLine("Error reading tag data");
            }
        } else {
            telemetry.addLine("No Tag Detected");
        }

        // --- Subsystems ---
        intakeMotor.setPower(1);
        conveyor.setPower(1);
        shooter1.setPower(shooterPower);
        shooter2.setPower(shooterPower);

        // Servo kick
        if (gamepad1.cross && !servoTriggered) {
            servo.setPosition(0.05);
            servoTimer.reset();
            servoTriggered = true;
        }
        if (servoTriggered && servoTimer.seconds() > 0.75) {
            servo.setPosition(1.1);
            servoTriggered = false;
        }

        // Manual shooter tuning
        if (gamepad1.left_bumper) {
            shooter1.setPower(baseShooterPower - 0.1);
            shooter2.setPower(baseShooterPower - 0.1);
        } else if (gamepad1.right_bumper) {
            shooter1.setPower(baseShooterPower + 0.1);
            shooter2.setPower(baseShooterPower + 0.1);
        }

        telemetry.update();
    }
}