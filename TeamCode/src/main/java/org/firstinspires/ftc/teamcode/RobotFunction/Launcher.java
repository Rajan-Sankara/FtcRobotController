package org.firstinspires.ftc.teamcode.RobotFunction;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.dfrobot.HuskyLens.Block;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class Launcher extends LinearOpMode {

    // === Constants for HuskyLens distance calculation ===
    private static final double REAL_TAG_WIDTH = 7.0; // inches
    private static final double FOCAL_LENGTH = (double) (70 * 28) / 7; // adjust as needed (example value)
    // === Constants for shooter physics ===
    private final double shooterHeight = 16.5; // in
    private final double launchAngleDeg = 65;
    private final double flywheelRadius = 1.417; // in
    private final double g = 386.09; // in/s^2
    private final double maxRPM = 6000;
    private final double powerRampSpeed = 0.02; // max change per loop
    private final ElapsedTime servoTimer = new ElapsedTime();
    boolean lastSquare = false;
    boolean lastTriangle = false;
    // === Hardware ===
    private DcMotorEx shooter1, shooter2;
    private ServoImplEx servo;
    private HuskyLens husky;
    private boolean servoTriggered = false;

    // Persisting variables so ramping and readiness checks persist across iterations
    private double currentShooterPower = 0.0;
    private boolean shooterAtSpeed = false;

    @Override
    public void runOpMode() {

        initializeLauncherSetup();
        initializeHuskyLens();
        telemetry.update();     // pushes the texts to driver hub
        waitForStart();         // waits for button to be pressed in driver hub

        while (opModeIsActive()) {

            double desiredRPM = 0.0;
            shooterAtSpeed = false;

            Block[] blocks = getHuskyLensBlock();
            if (blocks == null) {
                telemetry.addLine("No blocks returned (blocks == null)");
            } else {
                telemetry.addData("Blocks found", blocks.length);
                for (int i = 0; i < blocks.length; i++) {
                    Block tag = blocks[i];

                    // Distance calculation: distance = (real width * focal length) / pixel width
                    double distanceInches = (REAL_TAG_WIDTH * FOCAL_LENGTH) / Math.max(1, blocks[i].width); //Makes sure no negative #

                    displayHuskyLensValues(i, blocks, distanceInches);

                    // Only use tags of interest (adjust IDs to whatever your tags actually are)
                    if (blocks[i].id == 1 || blocks[i].id == 20) {
                        telemetry.addLine("HuskyLens Detected target tag ID: " + blocks[i].id);

                        // Physics calculation for ball velocity
                        double radiansDenominator = convertAngleToRadians(distanceInches);

                        if (radiansDenominator > 0) {
                            desiredRPM = getDesiredRPM(distanceInches, radiansDenominator);

                            // convert to [0,1] power
                            if (!Double.isNaN(desiredRPM) && !Double.isInfinite(desiredRPM)) {
                                increaseShooterSpeedSmoothly(desiredRPM);
                                calcAndShowShooterSpeed(desiredRPM);
                            }
                        }
                    }
                }
            }

            // --- Servo / trigger logic ---
            triggerServoLogic();

            telemetry.update();
        }
    }

    private double getDesiredRPM(double distanceInches, double radiansDenominator) {
        double desiredRPM;
        double ballVelocity = Math.sqrt((g * distanceInches * distanceInches) / radiansDenominator);
        desiredRPM = (ballVelocity / (2 * Math.PI * flywheelRadius)) * 60.0;
        if (Double.isNaN(desiredRPM) || Double.isInfinite(desiredRPM)) {
            telemetry.addData("DesiredRPM", "Invalid result");
        }
        return desiredRPM;
    }

    private double convertAngleToRadians(double distanceInches) {
        double angleRad = Math.toRadians(launchAngleDeg);
        double cosA = Math.cos(angleRad);
        double tanA = Math.tan(angleRad);

        // protect denominator/inside sqrt from invalid values
        double denominator = 2 * cosA * cosA * (distanceInches * tanA - shooterHeight);
        if (denominator <= 0) {
            telemetry.addData("Physics", "Invalid denominator (<=0) for distance=" + distanceInches);
        }
        return denominator;
    }

    private void increaseShooterSpeedSmoothly(double desiredRPM) {
        double desiredPower = Math.min(desiredRPM / maxRPM, 1.0);
        desiredPower = Math.max(desiredPower, 0.0);

        // Smooth ramping
        calcPowerForSmoothRamping(desiredPower);

        shooter1.setPower(currentShooterPower);
        shooter2.setPower(currentShooterPower);
    }

    private void calcAndShowShooterSpeed(double desiredRPM) {
        double currentRPM = currentShooterPower * maxRPM;
        double desiredRPMThreshold = 50.0; // ±50 RPM tolerance
        shooterAtSpeed = Math.abs(currentRPM - desiredRPM) <= desiredRPMThreshold;

        telemetry.addData("Shooter CurrentPower", "%.3f", currentShooterPower);
        telemetry.addData("Shooter CurrentRPM", "%.0f", currentRPM);
        telemetry.addData("Shooter DesiredRPM", "%.0f", desiredRPM);
        telemetry.addData("Shooter Ready", shooterAtSpeed ? "YES" : "NO");
    }

    private void triggerServoLogic() {
        if (gamepad1.cross && !servoTriggered && shooterAtSpeed) {
            servo.setPosition(0.01);
            servoTimer.reset();
            servoTriggered = true;
        }

        if (servoTriggered && servoTimer.seconds() > 1.0) {
            servo.setPosition(0.9);
            servoTriggered = false;
        }

        // handle one-shot button presses if you need them
        boolean squareJustPressed = gamepad1.square && !lastSquare;
        boolean triangleJustPressed = gamepad1.triangle && !lastTriangle;
        lastSquare = gamepad1.square;
        lastTriangle = gamepad1.triangle;
    }

    private void calcPowerForSmoothRamping(double desiredPower) {
        if (desiredPower > currentShooterPower + powerRampSpeed) {
            currentShooterPower += powerRampSpeed;
        } else if (desiredPower < currentShooterPower - powerRampSpeed) {
            currentShooterPower -= powerRampSpeed;
        } else {
            currentShooterPower = desiredPower;
        }
    }

    private void displayHuskyLensValues(int i, Block[] blocks, double distanceInches) {
        telemetry.addLine("=== Block #" + i + " ===");
        telemetry.addData("Tag ID", blocks[i].id);
        telemetry.addData("Width (px)", blocks[i].width);
        telemetry.addData("Height (px)", blocks[i].height);
        telemetry.addData("Distance (in)", "%.1f", distanceInches);
    }

    private Block[] getHuskyLensBlock() {
        // --- HuskyLens block detection ---
        Block[] blocks = null;
        try {
            blocks = husky.blocks();
        } catch (Exception e) {
            telemetry.addData("HuskyLens.blocks() error", e.toString());
        }
        return blocks;
    }

    private void initializeHuskyLens() {

        boolean huskyOk = false;
        try {
            huskyOk = husky.knock();
        } catch (Exception e) {
            telemetry.addData("HuskyLens knock() error", e.toString());
        }

        if (!huskyOk) {
            telemetry.addLine("Problem communicating with HuskyLens");
        } else {
            telemetry.addLine("HuskyLens Connected!");
            try {
                husky.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
            } catch (Exception e) {
                telemetry.addData("HuskyLens algorithm error", e.toString());
            }
        }

        telemetry.addLine("Waiting for start...");
    }

    private void initializeLauncherSetup() {
        shooter1 = hardwareMap.get(DcMotorEx.class, "OuttakeLeft");
        shooter2 = hardwareMap.get(DcMotorEx.class, "OuttakeRight");
        servo = hardwareMap.get(ServoImplEx.class, "OuttakeServo");
        husky = hardwareMap.get(HuskyLens.class, "HuskyLens");
        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter1.setDirection(DcMotorSimple.Direction.FORWARD);
    }
}
