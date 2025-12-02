package org.firstinspires.ftc.teamcode.Autonomous;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
@Autonomous
public class AutoBlueBottom extends LinearOpMode{
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
        } else {
            telemetry.addData(">>", "HuskyLens Connected!");
        }

        // Use trained HuskyLens tags (NOT FTC AprilTags)
        husky.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);

        telemetry.addLine("Waiting for start...");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // Drive forward for 2.5 sec, then rotate clockwise

            leftFront.setPower(-0.5);
            leftRear.setPower(-0.5);
            rightFront.setPower(-0.5);
            rightRear.setPower(-0.5);
            sleep(400);

              // retract


            leftFront.setPower(0);
            leftRear.setPower(0);
            rightFront.setPower(0);
            rightRear.setPower(0);


// --- 5) Drive forward for 7 seconds ---

            break;

        }
    }
}
