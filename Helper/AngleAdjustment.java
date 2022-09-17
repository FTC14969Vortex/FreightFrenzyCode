

/**
 You need to create a folder called Helper where you put all of your files with motor initializations, computer vision, etc.

 This file's path is: FtcRobotController/TeamCode/java/org.firstinspires.ftc.teamcode/Opmodes/AngleAdjustment.java
 **/



package org.firstinspires.ftc.teamcode.Helper;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;


import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import com.qualcomm.robotcore.util.ElapsedTime;



import java.util.ArrayList;



@Autonomous(name="Angle Adjustment", group="Linear Opmode")

public class AngleAdjustment extends LinearOpMode {
    public int autonomousStep;
    public float maxSpeed = 0.5f;
    public float medSpeed = 0.25f;
    float offset;
    float angleOne;
    float angleTwo;


    Robot robot = new Robot();
    private ElapsedTime runtime = new ElapsedTime();
    private double currentTime = runtime.seconds();
    double front_distance;
    double red_side_wall_distance;


    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //int armCurrentPosition = robot.arm.getCurrentPosition();


        while (!opModeIsActive() && !isStopRequested()) {

            //robot.arm.setTargetPosition(correct);
            //robot.arm.setPower(0);
            //robot.arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            //robot.claw.setPosition(-1);
            angleOne = robot.getRobotAngle();
            telemetry.addData("Robot Angle", "%.1f", angleOne);
//            telemetry.addData("currentTime", "%s", currentTime);
//            telemetry.addData("Front range", String.format("%.01f in", robot.frontRange.getDistance(DistanceUnit.CM)));
//            telemetry.addData("Red Wall range", String.format("%.01f in", robot.redWallRange.getDistance(DistanceUnit.CM)));
//
//            telemetry.addData("Sensor Device Name", String.format("%s", robot.redWallRange.getDeviceName()));
            telemetry.update();

        }



        waitForStart();
        runtime.reset();
        currentTime = runtime.seconds();

        float angleOne = robot.getRobotAngle();
        angleOne = (float) (angleOne / 180 * Math.PI);
        angleTwo = robot.getRobotAngle();

        autonomousStep=400;

        while (opModeIsActive()) {


            switch (autonomousStep) {
                case 400:
                    // current angle
                    //if (currentTime > 3 && autonomousStep == 400) {
                    angleTwo = robot.getRobotAngle();
                    //angleTwo = (float) (angleTwo / 180 * Math.PI);
                    if (360 - angleTwo < 90) {
                        robot.FLMotor.setPower(0.2);
                        robot.BLMotor.setPower(0.2);
                        robot.FRMotor.setPower(-0.2);
                        robot.BRMotor.setPower(-0.2);
                        //angleTwo = robot.getRobotAngle();
                        //angleTwo = (float) (angleTwo / 180 * Math.PI);
                        autonomousStep = 420;

                    } else if (angleTwo - 0 < 90) {
                        robot.FLMotor.setPower(-0.2);
                        robot.BLMotor.setPower(-0.2);
                        robot.FRMotor.setPower(0.2);
                        robot.BRMotor.setPower(0.2);
                        //angleTwo = robot.getRobotAngle();
                        //angleTwo = (float) (angleTwo / 180 * Math.PI);
                        autonomousStep = 421;
                    }

                    break;


                case 420:
                    angleTwo = robot.getRobotAngle();
                    //angleTwo = (float) (angleTwo/180*Math.PI);
                    if (360 - angleTwo  <= 1) {
                        robot.FLMotor.setPower(0);
                        robot.BLMotor.setPower(0);
                        robot.FRMotor.setPower(0);
                        robot.BRMotor.setPower(0);
                        robot.FLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        robot.BLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        robot.FRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        robot.BRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        robot.FLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        robot.BLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        robot.FRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        robot.BRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        autonomousStep=0;

                    } else {
                        autonomousStep=400;
                    }
                    break;


                case 421:
                    angleTwo = robot.getRobotAngle();
                    //angleTwo = (float) (angleTwo/180*Math.PI);
                    if (angleTwo - 0 <= 1) {
                        robot.FLMotor.setPower(0);
                        robot.BLMotor.setPower(0);
                        robot.FRMotor.setPower(0);
                        robot.BRMotor.setPower(0);
                        robot.FLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        robot.BLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        robot.FRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        robot.BRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        robot.FLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        robot.BLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        robot.FRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        robot.BRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        autonomousStep=0;

                    } else {
                        autonomousStep=400;
                    }
                    break;


                case 999:
                    if (autonomousStep == 999) {
                        telemetry.addData("case 999", "%s", currentTime);
                        angleTwo = robot.getRobotAngle();
                        if (angleOne > angleTwo)
                            offset = angleOne - angleTwo;
                        else {
                            offset = angleTwo - angleOne;
                        }
                        if (currentTime > 14) {
                            autonomousStep = 1000;
                        }
                    }

                case 4001:

                    break;

            }
//
//            front_distance = robot.frontRange.getDistance(DistanceUnit.INCH);
//
//            red_side_wall_distance = robot.redWallRange.getDistance(DistanceUnit.INCH);

            //robot.moveUpdate();
            telemetry.addData("OpModeActive", "Running");
            telemetry.addData("Robot Angle", "%.1f", robot.getRobotAngle());
            telemetry.addData("Autonomous Step", "%d ", autonomousStep);
            telemetry.addData("Angle2", "%.1f", angleTwo);
            telemetry.addData("Front range", String.format("%.01f cm", front_distance));
            telemetry.addData("Red Wall range", String.format("%.01f cm", red_side_wall_distance));



            telemetry.update();

        }

    }
}

//a = 5ft, b = 7ft, c = 9 ft