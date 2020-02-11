package org.firstinspires.ftc.teamcode;
/* here is a change
 */

/* This is mr. love's contribution
 */

/* Copyright (c) 2019 FIRST. All rights reserved.
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to
 imitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@Autonomous(name = "park", group = "Iterative Opmode")
public class OnlyParking extends LinearOpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    final static float rotatePerInchForwardBackward = 1;
    final static float rotatePerInchStrafing = 1;
    final static float rotationsPerDegreeRightLeft = 1;
    static float maxPower = (float)1;

    private DcMotor leftRear;
    private DcMotor leftFront;
    private DcMotor rightRear;
    private DcMotor rightFront;
    //private DcMotor spinRight;
    //private DcMotor spinLeft;
    //private DcMotor horizontalSlide;
    //private DcMotor spool;
    //private Servo grabServo;
    //private Servo rotateServo;
    //private Servo grabPlatform;


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftRear = hardwareMap.get(DcMotor.class, "left_rear");
        leftFront = hardwareMap.get(DcMotor.class, "left_front");
        rightRear = hardwareMap.get(DcMotor.class, "right_rear");
        rightFront = hardwareMap.get(DcMotor.class, "right_front");
        //rightFront.setTargetPosition(0);
        //leftFront.setTargetPosition(0);
        //rightRear.setTargetPosition(0);
        //leftRear.setTargetPosition(0);
        
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);


        // Tell the driver that initialization is complete.
        waitForStart();
        
        strafeRight(3000);
        strafeRight(3000);
        allOff();
        sleep(20);
        strafeRight(3000);
        strafeRight(3000);
        allOff();
        sleep(20);
        strafeRight(3000);
        strafeRight(3000);
        allOff();
        sleep(20);
        strafeLeft(3000);
        strafeLeft(3000);
        allOff();
        sleep(20);
        strafeLeft(3000);
        allOff();
        
        //forwardBackward(-10);

        //turn(true,5);
        telemetry.addData("Status", "Initialized");
    }
    void leftStrafe(double speedMax, double speedMin, double inches) {
        speedMax/=2;
        speedMin/=2;
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        leftRear.setPower(speedMax);
        leftFront.setPower(-speedMax);
        rightRear.setPower(-speedMax);
        rightFront.setPower(speedMax);
        double total=leftRear.getCurrentPosition()+inches;
        double start=leftRear.getCurrentPosition();
        while(leftRear.getCurrentPosition()<total){
            double inertiaCancel=1-Math.abs(start-leftRear.getCurrentPosition())/Math.abs(total);
            if(inertiaCancel<0.02){
                break;
            }
            telemetry.addData("percent complete",1-Math.abs(start-leftRear.getCurrentPosition())/Math.abs(total));
            if(Math.abs(speedMax*inertiaCancel)<speedMin){
                telemetry.addData("percent complete",1-Math.abs(start-leftRear.getCurrentPosition())/Math.abs(total));

                leftRear.setPower(speedMin);
                leftFront.setPower(-speedMin);
                rightRear.setPower(-speedMin);
                rightFront.setPower(speedMin);
            }else {
                leftRear.setPower(speedMax * inertiaCancel);
                leftFront.setPower(-speedMax * inertiaCancel);
                rightRear.setPower(-speedMax * inertiaCancel);
                rightFront.setPower(speedMax * inertiaCancel);
            }
            telemetry.update();

        }
        allOff();
    }



    void rightStrafe(double speedMax, double speedMin, double inches){
        //choose 1 motor to do all the counting (this should stay consistent across all methods then reset that motors
        //encoders
        speedMax/=2;
        speedMin/=2;
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //set all speeds to negative 1 for backward
        leftRear.setPower(-speedMax);
        leftFront.setPower(speedMax);
        rightRear.setPower(speedMax);
        rightFront.setPower(-speedMax);

        //declare variables for total target position and start position
        double total=leftRear.getCurrentPosition()-inches;
        double start=leftRear.getCurrentPosition();

        //this whileloop runs while the robot has not gone farther past the total
        while(leftRear.getCurrentPosition()>total){
            //the inertia cancel varaible calculates the percentage that your motion is complete
            double inertiaCancel=1-Math.abs(start-leftRear.getCurrentPosition())/Math.abs(total);

            //one it gets so low that the power would be negladble, stop the loop (it goes all the way to infinity if this
            //is not done
            if(inertiaCancel<0.02){
                break;
            }
            //check if we exceeded max speed
            if(Math.abs(speedMax*inertiaCancel)<speedMin){
                telemetry.addData("percent complete",1-Math.abs(start-leftRear.getCurrentPosition())/Math.abs(total));

                leftRear.setPower(-speedMin);
                leftFront.setPower(speedMin);
                rightRear.setPower(speedMin);
                rightFront.setPower(-speedMin);
            }else {
                leftRear.setPower(-speedMax * inertiaCancel);
                leftFront.setPower(speedMax * inertiaCancel);
                rightRear.setPower(speedMax * inertiaCancel);
                rightFront.setPower(-speedMax * inertiaCancel);
            }
            telemetry.update();

        }
        allOff();
    }


    void backward(double speedMax, double speedMin, double inches){
        //choose 1 motor to do all the counting (this should stay consistent across all methods then reset that motors
        //encoders
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //set all speeds to negative 1 for backward
        leftRear.setPower(-speedMax);
        leftFront.setPower(-speedMax);
        rightRear.setPower(-speedMax);
        rightFront.setPower(-speedMax);

        //declare variables for total target position and start position
        double total=leftRear.getCurrentPosition()-inches;
        double start=leftRear.getCurrentPosition();

        //this whileloop runs while the robot has not gone farther past the total
        while(leftRear.getCurrentPosition()>total){
            //the inertia cancel varaible calculates the percentage that your motion is complete
            double inertiaCancel=1-Math.abs(start-leftRear.getCurrentPosition())/Math.abs(total);

            //one it gets so low that the power would be negladble, stop the loop (it goes all the way to infinity if this
            //is not done
            if(inertiaCancel<0.02){
                break;
            }
            //check if we exceeded max speed
            if(Math.abs(speedMax*inertiaCancel)<speedMin){
                telemetry.addData("percent complete",1-Math.abs(start-leftRear.getCurrentPosition())/Math.abs(total));

                leftRear.setPower(-speedMin);
                leftFront.setPower(-speedMin);
                rightRear.setPower(-speedMin);
                rightFront.setPower(-speedMin);
            }else {
                leftRear.setPower(-speedMax * inertiaCancel);
                leftFront.setPower(-speedMax * inertiaCancel);
                rightRear.setPower(-speedMax * inertiaCancel);
                rightFront.setPower(-speedMax * inertiaCancel);
            }
            telemetry.update();

        }
        allOff();
    }


   void forward(double speedMax, double speedMin, double inches){
       leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

       leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


       leftRear.setPower(speedMax);
       leftFront.setPower(speedMax);
       rightRear.setPower(speedMax);
       rightFront.setPower(speedMax);
       double total=leftRear.getCurrentPosition()+inches;
       double start=leftRear.getCurrentPosition();
       while(leftRear.getCurrentPosition()<total){
           double inertiaCancel=1-Math.abs(start-leftRear.getCurrentPosition())/Math.abs(total);
           if(inertiaCancel<0.02){
               break;
           }
           telemetry.addData("percent complete",1-Math.abs(start-leftRear.getCurrentPosition())/Math.abs(total));
           if(Math.abs(speedMax*inertiaCancel)<speedMin){
               telemetry.addData("percent complete",1-Math.abs(start-leftRear.getCurrentPosition())/Math.abs(total));

               leftRear.setPower(speedMin);
               leftFront.setPower(speedMin);
               rightRear.setPower(speedMin);
               rightFront.setPower(speedMin);
           }else {
               leftRear.setPower(speedMax * inertiaCancel);
               leftFront.setPower(speedMax * inertiaCancel);
               rightRear.setPower(speedMax * inertiaCancel);
               rightFront.setPower(speedMax * inertiaCancel);
           }
            telemetry.update();
       }
       allOff();
   }
    void allOff(){
        rightFront.setPower(0);
        leftFront.setPower(0);
        rightRear.setPower(0);
        leftRear.setPower(0);
    }
    void turn(boolean right, float degrees){
        

        if(right){
            
         leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFront.setTargetPosition((int)(-1000000 * rotatePerInchForwardBackward) - 900000);

        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRear.setTargetPosition((int)(-1000000 * rotatePerInchForwardBackward));

        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setTargetPosition((int)(100000 * rotatePerInchForwardBackward));

        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRear.setTargetPosition((int)(100000 * rotatePerInchForwardBackward));

            allOnRight();
            while(rightRear.getCurrentPosition()<500){
                
            }
        }else{
             leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFront.setTargetPosition((int)(10000000 * rotatePerInchForwardBackward) + 900000);

        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRear.setTargetPosition((int)(100000000 * rotatePerInchForwardBackward));

        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setTargetPosition((int)(-10000000 * rotatePerInchForwardBackward));

        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRear.setTargetPosition((int)(-1000000 * rotatePerInchForwardBackward));

            allOnLeft();
            while(leftRear.getCurrentPosition()>500){
                
            }
        }
      
        allOff();
    }
   
    void allOnRight(){
        leftFront.setPower(-1);
        leftRear.setPower(-1);
        rightRear.setPower(1);
        rightFront.setPower(1);
    }
    void allOnLeft(){
        leftFront.setPower(1);
        leftRear.setPower(1);
        rightRear.setPower(-1);
        rightFront.setPower(-1);
    }
    void allOn(){
        leftFront.setPower(maxPower);
        leftRear.setPower(maxPower);
        rightRear.setPower(1);
        rightFront.setPower(maxPower);
    }
    void allOnForwardBack(){
        leftFront.setPower(maxPower);
        leftRear.setPower(maxPower);
        rightRear.setPower(.48);
        rightFront.setPower(.48);
    }
    
 
}
