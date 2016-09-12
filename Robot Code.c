#pragma config(Motor,  port1,           rightWheelFront, tmotorVex393_HBridge, openLoop, reversed)
#pragma config(Motor,  port2,           rightWheelBack, tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port3,           leftWheelFront, tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port4,           leftWheelBack, tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port5,           middleWheel,   tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port6,           fourBar,       tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port7,           catapult,      tmotorVex393_MC29, openLoop)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

task main()
{
  while(true) {

    //Drive Train
    motor[rightWheelFront] = vexRT[Ch2];
    motor[rightWheelBack] = vexRT[Ch2];
    motor[leftWheelFront] = vexRT[Ch3];
    motor[leftWheelBack] = vexRT[Ch3];
    motor[middleWheel] = vexRT[Ch4];



    //Whichever launching mechanism we decide on
    motor[fourBar] = vexRT[Btn6U];
    motor[catapult] = vexRT[Btn6D];


  }

}
