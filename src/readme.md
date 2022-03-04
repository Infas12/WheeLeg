# How to use socketcan Dji motor driver

## Overview

本代码框架中提供了对SocketCan的基本封装以及Dji M3508和M2006电机控制的基本封装。设计采用了面向对象的基本思想，基本可以做到电机即插即用。

电机类均提供了两种控制方式：速度控制和位置控制。使用时仅需配置PID参数。（两个电机空载的速度环都调过，但是位置环都没有调过，需要手动试试。）


## Initialization

1. Specify portname and initialize `CanManager`.

Use `ip link show` to check portname.

```c++
CanManager::Instance()->SetPortName("can0");
CanManager::Instance()->Init();
```

2. Construct Motor object and set CanID.

```c++
M3508 testMotor; //Construct object
testMotor.Registration(0x201);  //specify Can ID
```

3. Configure Control Mode.

Here we provided 3 control modes: `SPD_MODE`, `POS_MODE` and `RELAX_MODE`. When in `RELAX_MODE` the motor will have no output torque.

Example: to use speed control mode:

```C++
testMotor.controlMode = Motor::SPD_MODE;
```

4. Configure PID Parameter.

In the motor class we provide a nested double-loop PID for position and speed control. (Check `M3508::Update()`). 

Configuring `pidPosition` is not required when using `SPD_MODE`, but we strongly recommand you to do so. 

**The following set of parameters IS NOT TESTED.**

```C++
//Position Control
testMotor.pidPosition.kp = 2;
testMotor.pidPosition.ki = 0.1;
testMotor.pidPosition.kd = 0.01;
testMotor.pidPosition.maxOut = 10;
testMotor.pidPosition.maxIOut = 1;

//Speed Control
testMotor.pidSpeed.kp = 3;
testMotor.pidSpeed.ki = 0.2;
testMotor.pidSpeed.kd = 0.05;
testMotor.pidSpeed.maxOut = 10;
testMotor.pidSpeed.maxIOut = 10;
```

## Main Loop

1. Set the desired speed / position for the motor.

Velocity control example
```c++
testMotor.speedSet = velDesired;
```

Position control example
```c++
testMotor.positionSet = posDesired;
```

2. Update motor

```c++
testMotor.update = 0;
```

To get and set the position/speed of a motor, you may visit the attributes of `Motor`.

example: get the position feedback of a motor
```c++
float posFdb = testMotor.positionFdb;
```

See `Motor.hpp` for details.

3. Send Command

```C++
DjiMotorManager::Instance()->Update();
```


## Brief example
```c++
int main()
{
    CanManager::Instance()->SetPortName("can0");
    CanManager::Instance()->Init();

    M3508 testMotor; 
    testMotor.Registration(0x207); //set ID

    testMotor.pidSpeed.kp = 3;
	testMotor.pidSpeed.ki = 0.2;
	testMotor.pidSpeed.kd = 0.05;
	testMotor.pidSpeed.maxOut = 10;
	testMotor.pidSpeed.maxIOut = 10;
    testMotor.controlMode = Motor::SPD_MODE; //set control mode

    while(true)
    {
        testMotor.speedSet = 2.0f;
        testMotor.Update();
        DjiMotorManager::Instance()->Update();
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}



```