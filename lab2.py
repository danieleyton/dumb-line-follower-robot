import sim
import sys
import numpy as np

def automaticDrive(clientID, leftSensor, middleSensor, rightSensor, leftJointDynamic, rightJointDynamic, wheelRadius, nominalLinearVelocity):
    while(True):
        # read the sensors:
        sensorReading=[False, False, False]
        sensorReading[0]=(sim.simxReadVisionSensor(clientID, leftSensor, sim.simx_opmode_blocking)==1)
        sensorReading[1]=(sim.simxReadVisionSensor(clientID, middleSensor, sim.simx_opmode_blocking)==1)
        sensorReading[2]=(sim.simxReadVisionSensor(clientID, rightSensor, sim.simx_opmode_blocking)==1)
        err, state, dataLeft = sim.simxReadVisionSensor(clientID, leftSensor, sim.simx_opmode_blocking)
        err, state, dataRight = sim.simxReadVisionSensor(clientID, rightSensor, sim.simx_opmode_blocking)
        err, state, dataMiddle = sim.simxReadVisionSensor(clientID, middleSensor, sim.simx_opmode_blocking)

        # decide about left and right velocities:
        linearVelocityLeft=nominalLinearVelocity
        linearVelocityRight=nominalLinearVelocity
        print(dataLeft[0][10])
        if(dataLeft[0][10]>0.7 and dataRight[0][10]<=0.2):
            linearVelocityRight=linearVelocityRight*0.4
            #linearVelocityLeft=nominalLinearVelocity*0.7
        if(dataRight[0][10]>0.7 and dataLeft[0][10]<=0.2):
            linearVelocityLeft=linearVelocityLeft*0.4
            #linearVelocityRight=nominalLinearVelocity*0.7
        if(dataMiddle[0][10] <= 0.2):
            linearVelocityRight=nominalLinearVelocity
            linearVelocityLeft=nominalLinearVelocity

        # now make it move
        sim.simxSetJointTargetVelocity(clientID, leftJointDynamic, linearVelocityLeft/wheelRadius, sim.simx_opmode_blocking)
        sim.simxSetJointTargetVelocity(clientID, rightJointDynamic, linearVelocityRight/wheelRadius, sim.simx_opmode_blocking)

#def manualControl():
    # nu se unu

def main():
    sim.simxFinish(-1) # Terminar todas las conexiones
    clientID=sim.simxStart('127.0.0.1',19999,True,True,5000,5) # Iniciar una nueva conexion

    if clientID != -1:
        print("Conexion establecida")
        res, lineTracerBase = sim.simxGetObjectHandle(clientID, 'LineTracerBase', sim.simx_opmode_blocking)
        res, leftSensor = sim.simxGetObjectHandle(clientID, 'LeftSensor', sim.simx_opmode_blocking)
        res, middleSensor = sim.simxGetObjectHandle(clientID, 'MiddleSensor', sim.simx_opmode_blocking)
        res, rightSensor = sim.simxGetObjectHandle(clientID, 'RightSensor', sim.simx_opmode_blocking)
        res, leftJoin = sim.simxGetObjectHandle(clientID, 'LeftJoint', sim.simx_opmode_blocking)
        res, rightJoint= sim.simxGetObjectHandle(clientID, 'RightJoint', sim.simx_opmode_blocking)
        res, leftJointDynamic= sim.simxGetObjectHandle(clientID, 'DynamicLeftJoint', sim.simx_opmode_blocking)
        res, rightJointDynamic= sim.simxGetObjectHandle(clientID, 'DynamicRightJoint', sim.simx_opmode_blocking)
        nominalLinearVelocity = 0.08
        wheelRadius = 0.027
        interWheelDistance = 0.119

        # Decide about left and right velocities:
#        linearVelocityLeft = nominalLinearVelocity
#        linearVelocityRight = nominalLinearVelocity
#        # Now make it move ! (hopefully :ccc)
#        sim.simxSetJointTargetVelocity(clientID, leftJointDynamic, linearVelocityLeft/wheelRadius, sim.simx_opmode_blocking)
#        sim.simxSetJointTargetVelocity(clientID, rightJointDynamic, linearVelocityRight/wheelRadius, sim.simx_opmode_blocking)

#        sim.simxSetJointTargetVelocity(clientID, leftJointDynamic, 0/wheelRadius, sim.simx_opmode_blocking)
#        sim.simxSetJointTargetVelocity(clientID, rightJointDynamic, 0/wheelRadius, sim.simx_opmode_blocking)
        sim.simxSetJointTargetVelocity(clientID, leftJointDynamic, 0/wheelRadius, sim.simx_opmode_blocking)
        sim.simxSetJointTargetVelocity(clientID, rightJointDynamic, 0/wheelRadius, sim.simx_opmode_blocking)

        automaticDrive(clientID, leftSensor, middleSensor, rightSensor, leftJointDynamic, rightJointDynamic, wheelRadius, nominalLinearVelocity)
    else:
        print("Error: no se puede conectar")
        sys.exit("Error: no se puede conectar")


main()
