import sim
import sys
import numpy as np

def main():
    sim.simxFinish(-1) # Terminar todas las conexiones
    clientID=sim.simxStart('127.0.0.1',19999,True,True,5000,5) # Iniciar una nueva conexion

    if clientID != -1:
        print("Conexion establecida")
        res, lineTracerBase = sim.simxGetObjectHandle(clientID, 'LineTracerBase', sim.simx_opmode_blocking)
        res, leftJoin = sim.simxGetObjectHandle(clientID, 'LeftJoint', sim.simx_opmode_blocking)
        res, rightJoint= sim.simxGetObjectHandle(clientID, 'RightJoint', sim.simx_opmode_blocking)
        res, leftJointDynamic= sim.simxGetObjectHandle(clientID, 'DynamicLeftJoint', sim.simx_opmode_blocking)

        res, rightJointDynamic= sim.simxGetObjectHandle(clientID, 'DynamicRightJoint', sim.simx_opmode_blocking)
        nominalLinearVelocity = 0.3
        wheelRadius = 0.027
        interWheelDistance = 0.119

        # Decide about left and right velocities:
        linearVelocityLeft = nominalLinearVelocity
        linearVelocityRight = nominalLinearVelocity
        # Now make it move ! (hopefully :ccc)
        sim.simxSetJointTargetVelocity(clientID, leftJointDynamic, linearVelocityLeft/wheelRadius, sim.simx_opmode_blocking)
        sim.simxSetJointTargetVelocity(clientID, rightJointDynamic, linearVelocityRight/wheelRadius, sim.simx_opmode_blocking)

        sim.simxSetJointTargetVelocity(clientID, leftJointDynamic, 0/wheelRadius, sim.simx_opmode_blocking)
        sim.simxSetJointTargetVelocity(clientID, rightJointDynamic, 0/wheelRadius, sim.simx_opmode_blocking)

    else:
        print("Error: no se puede conectar")
        sys.exit("Error: no se puede conectar")


main()
