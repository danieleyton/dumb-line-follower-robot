import curses
import sim
import sys
import numpy as np

def automaticDrive(stdscr):
    stdscr.clear()
    stdscr.nodelay(1)
    stdscr.border(0)
    stdscr.addstr(2, 10, "En modo Automatico...")
    stdscr.addstr(4, 10, "Presione 'q' para salir")
    stdscr.refresh()

    key = ''
    while key != ord('q'):
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
        #print(dataLeft[0][10])
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

        key = stdscr.getch()
    stdscr.clear()
    curses.endwin()

def manualControl(stdscr):
    # nu se unu
    #stdscr = curses.newwin(40, 40, 4, 7)
    stdscr.clear()
    stdscr.nodelay(0)
    stdscr.border(0)
    stdscr.addstr(0,10,"Presione 'q' para salir")
    stdscr.refresh()

    key = ''
    while key != ord('q'):
        key = stdscr.getch()
        stdscr.addch(20,25,key)
        stdscr.refresh()
        if key == curses.KEY_UP:
            stdscr.addstr(2, 20, "Up")
            # mover robot hacia adelante
            linearVelocityLeft = 0.3
            linearVelocityRight = 0.3
            sim.simxSetJointTargetVelocity(clientID, leftJointDynamic, linearVelocityLeft/wheelRadius, sim.simx_opmode_blocking)
            sim.simxSetJointTargetVelocity(clientID, rightJointDynamic, linearVelocityRight/wheelRadius, sim.simx_opmode_blocking)

        elif key == curses.KEY_DOWN:
            stdscr.addstr(3, 20, "Down")
            # mover robot hacia atras ??
            linearVelocityLeft = 0
            linearVelocityRight = 0
            sim.simxSetJointTargetVelocity(clientID, leftJointDynamic, linearVelocityLeft/wheelRadius, sim.simx_opmode_blocking)
            sim.simxSetJointTargetVelocity(clientID, rightJointDynamic, linearVelocityRight/wheelRadius, sim.simx_opmode_blocking)
        elif key == curses.KEY_LEFT:
            stdscr.addstr(4, 20, "Left")
            linearVelocityLeft = 0.3*0.3
            linearVelocityRight = 0.3
            sim.simxSetJointTargetVelocity(clientID, leftJointDynamic, linearVelocityLeft/wheelRadius, sim.simx_opmode_blocking)
            sim.simxSetJointTargetVelocity(clientID, rightJointDynamic, linearVelocityRight/wheelRadius, sim.simx_opmode_blocking)
        elif key == curses.KEY_RIGHT:
            stdscr.addstr(5, 20, "Right")
            linearVelocityLeft = 0.3
            linearVelocityRight = 0.3*0.3
            sim.simxSetJointTargetVelocity(clientID, leftJointDynamic, linearVelocityLeft/wheelRadius, sim.simx_opmode_blocking)
            sim.simxSetJointTargetVelocity(clientID, rightJointDynamic, linearVelocityRight/wheelRadius, sim.simx_opmode_blocking)
    stdscr.clear()
    curses.endwin()

def main(stdscr): 
    curses.noecho()
    curses.cbreak()
    stdscr.keypad(True)
    stdscr.border(0)
    sim.simxFinish(-1) # Terminar todas las conexiones
    global clientID, lineTracerBase, leftSensor, middleSensor, rightSensor, leftJoin, rightJoint, leftJointDynamic, rightJointDynamic, nominalLinearVelocity, wheelRadius, interWheelDistance
    clientID=sim.simxStart('127.0.0.1',19999,True,True,5000,5) # Iniciar una nueva conexion

    if clientID != -1:
        stdscr.addstr(4, 10, "Conexion establecida")
        res, lineTracerBase = sim.simxGetObjectHandle(clientID, 'LineTracerBase', sim.simx_opmode_blocking)
        res, leftSensor = sim.simxGetObjectHandle(clientID, 'LeftSensor', sim.simx_opmode_blocking)
        res, middleSensor = sim.simxGetObjectHandle(clientID, 'MiddleSensor', sim.simx_opmode_blocking)
        res, rightSensor = sim.simxGetObjectHandle(clientID, 'RightSensor', sim.simx_opmode_blocking)
        res, leftJoin = sim.simxGetObjectHandle(clientID, 'LeftJoint', sim.simx_opmode_blocking)
        res, rightJoint= sim.simxGetObjectHandle(clientID, 'RightJoint', sim.simx_opmode_blocking)
        res, leftJointDynamic= sim.simxGetObjectHandle(clientID, 'DynamicLeftJoint', sim.simx_opmode_blocking)
        res, rightJointDynamic= sim.simxGetObjectHandle(clientID, 'DynamicRightJoint', sim.simx_opmode_blocking)
        nominalLinearVelocity = 0.09
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

        opt = 0
        stdscr.clear()
        stdscr.border()
        while opt != ord('3'):
            stdscr.addstr(4, 10, "Para usar el modo automatico presione 1")
            stdscr.addstr(6, 10, "Para usar el modo manual presione 2")
            stdscr.addstr(8, 10, "Para salir presione 3")
            opt = stdscr.getch()

            if opt == ord('1'):
                # enter modo automatico
                stdscr.addstr(2,10, "Enter modo automatico")
                automaticDrive(stdscr)
            elif opt == ord('2'):
                # enter modo manual
                stdscr.addstr(4, 10, "Enter modo manual")
                manualControl(stdscr)
            if opt == ord('3'):
                # quit
                stdscr.clear()
                stdscr.addstr(1, 10, "Saliendo del programa..")
                curses.endwin()


        
    else:
        stdscr.addstr(4, 10, "Error: no se puede conectar")
        sys.exit("Error: no se puede conectar")

curses.wrapper(main)
main()
