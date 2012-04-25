def myprog():
    set_digital_out(3, True)
    global qtarget = [ 0,0,0,0,0,0]
    global posetarget = [ 0,0,0,0,0,0]
    global dqtarget = [ 0,0,0,0,0,0 ]
    global speed = 0.75
    global thrd  = -1
    global motionFinished = 0
    global isServoing = 0
    global receive_buffer = [8, 0, 0, 0, 0, 0, 0, 0, 0]
    global FLOAT_SCALE = 0.0001

    def stopRobot():
        enter_critical	  
        if thrd != -1:
            kill thrd
            thrd = -1
        end
        exit_critical
        textmsg("Stop Robot")
        stopj(10)
        isServoing = 0
    end


    #Thread for running the movej command
    thread moveJthread():
        textmsg("Calls movej")
        textmsg(qtarget)
        textmsg(speed)
        movej(qtarget)
        textmsg("MoveJ called")
        #We reset our thread handle to -1 to indicate that the motion is finish.
        #This is done in a critical section to avoid race conditions
        enter_critical
        thrd = -1
        motionFinished = 1
        exit_critical
    end

    #Thread for running the movel command
        thread moveLthread():
        textmsg("Calls moveT")
        movel(posetarget)

        #We reset our thread handle to -1 to indicate that the motion is finish.
        #This is done in a critical section to avoid race conditions
        enter_critical	
        thrd = -1
        motionFinished = 1
        exit_critical
    end


    #Thread for running the servoj comand. The thread constantly updates the target 
    #of the servoj command to the most recently given target.
    thread servoQthread():
        textmsg("Start ServoQ Thread")
        while 1 == 1:
            enter_critical
            q = qtarget
            exit_critical
            #textmsg("Calls ServoJ")
            servoj(q, 3, 0.75, 0.008)            
        end
    end



    def moveQ():
        textmsg("MoveQ")
        cnt = 0           
        enter_critical  
        motionFinished = 0      
        while cnt < 6:
            qtarget[cnt] = receive_buffer[cnt+2]*FLOAT_SCALE
            cnt = cnt + 1
        end
        exit_critical
        textmsg(qtarget)
        textmsg("Get Speed")
        speed = receive_buffer[7]*FLOAT_SCALE
      	textmsg("speed ")
        textmsg(speed)
        if thrd != -1:
            textmsg("Kills old thread")
            kill thrd
        end
        enter_critical
        #We only wish to start a new thread if our previous motion is finished
        if thrd == -1:
            thrd = run moveJthread()
        end
        exit_critical
    end

    def moveT():
        #Reads in x,y,z,ax,ay,az,speed
        motionFinished = 0
        #pose = socket_read_ascii_float(7)
        #textmsg(pose)
        #cnt = 0
        #while cnt < pose[0]-1:
            #posetarget[cnt] = pose[cnt+1]
            #cnt = cnt + 1
        #end
        #q[0] is the length of the data, hence q[q[0]] is the last element
        #speed = pose[pose[0]]
        #textmsg("speed ")
        #textmsg(speed)
        #if thrd != -1:
            #Kill thrd
        #end
        #enter_critical
        #We only wish to start a new thread if our previous motion is finished
        #if thrd == -1:
            #thrd = run moveLthread()
        #end
        #exit_critical
    end

    def servoQ():     
        cnt = 0           
        enter_critical        
        while cnt < 6:
            qtarget[cnt] = receive_buffer[cnt+2]*FLOAT_SCALE
            cnt = cnt + 1
        end
        exit_critical

  #      textmsg(isServoing)

        enter_critical
        if isServoing == 0:
            textmsg(qtarget)
            isServoing = 1 
            if thrd == -1:
                textmsg("Start ServoQThread")
                thrd = run servoQthread()
            end
        end
        exit_critical
    end


#
# The main loop is running below
#

    #Setup the host name
    host = "192.168.100.1"
    port = PORT
    opened = socket_open(host, port)
    textmsg("Socket Status")
    textmsg(opened)

    while opened == False:
        opened = socket_open(host, port)
    end

    textmsg("Socket opened")
    errcnt = 0
    while errcnt < 1:
        if motionFinished == 1:
            textmsg("Sends finished")
            socket_send_byte(0)
        else:
            socket_send_byte(1)
        end
        
        receive_buffer = socket_read_binary_integer(8)
        #textmsg(receive_buffer)
        if receive_buffer[0] != 8:
            stopRobot()
            errcnt = errcnt + 1
        elif receive_buffer[1] == 0: #0: Stop Robot
            stopRobot()            
        elif receive_buffer[1] == 1: #1: Move to Q
            moveQ()
        elif receive_buffer[1] == 2: #2: Move to T
            moveT()
        elif receive_buffer[1] == 3: #3: Servo to T
            servoQ()
        elif receive_buffer[1] == 9999: #1: Do nothing
            #Right motion already taken
        end
    
        #if motionFinished == 1:
        #textmsg("Sends finished")
        #socket_set_var("FIN", 1)
        #motionFinished = 0
        #end
    end #end for While True:
    textmsg("Program Finished")
end
run program
