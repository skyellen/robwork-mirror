def myprog():
    global qtarget = [ 0,0,0,0,0,0]
    global posetarget = [ 0,0,0,0,0,0]
    global dqtarget = [ 0,0,0,0,0,0 ]
    global speed = 0.75
    global thrd  = -1
    global motionFinished = 0
    global isServoing = 0
    global isStopped = 1
    global receive_buffer = [8, 0, 0, 0, 0, 0, 0, 0, 0]
    global receive_buffer18 = [18, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    global FLOAT_SCALE = 0.0001  
    global force_selection = [ 0,0,0,0,0,0]
    global wrench = [ 0,0,0,0,0,0]
    global force_limits = [ 0,0,0,0,0,0]
    global force_frame = p[0,0,0,0,0,0]

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
        isStopped = 1
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
        textmsg("MoveJ done")
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
        enter_critical
        thrd = -1
        motionFinished = 1
        exit_critical
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
        enter_critical
        if thrd != -1:			
            textmsg("Kills old thread")
            kill thrd
            isServoing = 0
            textmsg("Continues after kill")
            thrd = -1
        end
        exit_critical
        
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

  		#textmsg(isServoing)

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

    def force_mode_start():
        cnt = 0           
        #enter_critical        
        while cnt < 6:
            force_frame[cnt] = receive_buffer[cnt+2]*FLOAT_SCALE
            cnt = cnt + 1
        end

		receive_buffer18 = socket_read_binary_integer(18)

        cnt = 0           
        while cnt < 6:
        	force_selection[cnt] = receive_buffer18[cnt+1]*FLOAT_SCALE;
            wrench[cnt] = receive_buffer18[cnt+1+6]*FLOAT_SCALE
            force_limits[cnt] = receive_buffer18[cnt+1+12]*FLOAT_SCALE
            cnt = cnt + 1
        end

        textmsg("Force Frame: ")
        textmsg(force_frame)
        textmsg("Force Selection: ")
        textmsg(force_selection)
        textmsg("Wrench:")
        textmsg(wrench)
        textmsg("Force Limits:")
        textmsg(force_limits)
        force_mode(force_frame, force_selection, wrench, 2, force_limits)

        #exit_critical
    end

	
    def force_mode_update():
        cnt = 0           
        #enter_critical        
        while cnt < 6:
            wrench[cnt] = receive_buffer[cnt+2]*FLOAT_SCALE
            cnt = cnt + 1
        end
        textmsg("Wrench Update:")
        textmsg(wrench)

        force_mode(force_frame, force_selection, wrench, 2, force_limits)

        #exit_critical
    end
	
	
	def force_mode_end():
        force_mode_end()
	end
	
	
    def teach_mode_start():
$CB3	textmsg("Start Teach Mode")
$CB3    teach_mode()
    end
	
    def teach_mode_end():
$CB3	textmsg("End Teach Mode")
$CB3    end_teach_mode()
    end

	def set_io():
		textmsg("Step OI")
		id = receive_buffer[2]
		onoff = receive_buffer[3]
		if onoff == 1:
	$CB3	set_standard_digital_out(id, True)
	$CB2	set_digital_out(id, True)
		else:
	$CB3	set_standard_digital_out(id, False)
	$CB2	set_digital_out(id, False)
		end	
	end
	
	
#
# The main loop is running below
#

    #Setup the host name
    host = HOST
    port = PORT
	textmsg("Host")
	textmsg(host)
	textmsg("Port")
	textmsg(port)
    opened = socket_open(host, port)
    textmsg("Socket Status")
    textmsg(opened)

    while opened == False:
        opened = socket_open(host, port)
    end 

    textmsg("Socket opened !!")
    errcnt = 0
	socket_send_byte(0)
    while errcnt < 1:       
		receive_buffer = socket_read_binary_integer(8)

        if motionFinished == 1:
            #textmsg("Sends finished")
            socket_send_byte(0)
        else:
            socket_send_byte(1)
        end

        #textmsg(receive_buffer)
        if receive_buffer[0] != 8:
			textmsg("Did not receive 8 integers as expected")
            stopRobot()
            errcnt = errcnt + 1
        elif receive_buffer[1] == 0: #0: Stop Robot
        	if isStopped == 0:
            	stopRobot()
            end            
        elif receive_buffer[1] == 1: #1: Move to Q
        	isStopped = 0
            moveQ()
        elif receive_buffer[1] == 2: #2: Move to T
        	isStopped = 0
            moveT()
        elif receive_buffer[1] == 3: #3: Servo to T
			textmsg("servo")
			isStopped = 0
            servoQ()
        elif receive_buffer[1] == 4: #4: Start Force Mode Base
        	textmsg("Force Mode Start")
            isStopped = 0
            force_mode_start()
        elif receive_buffer[1] == 5: #5: Force Mode Update
            isStopped = 0
            force_mode_update()
        elif receive_buffer[1] == 6: #6: End Force Mode
            force_mode_end()
        elif receive_buffer[1] == 7: #7: Teach mode start
            teach_mode_start()
        elif receive_buffer[1] == 8: #8: Teach mode end
            teach_mode_end()
		elif receive_buffer[1] == 9: #9: Set IO
			set_io()
        elif receive_buffer[1] == 9999: #1: Do nothing
        	isStopped = 0
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
