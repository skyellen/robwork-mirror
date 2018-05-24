def rwhw_urscript():
	textmsg("PROGRAM STARTED")
    global qtarget = [ 0,0,0,0,0,0]
    global posetarget = p[ 0,0,0,0,0,0]
    global dqtarget = [ 0,0,0,0,0,0 ]
    global acceleration = 1.4
    global tool_acceleration = 1.2
    global speed = 0.75
    global time = 0.0
    global blend = 0.0
    global motionFinished = 0
    global isServoing = 1
    global isStopped = 1
    global receive_buffer = [9, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    global receive_buffer18 = [18, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    global FLOAT_SCALE = 0.0001  
    global force_selection = [ 0,0,0,0,0,0]
    global wrench = [ 0,0,0,0,0,0]
    global force_limits = [ 0,0,0,0,0,0]
    global force_frame = p[0,0,0,0,0,0]
    global mass = 0.0
    global center_of_gravity = [0, 0, 0]
	
    def stopRobot():
        textmsg("Stopping Robot, decelerating joint speeds to zero!")
        stopj(10) # decelerate with 10 rad/s^2
        isServoing = 0
        isStopped = 1
    end

    def moveQ():
        textmsg("MoveQ:")
        cnt = 0           
        enter_critical  
        motionFinished = 0      
        while cnt < 6:
            qtarget[cnt] = receive_buffer[cnt+2]*FLOAT_SCALE
            cnt = cnt + 1
        end
        exit_critical
        textmsg(qtarget)

        # Set speed and blend
        speed = receive_buffer[8]*FLOAT_SCALE
        blend = receive_buffer[9]*FLOAT_SCALE

        textmsg("calling movej")
        movej(qtarget, acceleration, speed, time, blend)
        enter_critical
        motionFinished = 1
        exit_critical
        textmsg("movej done")
    end

    def moveT():
		textmsg("MoveT:")
        cnt = 0
		enter_critical
		motionFinished = 0
        while cnt < 6:
            posetarget[cnt] = receive_buffer[cnt+2]*FLOAT_SCALE
            cnt = cnt + 1
        end
		exit_critical
		
        textmsg(posetarget)

        # Set speed and blend
        speed = receive_buffer[8]*FLOAT_SCALE
        blend = receive_buffer[9]*FLOAT_SCALE

        textmsg("calling movel")
        movel(posetarget, tool_acceleration, speed, time, blend)

        enter_critical
        motionFinished = 1
        exit_critical
        textmsg("movel done")
    end

    def servoQ():
        cnt = 0           
        enter_critical        
        while cnt < 6:
            qtarget[cnt] = receive_buffer[cnt+2]*FLOAT_SCALE
            cnt = cnt + 1
        end
        exit_critical

        # Output target
        textmsg(qtarget)

        # We only servo if isServoing is 1, this is default and is changed by stopRobot
        if isServoing == 1:
            enter_critical
            q = qtarget
            exit_critical
$CB3        servoj(q, a=0, v=0, t=0.008, lookahead_time=0.1, gain=300)
$CB2        servoj(q, 3, 0.75, 0.008)
        end

        enter_critical
        motionFinished = 1
        exit_critical
    end

    def force_mode_start():
        cnt = 0
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
    end

	
    def force_mode_update():
        cnt = 0
        while cnt < 6:
            wrench[cnt] = receive_buffer[cnt+2]*FLOAT_SCALE
            cnt = cnt + 1
        end

        force_mode(force_frame, force_selection, wrench, 2, force_limits)
    end
	
	
	def force_mode_end():
        end_force_mode()
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
$CB3    	set_standard_digital_out(id, True)
$CB2    	set_digital_out(id, True)
		else:
$CB3    	set_standard_digital_out(id, False)
$CB2    	set_digital_out(id, False)
		end	
	end
	
	
	def set_tcp_payload():
		textmsg("Setting TCP payload")
    	cnt = 0
		mass = receive_buffer[cnt+2]*FLOAT_SCALE
		cnt = cnt + 1
		while cnt < 4:
			center_of_gravity[cnt - 1] = receive_buffer[cnt+2]*FLOAT_SCALE
			cnt = cnt + 1
		end
	
		textmsg("New payload: ")
		textmsg(mass)
		textmsg("Center of gravity: ")
		textmsg(center_of_gravity)
		
		set_payload(mass, center_of_gravity) 
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
		receive_buffer = socket_read_binary_integer(9)

        if motionFinished == 1:
            #textmsg("Sends finished")
            socket_send_byte(0)
        else:
            socket_send_byte(1)
        end

        #textmsg(receive_buffer)
        if receive_buffer[0] != 9:
			textmsg("Did not receive 9 integers as expected")
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
        elif receive_buffer[1] == 3: #3: Servo to Q
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
		elif receive_buffer[1] == 10: #10: Set Payload
			set_tcp_payload()	
        elif receive_buffer[1] == 9999: #1: Do nothing
        	isStopped = 0
            #Right motion already taken
        end

    end
    textmsg("Program Finished")
end
run program
