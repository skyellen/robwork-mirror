const std::string NEW_LINE= "\n"; const std::string QUOTATION = "\""; std::string UR_SCRIPT = "def myprog():" + NEW_LINE  + 
"    set_digital_out(3, True)" + NEW_LINE  + 
"    global qtarget = [ 0,0,0,0,0,0]" + NEW_LINE  + 
"    global posetarget = [ 0,0,0,0,0,0]" + NEW_LINE  + 
"    global dqtarget = [ 0,0,0,0,0,0 ]" + NEW_LINE  + 
"    global speed = 0.75" + NEW_LINE  + 
"    global thrd  = -1" + NEW_LINE  + 
"    global motionFinished = 0" + NEW_LINE  + 
"    global isServoing = 0" + NEW_LINE  + 
"    global isStopped = 1" + NEW_LINE  + 
"    global receive_buffer = [8, 0, 0, 0, 0, 0, 0, 0, 0]" + NEW_LINE  + 
"    global receive_buffer18 = [18, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]" + NEW_LINE  + 
"    global FLOAT_SCALE = 0.0001  " + NEW_LINE  + 
"    global force_selection = [ 0,0,0,0,0,0]" + NEW_LINE  + 
"    global wrench = [ 0,0,0,0,0,0]" + NEW_LINE  + 
"    global force_limits = [ 0,0,0,0,0,0]" + NEW_LINE  + 
"    global force_frame = p[0,0,0,0,0,0]" + NEW_LINE  + 
"" + NEW_LINE  + 
"    def stopRobot():" + NEW_LINE  + 
"        enter_critical	  " + NEW_LINE  + 
"        if thrd != -1:" + NEW_LINE  + 
"            kill thrd" + NEW_LINE  + 
"            thrd = -1" + NEW_LINE  + 
"        end" + NEW_LINE  + 
"        exit_critical" + NEW_LINE  + 
"        textmsg("+QUOTATION+"Stop Robot"+QUOTATION+")" + NEW_LINE  + 
"        stopj(10)" + NEW_LINE  + 
"        isServoing = 0" + NEW_LINE  + 
"        isStopped = 1" + NEW_LINE  + 
"    end" + NEW_LINE  + 
"" + NEW_LINE  + 
"" + NEW_LINE  + 
"    #Thread for running the movej command" + NEW_LINE  + 
"    thread moveJthread():" + NEW_LINE  + 
"        textmsg("+QUOTATION+"Calls movej"+QUOTATION+")" + NEW_LINE  + 
"        textmsg(qtarget)" + NEW_LINE  + 
"        textmsg(speed)" + NEW_LINE  + 
"        movej(qtarget)" + NEW_LINE  + 
"        textmsg("+QUOTATION+"MoveJ called"+QUOTATION+")" + NEW_LINE  + 
"        #We reset our thread handle to -1 to indicate that the motion is finish." + NEW_LINE  + 
"        #This is done in a critical section to avoid race conditions" + NEW_LINE  + 
"        enter_critical" + NEW_LINE  + 
"        thrd = -1" + NEW_LINE  + 
"        motionFinished = 1" + NEW_LINE  + 
"        exit_critical" + NEW_LINE  + 
"        textmsg("+QUOTATION+"MoveJ done"+QUOTATION+")" + NEW_LINE  + 
"    end" + NEW_LINE  + 
"" + NEW_LINE  + 
"    #Thread for running the movel command" + NEW_LINE  + 
"	thread moveLthread():" + NEW_LINE  + 
"        textmsg("+QUOTATION+"Calls moveT"+QUOTATION+")" + NEW_LINE  + 
"        movel(posetarget)" + NEW_LINE  + 
"" + NEW_LINE  + 
"        #We reset our thread handle to -1 to indicate that the motion is finish." + NEW_LINE  + 
"        #This is done in a critical section to avoid race conditions" + NEW_LINE  + 
"        enter_critical	" + NEW_LINE  + 
"        thrd = -1" + NEW_LINE  + 
"        motionFinished = 1" + NEW_LINE  + 
"        exit_critical" + NEW_LINE  + 
"    end" + NEW_LINE  + 
"" + NEW_LINE  + 
"" + NEW_LINE  + 
"    #Thread for running the servoj comand. The thread constantly updates the target " + NEW_LINE  + 
"    #of the servoj command to the most recently given target." + NEW_LINE  + 
"    thread servoQthread():" + NEW_LINE  + 
"        textmsg("+QUOTATION+"Start ServoQ Thread"+QUOTATION+")" + NEW_LINE  + 
"        while 1 == 1:" + NEW_LINE  + 
"            enter_critical" + NEW_LINE  + 
"            q = qtarget" + NEW_LINE  + 
"            exit_critical" + NEW_LINE  + 
"            #textmsg("+QUOTATION+"Calls ServoJ"+QUOTATION+")" + NEW_LINE  + 
"            servoj(q, 3, 0.75, 0.008)            " + NEW_LINE  + 
"        end" + NEW_LINE  + 
"        enter_critical" + NEW_LINE  + 
"        thrd = -1" + NEW_LINE  + 
"        motionFinished = 1" + NEW_LINE  + 
"        exit_critical" + NEW_LINE  + 
"    end" + NEW_LINE  + 
"" + NEW_LINE  + 
"" + NEW_LINE  + 
"" + NEW_LINE  + 
"    def moveQ():" + NEW_LINE  + 
"        textmsg("+QUOTATION+"MoveQ"+QUOTATION+")" + NEW_LINE  + 
"        cnt = 0           " + NEW_LINE  + 
"        enter_critical  " + NEW_LINE  + 
"        motionFinished = 0      " + NEW_LINE  + 
"        while cnt < 6:" + NEW_LINE  + 
"            qtarget[cnt] = receive_buffer[cnt+2]*FLOAT_SCALE" + NEW_LINE  + 
"            cnt = cnt + 1" + NEW_LINE  + 
"        end" + NEW_LINE  + 
"        exit_critical" + NEW_LINE  + 
"        textmsg(qtarget)" + NEW_LINE  + 
"        textmsg("+QUOTATION+"Get Speed"+QUOTATION+")" + NEW_LINE  + 
"        speed = receive_buffer[7]*FLOAT_SCALE" + NEW_LINE  + 
"      	textmsg("+QUOTATION+"speed "+QUOTATION+")" + NEW_LINE  + 
"        textmsg(speed)" + NEW_LINE  + 
"        enter_critical" + NEW_LINE  + 
"        if thrd != -1:			" + NEW_LINE  + 
"            textmsg("+QUOTATION+"Kills old thread"+QUOTATION+")" + NEW_LINE  + 
"            kill thrd" + NEW_LINE  + 
"            isServoing = 0" + NEW_LINE  + 
"            textmsg("+QUOTATION+"Continues after kill"+QUOTATION+")" + NEW_LINE  + 
"            thrd = -1" + NEW_LINE  + 
"        end" + NEW_LINE  + 
"        exit_critical" + NEW_LINE  + 
"        " + NEW_LINE  + 
"        enter_critical" + NEW_LINE  + 
"        #We only wish to start a new thread if our previous motion is finished" + NEW_LINE  + 
"        if thrd == -1:" + NEW_LINE  + 
"            thrd = run moveJthread()" + NEW_LINE  + 
"        end" + NEW_LINE  + 
"        exit_critical" + NEW_LINE  + 
"    end" + NEW_LINE  + 
"" + NEW_LINE  + 
"    def moveT():" + NEW_LINE  + 
"        #Reads in x,y,z,ax,ay,az,speed" + NEW_LINE  + 
"        motionFinished = 0" + NEW_LINE  + 
"        #pose = socket_read_ascii_float(7)" + NEW_LINE  + 
"        #textmsg(pose)" + NEW_LINE  + 
"        #cnt = 0" + NEW_LINE  + 
"        #while cnt < pose[0]-1:" + NEW_LINE  + 
"            #posetarget[cnt] = pose[cnt+1]" + NEW_LINE  + 
"            #cnt = cnt + 1" + NEW_LINE  + 
"        #end" + NEW_LINE  + 
"        #q[0] is the length of the data, hence q[q[0]] is the last element" + NEW_LINE  + 
"        #speed = pose[pose[0]]" + NEW_LINE  + 
"        #textmsg("+QUOTATION+"speed "+QUOTATION+")" + NEW_LINE  + 
"        #textmsg(speed)" + NEW_LINE  + 
"        #if thrd != -1:" + NEW_LINE  + 
"            #Kill thrd" + NEW_LINE  + 
"        #end" + NEW_LINE  + 
"        #enter_critical" + NEW_LINE  + 
"        #We only wish to start a new thread if our previous motion is finished" + NEW_LINE  + 
"        #if thrd == -1:" + NEW_LINE  + 
"            #thrd = run moveLthread()" + NEW_LINE  + 
"        #end" + NEW_LINE  + 
"        #exit_critical" + NEW_LINE  + 
"    end" + NEW_LINE  + 
"" + NEW_LINE  + 
"    def servoQ():" + NEW_LINE  + 
"        cnt = 0           " + NEW_LINE  + 
"        enter_critical        " + NEW_LINE  + 
"        while cnt < 6:" + NEW_LINE  + 
"            qtarget[cnt] = receive_buffer[cnt+2]*FLOAT_SCALE" + NEW_LINE  + 
"            cnt = cnt + 1" + NEW_LINE  + 
"        end" + NEW_LINE  + 
"        exit_critical" + NEW_LINE  + 
"" + NEW_LINE  + 
"  		#textmsg(isServoing)" + NEW_LINE  + 
"" + NEW_LINE  + 
"        enter_critical" + NEW_LINE  + 
"        if isServoing == 0:" + NEW_LINE  + 
"            textmsg(qtarget)" + NEW_LINE  + 
"            isServoing = 1 " + NEW_LINE  + 
"            if thrd == -1:" + NEW_LINE  + 
"                textmsg("+QUOTATION+"Start ServoQThread"+QUOTATION+")" + NEW_LINE  + 
"                thrd = run servoQthread()" + NEW_LINE  + 
"            end" + NEW_LINE  + 
"        end" + NEW_LINE  + 
"        exit_critical" + NEW_LINE  + 
"    end" + NEW_LINE  + 
"" + NEW_LINE  + 
"    def force_mode_start():" + NEW_LINE  + 
"        cnt = 0           " + NEW_LINE  + 
"        #enter_critical        " + NEW_LINE  + 
"        while cnt < 6:" + NEW_LINE  + 
"            force_frame[cnt] = receive_buffer[cnt+2]*FLOAT_SCALE" + NEW_LINE  + 
"            cnt = cnt + 1" + NEW_LINE  + 
"        end" + NEW_LINE  + 
"" + NEW_LINE  + 
"		receive_buffer18 = socket_read_binary_integer(18)" + NEW_LINE  + 
"" + NEW_LINE  + 
"        cnt = 0           " + NEW_LINE  + 
"        while cnt < 6:" + NEW_LINE  + 
"        	force_selection[cnt] = receive_buffer18[cnt+1]*FLOAT_SCALE" + NEW_LINE  + 
"            wrench[cnt] = receive_buffer18[cnt+1+6]*FLOAT_SCALE" + NEW_LINE  + 
"            force_limits[cnt] = receive_buffer18[cnt+1+12]*FLOAT_SCALE" + NEW_LINE  + 
"            cnt = cnt + 1" + NEW_LINE  + 
"        end" + NEW_LINE  + 
"" + NEW_LINE  + 
"        textmsg("+QUOTATION+"Force Frame: "+QUOTATION+")" + NEW_LINE  + 
"        textmsg(force_frame)" + NEW_LINE  + 
"        textmsg("+QUOTATION+"Force Selection: "+QUOTATION+")" + NEW_LINE  + 
"        textmsg(force_selection)" + NEW_LINE  + 
"        textmsg("+QUOTATION+"Wrench:"+QUOTATION+")" + NEW_LINE  + 
"        textmsg(wrench)" + NEW_LINE  + 
"        textmsg("+QUOTATION+"Force Limits:"+QUOTATION+")" + NEW_LINE  + 
"        textmsg(force_limits)" + NEW_LINE  + 
"        force_mode(force_frame, force_selection, wrench, 2, force_limits)" + NEW_LINE  + 
"" + NEW_LINE  + 
"        #exit_critical" + NEW_LINE  + 
"    end" + NEW_LINE  + 
"" + NEW_LINE  + 
"	" + NEW_LINE  + 
"    def force_mode_update():" + NEW_LINE  + 
"        cnt = 0           " + NEW_LINE  + 
"        #enter_critical        " + NEW_LINE  + 
"        while cnt < 6:" + NEW_LINE  + 
"            wrench[cnt] = receive_buffer[cnt+2]*FLOAT_SCALE" + NEW_LINE  + 
"            cnt = cnt + 1" + NEW_LINE  + 
"        end" + NEW_LINE  + 
"        textmsg("+QUOTATION+"Wrench Update:"+QUOTATION+")" + NEW_LINE  + 
"        textmsg(wrench)" + NEW_LINE  + 
"" + NEW_LINE  + 
"        force_mode(force_frame, force_selection, wrench, 2, force_limits)" + NEW_LINE  + 
"" + NEW_LINE  + 
"        #exit_critical" + NEW_LINE  + 
"    end" + NEW_LINE  + 
"	" + NEW_LINE  + 
"	" + NEW_LINE  + 
"	def force_mode_end():" + NEW_LINE  + 
"        force_mode_end()" + NEW_LINE  + 
"	end" + NEW_LINE  + 
"" + NEW_LINE  + 
"#" + NEW_LINE  + 
"# The main loop is running below" + NEW_LINE  + 
"#" + NEW_LINE  + 
"" + NEW_LINE  + 
"    #Setup the host name" + NEW_LINE  + 
"    host = HOST" + NEW_LINE  + 
"    port = PORT" + NEW_LINE  + 
"    opened = socket_open(host, port)" + NEW_LINE  + 
"    textmsg("+QUOTATION+"Socket Status"+QUOTATION+")" + NEW_LINE  + 
"    textmsg(opened)" + NEW_LINE  + 
"" + NEW_LINE  + 
"    while opened == False:" + NEW_LINE  + 
"        opened = socket_open(host, port)" + NEW_LINE  + 
"    end " + NEW_LINE  + 
"" + NEW_LINE  + 
"    textmsg("+QUOTATION+"Socket opened !!"+QUOTATION+")" + NEW_LINE  + 
"    errcnt = 0" + NEW_LINE  + 
"    while errcnt < 1:" + NEW_LINE  + 
" 		#textmsg("+QUOTATION+"running"+QUOTATION+")" + NEW_LINE  + 
"        if motionFinished == 1:" + NEW_LINE  + 
"            #textmsg("+QUOTATION+"Sends finished"+QUOTATION+")" + NEW_LINE  + 
"            socket_send_byte(0)" + NEW_LINE  + 
"        else:" + NEW_LINE  + 
"            socket_send_byte(1)" + NEW_LINE  + 
"        end" + NEW_LINE  + 
"        " + NEW_LINE  + 
"        receive_buffer = socket_read_binary_integer(8)" + NEW_LINE  + 
"        #textmsg(receive_buffer)" + NEW_LINE  + 
"        if receive_buffer[0] != 8:" + NEW_LINE  + 
"            stopRobot()" + NEW_LINE  + 
"            errcnt = errcnt + 1" + NEW_LINE  + 
"        elif receive_buffer[1] == 0: #0: Stop Robot" + NEW_LINE  + 
"        	if isStopped == 0:" + NEW_LINE  + 
"            	stopRobot()" + NEW_LINE  + 
"            end            " + NEW_LINE  + 
"        elif receive_buffer[1] == 1: #1: Move to Q" + NEW_LINE  + 
"        	isStopped = 0" + NEW_LINE  + 
"            moveQ()" + NEW_LINE  + 
"        elif receive_buffer[1] == 2: #2: Move to T" + NEW_LINE  + 
"        	isStopped = 0" + NEW_LINE  + 
"            moveT()" + NEW_LINE  + 
"        elif receive_buffer[1] == 3: #3: Servo to T" + NEW_LINE  + 
"        	isStopped = 0" + NEW_LINE  + 
"            servoQ()" + NEW_LINE  + 
"        elif receive_buffer[1] == 4: #4: Start Force Mode Base" + NEW_LINE  + 
"        	textmsg("+QUOTATION+"Force Mode Start"+QUOTATION+")" + NEW_LINE  + 
"            isStopped = 0" + NEW_LINE  + 
"            force_mode_start()" + NEW_LINE  + 
"        elif receive_buffer[1] == 5: #5: Force Mode Update" + NEW_LINE  + 
"            isStopped = 0" + NEW_LINE  + 
"            force_mode_update()" + NEW_LINE  + 
"        elif receive_buffer[1] == 6: #6: End Force Mode" + NEW_LINE  + 
"            force_mode_end()" + NEW_LINE  + 
"        elif receive_buffer[1] == 9999: #1: Do nothing" + NEW_LINE  + 
"        	isStopped = 0" + NEW_LINE  + 
"            #Right motion already taken" + NEW_LINE  + 
"        end" + NEW_LINE  + 
"    " + NEW_LINE  + 
"        #if motionFinished == 1:" + NEW_LINE  + 
"        #textmsg("+QUOTATION+"Sends finished"+QUOTATION+")" + NEW_LINE  + 
"        #socket_set_var("+QUOTATION+"FIN"+QUOTATION+", 1)" + NEW_LINE  + 
"        #motionFinished = 0" + NEW_LINE  + 
"        #end" + NEW_LINE  + 
"    end #end for While True:" + NEW_LINE  + 
"    textmsg("+QUOTATION+"Program Finished"+QUOTATION+")" + NEW_LINE  + 
"end" + NEW_LINE  + 
"run program" + NEW_LINE  + 
"";