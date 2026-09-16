MODULE CommunicationMain

    ! main
    
        ! hard coded mug leave pose
    CONST mug_vector mug_leave_pose := [[513.42,-441.85,110.96],[0,0,-1]];
    
    
PROC main()
    ! 1. Initialize variables
    shared_movement_left.wait_flag := FALSE;
    shared_movement_right.wait_flag := FALSE;
    shared_movement_left.flag := flag_nothing;
    shared_movement_right.flag := flag_nothing;
    
    TPErase;

    WHILE TRUE DO
        ! 2. FIX: Call server_init first! 
        ! This runs SocketCreate and waits for your Python script to connect.
        server_init; 
        
        ! 3. Only once Python is connected does it enter this loop.
        single_client_communication; 
    ENDWHILE
ENDPROC
ENDMODULE