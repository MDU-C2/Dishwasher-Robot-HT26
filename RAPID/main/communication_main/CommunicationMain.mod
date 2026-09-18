MODULE CommunicationMain
    PROC main()
        ! Reset Flags
        shared_movement_left.wait_flag := FALSE;
        shared_movement_right.wait_flag := FALSE;
        shared_movement_left.flag := flag_nothing;
        shared_movement_right.flag := flag_nothing;
        
        TPErase;
        WHILE TRUE DO
            server_init; 
            single_client_communication; 
        ENDWHILE
    ENDPROC
ENDMODULE