MODULE server

    !==== variables ====!
    VAR socketdev server_socket;
    VAR socketdev client_socket;

    CONST num delay_time:=0.2;

    ! process variables
    VAR string message:="";
    VAR robtarget hand_frame;
    VAR num message_index:=-1;
    
    VAR robtarget cup_end_frame:=[[0,0,0],[0,0,0,0],[1,1,0,0],[11,12.3,9E9,9E9,9E9,9E9]];
    ! dummy values

    CONST num min_z_value := 120;
    VAR num position_in_file_index;
    
    ! Open socket connection
PROC server_init()
    VAR string pc_ip := "192.168.125.201";  ! IP of PC
    VAR string robot_bind_ip := "192.168.125.1"; ! listen on all ports
    VAR num port := 1025;

    SocketClose server_socket;
    SocketClose client_socket;
    SocketCreate server_socket;
    
    ! 1. Bind to "" (Listen on all local interfaces)
    SocketBind server_socket, robot_bind_ip, port; 
    SocketListen server_socket;
    
    TPWrite "Robot is waiting for ANY PC to connect...";
    
    ! 2. Since pc_ip is "", the robot will accept your PC (.201)
    ! It will then WRITE "192.168.125.201" into pc_ip.
    SocketAccept server_socket, client_socket \ClientAddress := pc_ip;
    
    TPWrite "Connected to: " + pc_ip;
ENDPROC

    ! hold comminication while client is connected
    ! close communication if timer runs out or clinet close communication
    PROC single_client_communication()
        VAR mug_vector buffer;
        ! only want one clinet, therefore we do not need to open other ports and arange new connections!

        ! while we want to have a communication we keep on having one
        WHILE TRUE DO

!            SocketReceive client_socket\Str:=message\Time:=30; !you have 30 sec to send message or conneciton closes
            
            SocketReceive client_socket\Str:=message;

            ! switch case
            TEST message

            CASE "Connection_test": 
                TPWrite("[INFO] client is sending test message");
                SocketSend client_socket\Str:="Connection_Confirmed";
                
            CASE "Get_Coordinates": 
                sendHandCoordinates;
                
            CASE "Move":
                Move;    
                
            CASE "Home":
                moveToHomeTarget;

            CASE "Presentation":
                Presentation;
                
            CASE "Pick_Up_Sequence":
                 pickupSequence; 
                 
            CASE "Leave_Sequence":
                leaveSequence;
                
            CASE "Move_Calibration_Position": 
                calibrationMovement;
                
            CASE "Move_Calibration_home": 
                calibrationMoveHome;
            CASE "Presentation":
                Presentation;
                           
            DEFAULT:
                TPWrite("[INFO] message from client: "+message);
                SocketSend client_socket\Str:="default_"+message;
            ENDTEST
            WaitTime(delay_time);
            SocketSend client_socket\Str:="Ask_next"; ! ask for next "order"

        ENDWHILE

    ERROR
        ! if errors occure during run
        IF ERRNO=ERR_SOCK_CLOSED THEN
            ! clinet closed connection before sending end ack!
            server_init;
            RETURN ;
        ELSEIF ERRNO=ERR_SOCK_TIMEOUT THEN
            SocketClose client_socket;
            ! socket never send annything, close connection and return to main
            RETURN ;
        ENDIF
    ENDPROC


ENDMODULE