MODULE LeftArmMain   
!    ***********************************************************
!     Module:  LeftArmMain
!     Description:  Main module for arm movement which works together with the communication module.
!    ***********************************************************

    CONST robtarget home_target := [[609,13,136],[0.56458,0.45107,0.48932,0.48820],[-1,-1,0,4],[-177.987,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget calib_target_outofway:=[[-155.85,250.06,761.03],[0.903777,0.225568,-0.131867,-0.338994],[-1,1,-1,4],[-147.134,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget calib_home_target := [[442.004,-92.0926,300.604],[0.0189937,-0.0236138,0.999427,-0.0150419],[-1,1,-1,4],[-152.666,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget home_target_v2:=[[247.91,314.47,202.37],[0.680648,0.280046,0.674068,0.0626469],[0,-2,0,5],[107.401,9E+09,9E+09,9E+09,9E+09,9E+09]];   
    CONST robtarget home_target_v3:=[[357.9,284.46,274.39],[0.195938,0.546826,-0.570092,0.581021],[0,0,0,4],[175.044,9E+09,9E+09,9E+09,9E+09,9E+09]]; 
    CONST robtarget EGM_starting_point := [[442.004,-92.0926,171.604],[0.0189937,-0.0236138,0.999427,-0.0150419],[-1,1,-1,4],[-152.666,9E+09,9E+09,9E+09,9E+09,9E+09]];
    
    ! used for mug manipulation
    CONST pos sholder_pos_close := [110,200,460];
    CONST pos sholder_pos_far := [1000,800,460];
    CONST robtarget cup_target := [[499.548,-110.253,-46.3938],[0.0565402,0.114235,0.990146,-0.0580089],[-2,-3,-1,4],[-177.807,9E+09,9E+09,9E+09,9E+09,9E+09]]; 
        
    CONST speeddata movement_speed := v1000;  !1000
    CONST speeddata pick_speed := v200; !400
    CONST speeddata transit_speed := v1500; ! Optimization: for fast resets
    CONST num max_magnitude := 300;         
    
    ! SPEED OPTIMIZATION: Increased step_size for smoother, faster calculation
    CONST num step_size := 150;             
        
    ! used in basic movement
    CONST num x_offset := 10;                
    CONST num y_offset := -10;                
    CONST num z_offset := 0;        
    
    ! used when fetching a mug
    CONST num gripper_offset := 0;    
    CONST num pick_offset := 100;
    CONST num offset_z_when_fetching := 100;
    
    PROC main()
        VAR mug_vector buffer; ! RESTORED
        
        g_calibrate; 
        TPErase;     
        
        ! Optimization: Config commands outside loop
        ConfJ\On;   
        ConfL\On;
        
        moveToHomeTarget;

        WHILE TRUE DO
            ! Synchronized Handshake with Communication Task
            WaitUntil shared_movement_left.wait_flag = TRUE;
            
            TEST shared_movement_left.flag 
            CASE flag_ERROR:
                TPWrite("ERROR OCCURED");
                StopMove;
                STOP;

            CASE flag_move: 
                MovementProc Offs(shared_movement_left.target,x_offset,y_offset,z_offset), step_size,max_magnitude, movement_speed;

       !     CASE flag_move_home: 
        !        MoveToHome; 
                
            CASE flag_gripper_grip: 
                g_gripIn;   
                
            CASE flag_gripper_release: 
                g_gripOut; 
                
            CASE flag_move_home_target: 
                moveToHomeTarget;
                
            CASE flag_move_EGM: 
                ! Optimization: Smooth approach to EGM point
              !  MoveJ EGM_starting_point,movement_speed,fine,tGripper;
                !EGMfollowCup;
                
            CASE flag_move_calibration:
                ! Optimization: Blended movement through home target
                MoveJ calib_home_target,transit_speed,z50,tGripper;
                MoveJ shared_movement_left.target,movement_speed,fine,tGripper;
                
            CASE flag_move_calibration_home:
                MoveJ calib_home_target,movement_speed,fine,tGripper;

             CASE flag_move_calibration_outofway: 
                MoveJ calib_target_outofway,transit_speed,z50,tGripper;
  
            CASE flag_pick_up_mug: 
                FetchMug shared_movement_left.mug.position,pick_offset,shared_movement_left.mug.normal;  
                
            CASE flag_leave_mug: 
                 ! Optimization: Uses standard normal for safer release
                 LeaveMug shared_movement_left.mug.position,[0,0,1],pick_offset;
            
            CASE flag_hand_over: 
                ! Logic inside handOverSequence handles synchronization
                handOverSequence;
                
            ENDTEST
            
            ! Completion Signal
            shared_movement_left.wait_flag := FALSE;
        ENDWHILE
    ENDPROC
ENDMODULE