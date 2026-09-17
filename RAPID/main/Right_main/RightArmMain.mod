MODULE RightArmMain   
!    ***********************************************************
!     Module:  RightArmMain
!     Description: Optimized for speed and parallel tasking.
!     Author: fjn20007 
!    ***********************************************************

    ! CONST VALUES
    CONST robtarget calib_target_outofway :=[[-220.65,-330.94,687.12],[0.149229,0.120937,-0.0351868,-0.980748],[1,2,0,5],[110.617,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget calib_home_target:=[[447.68,-5.26,181.84],[0.0086866,-0.994821,0.101181,0.00433661],[1,1,1,5],[139.829,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget home_target_v3:=[[363.04,-198.14,250.65],[0.0417504,0.325274,0.761486,0.559099],[0,0,1,4],[177.611,9E+09,9E+09,9E+09,9E+09,9E+09]]; 
    CONST robtarget home_target := home_target_v3;
    
    ! used for mug manipulation
    CONST pos sholder_pos_close := [110,-200,460];
    CONST pos sholder_pos_far := [200,-400,460];
    CONST robtarget cup_target := [[499.548,-110.253,-46.3938],[0.0565402,0.114235,0.990146,-0.0580089],[-2,-3,-1,4],[-177.807,9E+09,9E+09,9E+09,9E+09,9E+09]]; 
        
    CONST speeddata movement_speed := v500; 
    CONST speeddata transit_speed := v1000; 
    CONST speeddata pick_speed := v800;
    CONST num max_magnitude := 300;         
    CONST num step_size := 150;             ! Optimized for faster calculation
        
    ! used in basic movement
    CONST num x_offset := 10;                
    CONST num y_offset := -10;               
    CONST num z_offset := 0;    
    
    ! used when fetching a mug
    CONST num gripper_offset := 0;    
    CONST num pick_offset := 100;
    CONST num offset_z_when_fetching := 100;

    PROC main()
        VAR mug_vector buffer;
        
        ! INITIALIZATION
        g_calibrate; 
        TPErase;     
        
        ! SPEED OPTIMIZATION: Set config once outside the loop
        ConfJ\On;   
        ConfL\On;
        
        moveToHomeTarget;

        WHILE TRUE DO
            ! Wait for signal from the Communication Task (processes.mod)
            WaitUntil shared_movement_right.wait_flag = TRUE;
            
            TEST shared_movement_right.flag 
            CASE flag_ERROR:
                TPWrite("ERROR OCCURED");
                StopMove;
                STOP;

            CASE flag_move: 
                ! Use optimized MovementProc logic
                MovementProc Offs(shared_movement_right.target,x_offset,y_offset,z_offset), step_size, max_magnitude, movement_speed;

            CASE flag_move_home: 
                MoveToHome; 
                
            CASE flag_gripper_grip: 
                g_gripIn;   
                
            CASE flag_gripper_release: 
                g_gripOut; 
                
            CASE flag_move_home_target: 
                ! Optimized moveToHomeTarget uses zones now
                moveToHomeTarget;
                
            CASE flag_move_EGM: 
                ! Smooth transit to EGM start point
                MoveJ shared_movement_right.target, movement_speed, z50, tGripper;
                EGMfollowCup;
                
            CASE flag_move_calibration:
                ! Blended move to calibration area
                MoveJ calib_home_target, transit_speed, z100, tGripper;
                
                IF shared_movement_right.target.trans.z < 60 THEN
                    shared_movement_right.target.trans.z := 60;
                ENDIF
                
                ! Keep fine for the actual calibration capture
                MoveJ shared_movement_right.target, movement_speed, fine, tGripper;
                              
            CASE flag_move_calibration_home:
                ! Optimized transit back
                MoveJ calib_home_target, transit_speed, z50, tGripper;
                
            CASE flag_move_calibration_outofway: 
                MoveJ calib_target_outofway, transit_speed, z100, tGripper;

            CASE flag_pick_up_mug: 
                ! Executes modernized FetchMug logic
                FetchMug shared_movement_right.mug.position, pick_offset, shared_movement_right.mug.normal;  
                
            CASE flag_leave_mug: 
                ! Standard dishwasher placement
                LeaveMugV2;
                
            CASE flag_hand_over: 
                ! IMPORTANT: handOverSequence now contains synchronized role logic
                handOverSequence;
                
            ENDTEST
            
            ! Update global status and signal task completion
            current_right_target := CRobT(\Tool:=tGripper); 
            shared_movement_right.wait_flag := FALSE;
        ENDWHILE
    ENDPROC
ENDMODULE