MODULE MugManipulation
    ! ===========================================================
    ! Module:  MugManipulation
    ! Description: Optimized for Speed and Handover Synchronization
    ! Role: Right Arm (Primary Receiver and Placer)
    ! ===========================================================

    ! fetch up mug
    PROC FetchMug(pos mug_position, num offset_lenght, pos mug_normal)
        VAR robtarget target;
        VAR orient hand_rotation;
        VAR pos offset_dir;

        ! 1. Calculate approach orientation
        hand_rotation := NormalToOrientationSemiOptimal(mug_position, mug_normal);
        
        ! 2. Calculate approach direction
        offset_dir := RotatePointUsingQuaternion([0,0,1], hand_rotation);
        
        target := CRobT(\Tool := tGripper);
        ConfJ \Off;

        ! 3. Apply pickup offsets
        mug_position := mug_position + [0,0,1]*zOffset(mug_normal) + ([1,0,0]*x_offset + [0,1,0]*y_offset + [0,0,1]*z_offset);
        
        target.rot := hand_rotation;
        ! 100mm Approach point
        target.trans := mug_position - offset_dir*100;

        ! SPEED FIX: MovementProc handles the move smoothly
        MovementProc target, step_size, max_magnitude, movement_speed;
        
        ! 4. Open gripper while final approaching
        g_GripOut;
        WaitTime 0.1;
                
        ! 5. Linear move to pick
        target.trans := mug_position + offset_dir*gripper_offset ;
        MoveL target, movement_speed, z10, tGripper;
        
        ! 6. Secure mug
        g_GripIn;
        WaitTime 0.4; ! Optimized physical motor time
        
        ! 7. Smooth retreat
        target.trans := mug_position - offset_dir*offset_lenght + [0,0,1]*offset_z_when_fetching;
        MoveL target, movement_speed, z50, tGripper;
    ENDPROC
    
    ! Logic for receiving the mug from the Left arm
    PROC handOverSequence()
        VAR robtarget target;
        VAR pos offset_dir;
        VAR pos target_pos;
       
        target := CRobT(\Tool := tGripper);
        target.rot := MugHandOverOrient();
        
        offset_dir := RotatePointUsingQuaternion([0,0,1], target.rot);
        target_pos := shared_movement_right.hand_over_pose.position;
    
        ConfJ \Off;

        ! ===========================================================
        ! RECEIVER ROLE (RIGHT ARM)
        ! ===========================================================
        
        ! 1. Parallel transit to meeting point offset (Waiting zone)
        ! We move here while the Left arm is still busy at the table
        target.trans := target_pos - 150*offset_dir;
        MovementProc target, step_size, max_magnitude, movement_speed;
        
        g_GripOut; ! Ensure open for the catch
        shared_movement_right.wait_flag := FALSE; ! Signal: "Right arm is waiting in the middle"
        
        ! 2. Wait for the Sequencer (processes.mod) to signal the catch
        WaitUntil shared_movement_right.wait_flag = TRUE; 
        
        ! 3. Move in for the handover
        target.trans := target_pos;
        MoveL target, movement_speed, fine, tGripper;
        
        ! 4. Physical Grip
        g_GripIn;
        WaitTime 0.4; ! CRITICAL: Robot must physically hold mug before Left opens
        
        ! 5. Confirm grip complete
        shared_movement_right.wait_flag := FALSE; 
        
        ! 6. Wait for Sequencer to confirm Left has released and moved away
        WaitUntil shared_movement_right.wait_flag = TRUE;
        
        TPWrite "Handover Complete - Right Arm has control";
    ENDPROC
        
    ! Leave mug in dishwasher/basket
   PROC LeaveMug(pos mug_end_position, pos mug_end_normal, num offset_lenght)
        VAR robtarget target;
        VAR orient hand_rotation;
        VAR pos offset;
        
        hand_rotation := NormalToOrientationSemiOptimal(mug_end_position, mug_end_normal);
        offset := [0,0,1]*offset_lenght + ([1,0,0]*x_offset + [0,1,0]*y_offset + [0,0,1]*z_offset);
        
        target := CRobT(\Tool := tGripper);
        ConfJ \Off;

        ! 1. Approach placement area
        target.rot := hand_rotation;
        target.trans := mug_end_position + offset;
        MovementProc target, step_size, max_magnitude, movement_speed;
                
        ! 2. Lower precisely
        target.trans := mug_end_position;
        MoveL target, movement_speed, fine, tGripper;
        
        ! 3. Release
        g_GripOut;
        WaitTime 0.2;
        
        ! 4. Fast vertical retreat
        target.trans := mug_end_position + (offset*1.5);
        MoveL target, movement_speed, z50, tGripper;
   ENDPROC
   
   ! Fixed destination placement
   PROC LeaveMugV2()
        VAR robtarget end_target;
        end_target := [[513.42,-441.85,110.96],[0.353418,-0.368452,0.597902,-0.617941],[1,1,1,4],[-179.943,9E+09,9E+09,9E+09,9E+09,9E+09]];
        
        ConfJ \On;
        MoveJ end_target, movement_speed, z50, tGripper;
        MoveL Offs(end_target,0,0,-60), movement_speed, fine, tGripper;
        
        g_GripOut;
        WaitTime 0.2;
        
        MoveL end_target, movement_speed, z50, tGripper;
   ENDPROC
ENDMODULE