MODULE MugManipulation
    ! ===========================================================
    ! Corrected for Type Mismatches and Task-Scope Errors
    ! ===========================================================

    PROC FetchMug(pos mug_position, num offset_lenght, pos mug_normal)
        VAR robtarget target;
        VAR orient hand_rotation;
        VAR pos offset_dir;

        hand_rotation := NormalToOrientationSemiOptimal(mug_position,mug_normal);
        offset_dir := RotatePointUsingQuaternion([0,0,1],hand_rotation);
        target := CRobT(\Tool := tGripper);
        
        mug_position := mug_position + [0,0,1]*zOffset(mug_normal) + ([1,0,0]*x_offset + [0,1,0]*y_offset +[0,0,1]*z_offset);
        target.rot := hand_rotation;
        target.trans := mug_position - offset_dir*100;

        MovementProc target, step_size, max_magnitude, movement_speed;
        g_GripOut;
        WaitTime 0.1;
                
        target.trans := mug_position + offset_dir*gripper_offset;
        MoveL target, movement_speed, z50, tGripper;
        
        g_GripIn;
        WaitTime 0.2;
        
        target.trans := mug_position - offset_dir*offset_lenght + [0,0,1]*offset_z_when_fetching;
        MoveL target, vmax, z100, tGripper;
    ENDPROC
    
    PROC handOverSequence()
        VAR robtarget target;
        VAR pos offset_dir;
        VAR pos target_pos;
       
        target := CRobT(\Tool := tGripper);
        target.rot := MugHandOverOrient();
        
        offset_dir := RotatePointUsingQuaternion([0,0,1], target.rot);
        offset_dir.x := Round(offset_dir.x \Dec:=4);
        offset_dir.y := Round(offset_dir.y \Dec:=4);
        offset_dir.z := Round(offset_dir.z \Dec:=4);
        
        ConfJ \Off;
        
        ! ===========================================================
        ! FIX FOR REFERENCE ERROR: 
        ! Use task-specific variables so the compiler stays happy
        ! ===========================================================
        IF (RobName() = "ROB_R") THEN
            ! This block only compiles correctly if shared_movement_right is in T_ROB_R
            target_pos := shared_movement_right.hand_over_pose.position;

            target.trans := target_pos - 150*offset_dir;
            MovementProc target, step_size, max_magnitude, movement_speed;
            
            g_GripOut; 
            shared_movement_right.wait_flag := FALSE; 
            
            WaitUntil shared_movement_right.wait_flag = TRUE; 
            
            target.trans := target_pos;
            MoveL target, movement_speed, fine, tGripper;
            
            g_GripIn;
            WaitTime 0.4; 
            shared_movement_right.wait_flag := FALSE; 

        ELSE
            ! This block only compiles correctly if shared_movement_left is in T_ROB_L
            target_pos := shared_movement_left.hand_over_pose.position;

            target.trans := target_pos;
            MovementProc target, step_size, max_magnitude, movement_speed;
            
            shared_movement_left.wait_flag := FALSE; 
            
            WaitUntil shared_movement_left.wait_flag = TRUE; 
            
            g_GripOut;
            WaitTime 0.2; 
            
            target.trans := target.trans - offset_dir*150;
            MoveL target, movement_speed, z50, tGripper;
            
            shared_movement_left.wait_flag := FALSE; 
        ENDIF
    ENDPROC
        
    PROC LeaveMug(pos mug_end_position, pos mug_end_normal, num offset_lenght)
        VAR robtarget target;
        VAR orient hand_rotation;
        VAR pos offset;
        
        hand_rotation := NormalToOrientationSemiOptimal(mug_end_position,mug_end_normal);
        offset := [0,0,1]*offset_lenght + ([1,0,0]*x_offset + [0,1,0]*y_offset + [0,0,1]*z_offset);
        target := CRobT(\Tool := tGripper);
       
        target.rot := hand_rotation;
        target.trans := mug_end_position + offset;
        MovementProc target, step_size, max_magnitude, movement_speed;
                
        target.trans := mug_end_position;
        MoveL target, movement_speed, fine, tGripper;
        
        g_GripOut;
        WaitTime 0.2;

        ! --- FIX FOR TYPE MISMATCH: Use 'target' (robtarget) instead of 'mug_end_position' (pos) ---
        MoveL Offs(target, 0, 0, 100), movement_speed, z50, tGripper;
    ENDPROC
   
    PROC LeaveMugV2()
        VAR robtarget end_target;
        end_target := [[513.42,-441.85,110.96],[0.353418,-0.368452,0.597902,-0.617941],[1,1,1,4],[-179.943,9E+09,9E+09,9E+09,9E+09,9E+09]];
        
        ConfJ \On;
        MoveJ end_target, movement_speed, fine, tGripper;
        MoveL Offs(end_target, 0, 0, -60), movement_speed, fine, tGripper;
        
        g_GripOut;
        WaitTime 0.2;
        
        MoveL end_target, movement_speed, fine, tGripper;
   ENDPROC
ENDMODULE