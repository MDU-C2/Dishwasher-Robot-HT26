MODULE MugManipulation

    ! fetch up mug
    PROC FetchMug(pos mug_position, num offset_lenght, pos mug_normal)
        VAR robtarget target;
        VAR orient hand_rotation;
        VAR pos offset_dir;

        hand_rotation := NormalToOrientationSemiOptimal(mug_position,mug_normal);
        offset_dir := RotatePointUsingQuaternion([0,0,1],hand_rotation);
        offset_dir.x := Round(offset_dir.x \Dec:=4);
        offset_dir.y := Round(offset_dir.y \Dec:=4);
        offset_dir.z := Round(offset_dir.z \Dec:=4);
        
        target := CRobT(\Tool := tGripper);
        ConfJ \Off;

        mug_position := mug_position + [0,0,1]*zOffset(mug_normal) + ([1,0,0]*x_offset + [0,1,0]*y_offset +[0,0,1]*z_offset);
        
        target.rot := hand_rotation;
        target.trans := mug_position - offset_dir*offset_lenght;

        MovementProc target,step_size,max_magnitude,movement_speed;
        
        g_GripOut;
        WaitTime(0.2);
        
        target.trans := mug_position + offset_dir*gripper_offset;
        moveL target,movement_speed,fine,tGripper;
        
        g_GripIn \HoldForce:=20;
        WaitTime(0.3); 
        
        target.trans := mug_position - offset_dir*offset_lenght + [0,0,1]*offset_z_when_fetching;
        moveL target,vmax,z100,tGripper;
    ENDPROC
    
  
    PROC handOverSequence()
        VAR robtarget target;
        VAR pos offset_dir;
        VAR pos target_pos;
        CONST num handover_z_offset := -30; 
       
        target := CRobT(\Tool := tGripper);
        target.rot := MugHandOverOrient();
        offset_dir := RotatePointUsingQuaternion([0,0,1], target.rot);
        
        ConfJ \Off;
        
        IF (RobName() = "ROB_R") THEN
            target_pos := shared_movement_right.hand_over_pose.position;
            target_pos.z := target_pos.z + handover_z_offset;

            target.trans := target_pos - 150*offset_dir;
            MovementProc target, step_size, max_magnitude, movement_speed;
            
            g_GripOut; 
            shared_movement_right.wait_flag := FALSE; 
            
            WaitUntil shared_movement_right.wait_flag = TRUE; 
            
            target.trans := target_pos;
            MoveL target, movement_speed, fine, tGripper;
            
            g_GripIn \HoldForce:=20;
            WaitTime 0.3; 
            shared_movement_right.wait_flag := FALSE; 

        ELSE
            target_pos := shared_movement_left.hand_over_pose.position;
            target.trans := target_pos;
            MovementProc target, step_size, max_magnitude, movement_speed;
            
            shared_movement_left.wait_flag := FALSE; 
            
            WaitUntil shared_movement_left.wait_flag = TRUE; 
            
            g_GripOut;
            WaitTime 0.1;
            
            target.trans := target.trans - offset_dir*150;
            MoveL target, movement_speed, z50, tGripper;
            
            shared_movement_left.wait_flag := FALSE; 
        ENDIF
    ENDPROC
        
    ! leave mug with dynamic coordinates
   PROC LeaveMug(pos mug_end_position, pos mug_end_normal, num offset_lenght)
        VAR robtarget target;
        VAR orient hand_rotation;
        VAR pos offset;
        
        hand_rotation := NormalToOrientationSemiOptimal(mug_end_position,mug_end_normal);
        offset := [0,0,1]*offset_lenght; 
        target := CRobT(\Tool := tGripper);
        ConfJ \Off;

        ! 1. Move to hover position
        target.rot := hand_rotation;
        target.trans := mug_end_position + offset;
        MovementProc target,step_size,max_magnitude,movement_speed;
        
        ! 2. ADDITIONAL ROTATION: Spin 90 degrees around Tool Z before dropping
        target.rot := target.rot * OrientZYX(90, 0, 0);
        MoveL target, movement_speed, fine, tGripper;
        WaitTime 0.1;
        
        ! 3. Lower to placement
        target.trans := mug_end_position;
        moveL target,movement_speed,fine,tGripper;
        
        g_GripOut;
        WaitTime 0.2;
        
        ! 4. Retreat
        target.trans := mug_end_position + offset;
        moveL target,movement_speed,z50,tGripper;
   ENDPROC
   
    ! leave mug with hardcoded basket position
    PROC LeaveMugV2()
        VAR robtarget end_target;
        ! Standard approach target
        end_target := [[513.42,-441.85,110.96],[0.353418,-0.368452,0.597902,-0.617941],[1,1,1,4],[-179.943,9E+09,9E+09,9E+09,9E+09,9E+09]];
        
        ConfJ \On;
        
        ! 1. Move to hover position above basket
        MoveJ end_target, movement_speed, z50, tGripper;
        
        ! 2. ADDITIONAL ROTATION: Re-orient the wrist 90 degrees
        ! This ensures the mug is facing the desired direction in the basket
        end_target.rot := end_target.rot * OrientZYX(90, 0, 0);
        MoveL end_target, movement_speed, fine, tGripper;
        
        ! 3. Lower into the basket slots
        MoveL Offs(end_target, 0, 0, -60), movement_speed, fine, tGripper;
        
        g_GripOut;
        WaitTime 0.2;
        
        ! 4. Retreat
        MoveL end_target, movement_speed, fine, tGripper;
   ENDPROC
    
ENDMODULE