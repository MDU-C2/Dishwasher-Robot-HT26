MODULE MugManipulation

    ! fetch up mug

PROC FetchMug(pos mug_position, num offset_lenght, pos mug_normal)
        VAR robtarget target;
        VAR robtarget approach_point;
        VAR orient hand_rotation;
        VAR pos offset_dir;

        ! 1. Math: hand_rotation will now be 'Sideways' if X > 350, otherwise 'Normal'
        hand_rotation := NormalToOrientationSemiOptimal(mug_position, mug_normal);
        offset_dir := RotatePointUsingQuaternion([0,0,1], hand_rotation);
        
        target := CRobT(\Tool := tGripper);
        target.rot := hand_rotation;
        
        ! Apply calibration offsets
        mug_position := mug_position + [0,0,1]*zOffset(mug_normal) + ([1,0,0]*x_offset + [0,1,0]*y_offset + [0,0,1]*z_offset);
        target.trans := mug_position;
        approach_point := target;

        ! 2. Path Logic: Match the approach style to the area
        IF (RobName() = "ROB_L" AND mug_position.x < 510 AND mug_position.y < 200) THEN
            ! SIDE APPROACH PATH: Moves to the left corridor
                    TPWrite "Left side approach";
            approach_point.trans := mug_position + [0, 300, 0]; 
        ELSE
            ! NORMAL PATH: Simply pulls back 100mm along the gripper axis
            approach_point.trans := mug_position - (offset_dir *50);
                                TPWrite "normal approach";

        ENDIF

        ! 3. EXECUTION
        MovementProc approach_point, step_size, max_magnitude, v500;
        g_GripOut;
        
        MoveL target, movement_speed, fine, tGripper;
        g_GripIn \HoldForce:=20;
        WaitTime 0.6; 
        
        MoveL approach_point, v800, z50, tGripper;
    ENDPROC
    
  
PROC handOverSequence()
        VAR robtarget target;
        VAR pos offset_dir;
        VAR pos target_pos;
        CONST num handover_z_offset := -40; 
       
        ! 1. Get current position/orientation
        target := CRobT(\Tool := tGripper);
        
        ConfJ \Off;

        IF (RobName() = "ROB_R") THEN
            ! ===========================================================
            ! RECEIVER ROLE (RIGHT ARM)
            ! ===========================================================
            ! Force standard handover rotation
            target.rot := MugHandOverOrient();
            offset_dir := RotatePointUsingQuaternion([0,0,1], target.rot);
            
            target_pos := shared_movement_right.hand_over_pose.position;
            target_pos.z := target_pos.z + handover_z_offset;

            ! Move to waiting position (150mm away)
            target.trans := target_pos - 150 * offset_dir;
            MovementProc target, step_size, max_magnitude, movement_speed;
            
            g_GripOut; 
            shared_movement_right.wait_flag := FALSE; 
            WaitUntil shared_movement_right.wait_flag = TRUE; 
            
            ! Move in for the catch
            target.trans := target_pos;
            MoveL target, movement_speed, fine, tGripper;
            
            g_GripIn \HoldForce:=20;
            WaitTime 0.5; 
            shared_movement_right.wait_flag := FALSE; 

        ELSE
            ! ===========================================================
            ! GIVER ROLE (LEFT ARM) - THIS IS WHERE THE FLIP HAPPENS
            ! ===========================================================
            ! 2. MANDATORY: Overwrite the pickup rotation with Handover rotation
            ! This ensures the 'target' passed to MovementProc has the NEW angle.
            target.rot := MugHandOverOrient();
            
            ! 3. Get the meeting point coordinates
            target_pos := shared_movement_left.hand_over_pose.position;
            target.trans := target_pos;

            ! 4. MOVE TO MEETING POINT
            ! Because 'target.rot' is now different from the current robot state,
            ! MovementProc will physically rotate (flip) the wrist while moving to center.
            MovementProc target, step_size, max_magnitude, movement_speed;
            
            ! Ensure we stop exactly at the handover point
            MoveL target, movement_speed, fine, tGripper;
            
            shared_movement_left.wait_flag := FALSE; ! Signal: "I am ready and flipped"
            
            WaitUntil shared_movement_left.wait_flag = TRUE; 
            
            g_GripOut;
            WaitTime 0.2;
            
            ! Retreat
            offset_dir := RotatePointUsingQuaternion([0,0,1], target.rot);
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
        MovementProc target,step_size,max_magnitude,v1500; ! movement speed
        
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