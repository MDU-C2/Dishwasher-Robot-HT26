MODULE MugManipulation

    ! fetch up mug

PROC FetchMug(pos mug_position, num offset_lenght, pos mug_normal)
        VAR robtarget target;
        VAR robtarget side_lane;
        VAR orient hand_rotation;
        VAR pos offset_dir;

        ! 1. PRE-PICK: Move to side corridor to go around mugs
       ! IF (RobName() = "ROB_L") THEN
       !     MoveJ pSafeEntryLeft, v1000, z100, tGripper;
       ! ENDIF

        ! 2. Math Setup
        hand_rotation := NormalToOrientationSemiOptimal(mug_position, mug_normal);
        offset_dir := RotatePointUsingQuaternion([0,0,1], hand_rotation);
        
        target := CRobT(\Tool := tGripper);
        ConfJ \Off;

        ! Apply calibration offsets
        mug_position := mug_position + [0,0,1]*zOffset(mug_normal) + ([1,0,0]*x_offset + [0,1,0]*y_offset + [0,0,1]*z_offset);
        
        target.rot := hand_rotation;
        target.trans := mug_position;

        ! 3. HORIZONTAL SIDE APPROACH
        side_lane := target;
        
        ! FIX: Use Vector Addition [+] instead of Offs() because mug_position is 'pos'
        IF (RobName() = "ROB_L") THEN
            ! Offset 150mm to the Left (+Y)
            side_lane.trans := mug_position + [0, 20, 0]; 
        ELSE
            ! Offset 150mm to the Right (-Y)
            side_lane.trans := mug_position + [0, -20, 0]; 
        ENDIF

        ! Move to side lane, then slide in
       ! MovementProc side_lane, step_size, max_magnitude, v1000;
        MovementProc side_lane, 10, max_magnitude, v1000;
        g_GripOut;
        
        ! Move linearly from the side to the mug center
        MoveL target, movement_speed, fine, tGripper;
        
        g_GripIn \HoldForce:=20;
        WaitTime 0.5; 
        
        ! Retreat back to the side lane corridor
        MoveL side_lane, v800, z50, tGripper;
    ENDPROC
    
  
    PROC handOverSequence()
        VAR robtarget target;
        VAR pos offset_dir;
        VAR pos target_pos;
        VAR num spin_count := 0;
        CONST num handover_z_offset := -40; 
       
        target := CRobT(\Tool := tGripper);
        ConfJ \Off;

        IF (RobName() = "ROB_R") THEN
            ! RECEIVER (RIGHT): ADAPTIVE - NO FLIP
            WaitUntil shared_movement_left.wait_flag = FALSE; 
            target.rot := shared_movement_left.target.rot; ! COPY Left Arm's rotation
            
            target_pos := shared_movement_right.hand_over_pose.position;
            target_pos.z := target_pos.z + handover_z_offset;
            offset_dir := RotatePointUsingQuaternion([0,0,1], target.rot);

            target.trans := target_pos - 150*offset_dir;
            MovementProc target, step_size, max_magnitude, movement_speed;
            
            g_GripOut; 
            shared_movement_right.wait_flag := FALSE; 
            WaitUntil shared_movement_right.wait_flag = TRUE; 
            
            target.trans := target_pos;
            MoveL target, movement_speed, fine, tGripper;
            g_GripIn \HoldForce:=20;
            WaitTime 0.5; 
            shared_movement_right.wait_flag := FALSE; 

        ELSE
            ! GIVER (LEFT): Keep pick orientation, spin only if unreachable
            target_pos := shared_movement_left.hand_over_pose.position;
            target.trans := target_pos;

            WHILE NOT checkJointValues(target) DO
                target.rot := target.rot * OrientZYX(45, 0, 0); ! Spin around mug
                spin_count := spin_count + 1;
                IF spin_count > 8 THEN target.trans.x := target.trans.x + 50; spin_count := 0; ENDIF
            ENDWHILE
            
            shared_movement_left.target := target;
            MovementProc target, step_size, max_magnitude, movement_speed;
            MoveL target, movement_speed, fine, tGripper;
            
            shared_movement_left.wait_flag := FALSE; 
            WaitUntil shared_movement_left.wait_flag = TRUE; 
            g_GripOut;
            WaitTime 0.2;
            
            offset_dir := RotatePointUsingQuaternion([0,0,1], target.rot);
            target.trans := target.trans - offset_dir*150;
            MoveL target, vMax, z50, tGripper;
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