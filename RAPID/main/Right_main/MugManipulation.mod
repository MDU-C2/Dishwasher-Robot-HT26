MODULE MugManipulation

    ! ========== HANDOVER TUNING (TASK PERS - live-tunable via RWS / FlexPendant) ==========
    !   rotated   : upright, Back, Front, left_side, right_side
    !   unrotated : upside_down (mug_normal.z < -0.5)
    TASK PERS pos handover_offset_rotated   := [0, 0, 20];
    TASK PERS pos handover_offset_unrotated := [0, 0, -20];

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

        MovementProc target,step_size,max_magnitude,v1500; !movement speed
        
        g_GripOut;
        WaitTime(0.2);
        
        target.trans := mug_position + offset_dir*gripper_offset;
        moveL target,v1500,z50,tGripper; ! v1500 movement speed
        
     !   g_GripIn \HoldForce:=20;
        g_GripIn;
    !    WaitTime(0.3); 
        
        target.trans := mug_position - offset_dir*offset_lenght + [0,0,1]*offset_z_when_fetching;
        moveL target,vmax,z100,tGripper;
    ENDPROC
    
  
PROC handOverSequence(pos mug_normal)
        VAR robtarget target;
        VAR pos offset_dir;
        VAR pos target_pos;
        ! Stagger the height so the grippers are on different levels of the mug
        VAR pos active_handover_offset;
        VAR bool is_upside_down := FALSE;
        
        is_upside_down := (mug_normal.z < -0.5);
        target := CRobT(\Tool := tGripper);
        ConfJ \Off;

        IF (RobName() = "ROB_R") THEN
            ! RECEIVER ROLE (RIGHT ARM)
            ! Select handover offset and orientation from the orientation ROB_L picked the mug in
            IF is_upside_down THEN
                active_handover_offset := handover_offset_unrotated;
                target.rot := MugHandOverOrient() * OrientZYX(180, 0, 0);
                TPWrite "Handover offset: unrotated (upside_down)";
            ELSE
                active_handover_offset := handover_offset_rotated;
                target.rot := MugHandOverOrient();
                TPWrite "Handover offset: rotated";
            ENDIF

            offset_dir := RotatePointUsingQuaternion([0,0,1], target.rot);
            target_pos := shared_movement_right.hand_over_pose.position + active_handover_offset;

            ! Open gripper and move smoothly to safe 100mm approach standoff while Left lifts up
            g_GripOut; 
            target.trans := target_pos - 100 * offset_dir;
            MovementProc target, step_size, max_magnitude, v800;
            MoveL target, v800, fine, tGripper;
            
            shared_movement_right.wait_flag := FALSE; 
            
            ! Wait until Left arm has lifted the mug into the meeting point
            WaitUntil shared_movement_right.wait_flag = TRUE; 
            
            ! Linear horizontal approach into the mug right as Left is in position
            target.trans := target_pos;
            MoveL target, v300, fine, tGripper;
            
            g_GripIn \HoldForce:=20;
            WaitTime 0.3; 
            shared_movement_right.wait_flag := FALSE; 

        ELSE
            ! GIVER ROLE (LEFT ARM)
            target_pos := shared_movement_right.hand_over_pose.position;
            target.trans := target_pos;
            
            target.rot := MugHandOverOrient();
            offset_dir := RotatePointUsingQuaternion([0,0,1], target.rot);
            
            MovementProc target, step_size, max_magnitude, movement_speed;
            MoveL target, movement_speed, fine, tGripper;
            
            shared_movement_right.wait_flag := FALSE; 
            WaitUntil shared_movement_right.wait_flag = TRUE; 
            
            g_GripOut;
            WaitTime 0.2;
            
            target.trans := target.trans - offset_dir*150;
            MoveL target, movement_speed, z50, tGripper;
            
            shared_movement_right.wait_flag := FALSE; 
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
   
       ! New procedure to place the mug opening-up
    PROC LeaveMugUpright(pos mug_end_position, num offset_height)
        VAR robtarget target;
        VAR orient hand_rotation;
        ! Define the upward normal [X=0, Y=0, Z=1]
        VAR pos upward_normal := [0,0,-1];
        
        mug_end_position := [692.42, -424.85, 175.0];
        
        ! 1. Calculate orientation for the mug to be upright
        ! We use SemiOptimal to ensure the YuMi's elbow stays away from its body
        hand_rotation := NormalToOrientationSemiOptimal(mug_end_position, upward_normal);
        
        target := CRobT(\Tool := tGripper);
        ConfJ \Off;
        ConfL \Off;

        ! 2. Move to hover position above the destination
        target.rot := hand_rotation;
        target.trans := mug_end_position + [0, 0, offset_height];
        
        ! Use MovementProc to handle the transition and avoid joint flips
        MovementProc target, step_size, max_magnitude, v1500; ! leave mug at v1500 movement speed
        
        ! 3. Lower to the final placement position
        target.trans := mug_end_position;
        MoveL target, movement_speed, fine, tGripper;
        
        ! 4. Release the mug
        g_GripOut;
        cycleGripOutDone := TRUE;  !time
        WaitTime 0.2;
        
        ! 5. Retreat straight up
        MoveL Offs(target, 0, 0, offset_height), movement_speed, z50, tGripper;
    ENDPROC
    
        ! New dynamic version for placing the mug upright in a free slot
    PROC LeaveMugUprightV2(pos mug_end_position, num offset_height)
        VAR robtarget target;
        VAR orient hand_rotation;
        ! [0,0,-1] assumes you want the gripper pointing down (mug opening UP)
        VAR pos upward_normal := [0,0,-1];
        
        ! 1. Calculate orientation for this specific slot
        hand_rotation := NormalToOrientationSemiOptimal(mug_end_position, upward_normal);
        
        target := CRobT(\Tool := tGripper);
        ConfJ \Off;
        ConfL \Off;

        ! 2. Move to hover position above the free slot
        target.rot := hand_rotation;
        target.trans := mug_end_position + [0, 0, offset_height];
        
                WaitTime 0.1;
        
        ! Use MovementProc to handle the transition safely (prevents kinematics errors)
        MovementProc target, step_size, max_magnitude, v1500; 
        
        
        ! 3. Lower precisely into the slot
        target.trans := mug_end_position;
        MoveL target, v500, fine, tGripper;
        
        ! 4. Release
        g_GripOut;
        cycleGripOutDone := TRUE;  !time
        WaitTime 0.2;
        
        ! 5. Fast retreat vertically
        MoveL Offs(target, 0, 0, offset_height), v1500, z50, tGripper;
    ENDPROC  
    
PROC LeaveMugUprightV3(pos mug_end_position, num offset_height)
        VAR robtarget target;
        VAR orient hand_rotation;
        ! [0,0,-1] assumes you want the gripper pointing down (mug opening UP)
        VAR pos upward_normal := [0,0,-1];
        
        ! 1. Calculate orientation for this specific slot
        hand_rotation := NormalToOrientationSemiOptimal(mug_end_position, upward_normal);
        
        target := CRobT(\Tool := tGripper);
        ConfJ \Off;
        ConfL \Off;

        ! 2. Move directly to hover position above the free slot
        target.rot := hand_rotation;
        target.trans := mug_end_position + [0, 0, offset_height];
        
        WaitTime 0.1;
        
        ! Use MovementProc to handle the transition safely (prevents kinematics errors)
        MovementProc target, step_size, max_magnitude, v1500; 
        
        ! 4. Lower precisely into the slot
        target.trans := mug_end_position;
        MoveL target, v1500, fine, tGripper; !500
        
        ! 5. Release
        g_GripOut;
        cycleGripOutDone := TRUE;  !time
        WaitTime 0.2;
        
        ! 6. Fast retreat vertically
        MoveL Offs(target, 0, 0, offset_height), v1500, z50, tGripper;
    ENDPROC
ENDMODULE