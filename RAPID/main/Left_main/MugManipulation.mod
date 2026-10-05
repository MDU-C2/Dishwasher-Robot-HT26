MODULE MugManipulation

    ! fetch up mug

    PROC FetchMug(pos mug_position, num offset_lenght, pos mug_normal)
        VAR robtarget target;
        VAR orient hand_rotation;
        VAR robtarget approach_point;
        VAR pos offset_dir;

        hand_rotation := NormalToOrientationSemiOptimal(mug_position,mug_normal);
    !   hand_rotation := NormalToOrientation(mug_normal);
    !   hand_rotation := [0.67, 0.64, 0.29, 0.24];
    !   hand_rotation := [0.302, -0.271, 0.637, -0.655];
    !       hand_rotation := [0.73, 0.68, 0.030, -0.002];
     
     
        ! Apply calibration offsets
        offset_dir := RotatePointUsingQuaternion([0,0,1],hand_rotation);
        
        offset_dir.x := Round(offset_dir.x \Dec:=4);
        offset_dir.y := Round(offset_dir.y \Dec:=4);
        offset_dir.z := Round(offset_dir.z \Dec:=4);
        
        
         approach_point := target;
         mug_position.z := 55;
      
                ! 2. Path Logic: Match the approach style to the area
        IF (RobName() = "ROB_L" AND mug_position.x < xval AND mug_position.y < yval) THEN
        !IF (RobName() = "ROB_L" AND mug_position.x < 490 ) THEN
            ! SIDE APPROACH PATH: Moves to the left corridor
            TPWrite "Left side approach";
                  !   x_offset := 8;
                  !   y_offset := -20;
        
       !     approach_point.trans := mug_position + [0, 150, 0]; 
            pSafeEntryLeft.trans  := mug_position + [0, 80, 0]; 
            MoveJ pSafeEntryLeft, v1500,z150,tGripper;
            
        !    approach_point.rot :=  NormalToOrientationSemiOptimal(mug_position,mug_normal);
            
        !    MoveJ  approach_point , v1000, fine, tGripper; 
            
      !  ELSE
            ! NORMAL PATH: Simply pulls back 100mm along the gripper axis
        !    approach_point.trans := mug_position - (offset_dir *50);
         !                    TPWrite "normal approach";
            ! x_offset := 0;           
       !      y_offset := 0;                
             z_offset := 0; 

        ENDIF
        

         
        target := CRobT(\Tool := tGripper);
        ConfJ \Off;

        TPWrite "mugs pos:" \Pos:=mug_position;
        mug_position := mug_position + [0,0,1]*zOffset(mug_normal) + ([1,0,0]*x_offset + [0,1,0]*y_offset +[0,0,1]*z_offset);
        TPWrite "mugs after offsets pos:" \Pos:=mug_position;
        
        target.rot := hand_rotation;
        target.trans := mug_position - offset_dir*10;
        TPWrite "mugs offset pos:" \Pos:=mug_position;
      !  moveJ target,v1000,fine,tGripper;
      !  MovementProc target,5,max_magnitude,movement_speed;
        
        ! grippers out
        WaitTime(0.2);
        g_GripOut;
                
        TPWrite("At mug picking frame");
        
        !pick up mug
        target.trans := mug_position + offset_dir*gripper_offset;
        moveL target,v800,fine,tGripper;
!        MovementProc target,step_size,max_magnitude,movement_speed;
        
        ! grippers in
        WaitTime(1);
        
        g_GripIn;
        
!        WaitTime(1);
!        moveL Offs(target,0,0,30),movement_speed,z50,tGripper;
        WaitTime(0.2);
        target.trans := mug_position - offset_dir*offset_lenght + [0,0,1]; ! *offset_z_when_fetching
        moveJ target,v800,z50,tGripper;
!        MovementProc target,step_size,max_magnitude,movement_speed;
        
    ENDPROC
    
  
PROC handOverSequence()
        VAR robtarget target;
        VAR pos offset_dir;
        VAR pos target_pos;
        ! Stagger the height so the grippers are on different levels of the mug
        CONST num handover_z_offset := 23; 
        CONST num handover_y_offset := -8;
       
        target := CRobT(\Tool := tGripper);
        
        ! ===========================================================
        ! THE FIX: FORCE STANDARD ORIENTATION (THE FLIP)
        ! Instead of keeping the pick angle, we return to the angle 
        ! where we know the handover is safe and tested.
        ! ===========================================================
        target.rot := MugHandOverOrient();
        offset_dir := RotatePointUsingQuaternion([0,0,1], target.rot);
        
        ConfJ \Off;

        IF (RobName() = "ROB_R") THEN
            ! RECEIVER ROLE (RIGHT ARM)
            target_pos := shared_movement_right.hand_over_pose.position;
            ! Right arm grabs lower (staggered)
            target_pos.z := target_pos.z + handover_z_offset;
            target_pos.y := target_pos.y + handover_y_offset;

            ! Move to safe waiting distance (150mm away)
            target.trans := target_pos - 150 * offset_dir;
            MovementProc target, step_size, max_magnitude, movement_speed;
            
            g_GripOut; 
            shared_movement_right.wait_flag := FALSE; 
            
            ! Wait for Sequencer to say the Left arm is ready and holding the cup
            WaitUntil shared_movement_right.wait_flag = TRUE; 
            
            ! Linear move into the mug
            target.trans := target_pos;
            MoveL target, movement_speed, fine, tGripper;
            
            g_GripIn \HoldForce:=20;
            WaitTime 0.5; 
            shared_movement_right.wait_flag := FALSE; 

        ELSE
            ! GIVER ROLE (LEFT ARM)
            target_pos := shared_movement_left.hand_over_pose.position;
            target.trans := target_pos;
            
            ! Move to the meeting point while rotating (flipping) to standard orientation
            ! Using MovementProc here ensures the 'flip' happens safely across the segments
            MovementProc target, step_size, max_magnitude, movement_speed;
            
            ! Stop exactly at the meeting point
            MoveL target, movement_speed, fine, tGripper;
            
            shared_movement_left.wait_flag := FALSE; 
            
            ! Wait for Right arm to secure the grip
            WaitUntil shared_movement_left.wait_flag = TRUE; 
            
            g_GripOut;
            WaitTime 0.2;
            
            ! Retreat safely
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