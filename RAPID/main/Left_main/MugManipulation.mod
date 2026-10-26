MODULE MugManipulation

    CONST robtarget rRightroll := [[477.90, 46.47, 226.92], [0.192592, 0.081852, -0.724543, 0.656693], [0,0,0,4],[-175,9E9,9E9,9E9,9E9,9E9]];  !funka.men för mycket

    ! fetch up mug
    PROC FetchMug(pos mug_position, num offset_lenght, pos mug_normal)
        VAR robtarget target;
        VAR orient hand_rotation;
        VAR robtarget approach_point;
        VAR pos offset_dir;
        
        !test 1
        VAR robtarget entry_point;
        VAR num open_mm;
        VAR num close_mm;
     
        

     !   IF mug_position.z < 55 THEN
            
            IF (mug_normal.z = 1) THEN
                mug_position.z := 55;
                TPWrite "upright fetch";
            ELSE
              !  mug_position.z := 35;
                mug_position.z := 45;
                TPWrite "downward fetch";

            ENDIF
     !   ENDIF   

        
        hand_rotation := NormalToOrientationSemiOptimal(mug_position,mug_normal);
        
     
        ! Apply calibration offsets
        offset_dir := RotatePointUsingQuaternion([0,0,1],hand_rotation);
        offset_dir.x := Round(offset_dir.x \Dec:=4);
        offset_dir.y := Round(offset_dir.y \Dec:=4);
        offset_dir.z := Round(offset_dir.z \Dec:=4);
        
        
        approach_point := target;
      
    ! Path Logic: Match the approach style to the area
        
        cycleGripOutDone := FALSE;
        ClkReset cycleClock;
        ClkStart cycleClock;
        g_GripOut;
        entry_point := CRobT(\Tool:=tGripper);  !test 1
        
        gripper_position := g_GetPos();
        TPWrite "gripper before" \Num:=gripper_position;
        ErrWrite \I, "gripper before: " + NumToStr(gripper_position, 3) + " s", "";
                
        IF (RobName() = "ROB_L" AND mug_position.x > xval) THEN ! AND mug_position.y > yval
                            
            TPWrite "normal approach";

         !   x_offset := 5;  
         !   y_offset := -22;
       
        
            IF (mug_normal.z = 1 OR mug_normal.z = -1) THEN
            
                MoveJ pSafeEntryNormalC, transit_speed,fine,tGripper;
                hand_rotation := [0.653281,0.653281,0.270598,0.270598]; 
                
                IF(RobName() = "ROB_L" AND mug_position.x > 560 AND mug_position.y > 170)THEN  ! längst bak (170 är vänser), kan inte nå dit med vinkel på hand
                    
                     hand_rotation := [0.44079,0.33107,0.61122,0.56789]; 
                     
                     IF(RobName() = "ROB_L" AND mug_position.x > 605 AND mug_position.x < 680) THEN  ! mug ligger på gräns 
                        TPWrite "gräns";
                        
                        mug_position.x := 605;
                     ENDIF
                ENDIF
            
                pSafeEntryNormalV.rot := hand_rotation;
                !pSafeEntryNormalV.trans := mug_position + [-30, 30, 0]; 
                pSafeEntryNormalV.trans := mug_position + [-30, 15, 0]; 
            
                entry_point:= pSafeEntryNormalV; !test 1
                MoveJ entry_point, transit_speed,z10,tGripper;
                
               ! MoveJ pSafeEntryNormalV, transit_speed,fine,tGripper;
            
           ELSE
               MoveJ pSafeEntryOvanC, transit_speed,fine,tGripper;
               TPWrite "From Above";
               pSafeEntryNormalV.rot := hand_rotation;
               pSafeEntryNormalV.trans := mug_position + [0, 0, 40]; 
               
               entry_point:= pSafeEntryNormalV; !test 1
               MoveL entry_point, transit_speed,fine,tGripper;
               
               !MoveL pSafeEntryNormalV, transit_speed,fine,tGripper;

           ENDIF
           
            
          
            
        ELSEIF(RobName() = "ROB_L" AND mug_position.x < xval AND mug_position.y < yval) THEN
       
        ! SIDE APPROACH PATH: Moves to the left corridor
              !if not upright or upside down
            
            IF (mug_normal.z = 1 OR mug_normal.z = -1) THEN 
                TPWrite "Left side approach";
                    
                       !  y_offset := -30;
                         
                hand_rotation := [0.7,0.71,0,-0.036];
                pSafeEntryLeft.trans  := mug_position + [0, 60, 0]; 
                
                entry_point:= pSafeEntryLeft; !test 1
                MoveJ entry_point, transit_speed,z50,tGripper;
                
             !   MoveJ pSafeEntryLeft, transit_speed,z50,tGripper;
                
            ELSE
                pSafeEntryOvanC2.trans  := mug_position + [0, 0, 300];
                TPWrite "Close From Above";
                entry_point:= pSafeEntryOvanC2; !test 1
                MoveJ  entry_point, transit_speed,fine,tGripper;
                
             !   MoveJ pSafeEntryOvanC2, transit_speed,fine,tGripper;
            
            ENDIF
        ENDIF
        

        MoveJ  entry_point, transit_speed,fine,tGripper; 
        target := CRobT(\Tool := tGripper);
        ConfJ \Off;

        TPWrite "mugs pos:" \Pos:=mug_position;
        mug_position := mug_position + [0,0,1]*zOffset(mug_normal) + ([1,0,0]*x_offset + [0,1,0]*y_offset +[0,0,1]*z_offset);
        TPWrite "mugs after offsets pos:" \Pos:=mug_position;
        
        target.rot := hand_rotation;
        target.trans := mug_position - offset_dir*10;
        TPWrite "mugs offset pos:" \Pos:=mug_position;
                
        TPWrite("At mug picking frame");
        
        !pick up mug
 !        target.trans := mug_position + offset_dir*gripper_offset;
        
        open_mm :=g_GetPos();  !test 1
        ErrWrite \I, "open_mm: " + NumToStr(open_mm, 3) + "mm", "";
        moveL target,v500,fine,tGripper;
        
        
         
        ! grippers in
        WaitTime(1);
        g_GripIn \HoldForce:=4.5;
        
        close_mm := g_GetPos();   !test 1
        ErrWrite \I, "close_mm: " + NumToStr(close_mm, 3) + "mm", "";
        
       ! gripper_position := g_GetPos();                                                   
       ! ErrWrite \I, "gripper after: " + NumToStr(gripper_position, 3) + " s", "";

    !  IF(open_mm > 1) AND (close_mm < open_mm /4) THEN    !test 1
    IF(close_mm = 0) THEN 
          TPWrite "Empty grasp, return to entry";
          g_GripOut;
          MoveJ entry_point, transit_speed, fine,tGripper;
          shared_movement_left.flag := flag_nothing;
          RETURN;
      ENDIF
        
       ! IF (gripper_position < 50) THEN         redan kommenterade bort innan test 1
         !   moveToHomeTarget;

!        WaitTime(1);
        moveL Offs(target,0,0,30),movement_speed,z50,tGripper;
        WaitTime(0.2);
        target.trans := mug_position - offset_dir*offset_lenght + [0,20,30]; ! *offset_z_when_fetching

        
        moveL target,v500,z50,tGripper; 
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
    !    gripper_position := grip_max;
        
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
            !WaitTime 0.1;
            
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
            
            
            !right roll first
            MoveJ rRightroll, vmax, z200, tGripper;
            
            ! Move to the meeting point while rotating (flipping) to standard orientation
            ! Using MovementProc here ensures the 'flip' happens safely across the segments
            MovementProc target, step_size, max_magnitude, movement_speed;
             !MoveJ target, movement_speed, fine, tGripper;

            
            ! Stop exactly at the meeting point
            MoveL target, movement_speed, fine, tGripper;
            
            shared_movement_left.wait_flag := FALSE; 
            
            ! Wait for Right arm to secure the grip
            WaitUntil shared_movement_left.wait_flag = TRUE; 
            WaitTime 0.2;
            g_GripOut;
            WaitTime 0.2;
            
            ! Retreat safely
            target.trans := target.trans - offset_dir*150;
            MoveL target, movement_speed, z50, tGripper;
            
            shared_movement_left.wait_flag := FALSE; 
            
            !end timer
            WaitUntil cycleGripOutDone;
            ClkStop cycleClock;
            TPWrite "Cycle time (s): "\Num:=ClkRead(cycleClock);
          !  ErrWrite \I, "Cycle time (s):", NumToStr(ClkRead(cycleClock), 2);
            ErrWrite \I, "Cycle time(s): " + NumToStr(ClkRead(cycleClock), 3) + " s", "";
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