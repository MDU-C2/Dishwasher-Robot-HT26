MODULE MugManipulation

    CONST robtarget rRightroll := [[477.90, 46.47, 226.92], [0.192592, 0.081852, -0.724543, 0.656693], [0,0,0,4],[-175,9E9,9E9,9E9,9E9,9E9]];  !funka.men f�r mycket
    CONST robtarget rRightroll2 := [[470.90, 103, 176], [0.038, 0.51, 0.85, -0.017], [0,0,0,4],[-175,9E9,9E9,9E9,9E9,9E9]];  !funka.men f�r mycket


    VAR num prev_failed_approach := 0;
    VAR pos prev_failed_entry_pos := [0, 0, 0];

    ! fetch up mug
PROC FetchMug(pos mug_position, num offset_lenght, pos mug_normal)
        VAR robtarget target;
        VAR orient hand_rotation;
        VAR pos offset_dir;
        VAR robtarget entry_point;
        VAR robtarget way_point;
        VAR robtarget settled_entry;
        VAR num open_mm;
        VAR num close_mm;
        VAR pos current_offset := [0, 0, 0];
        VAR num standoff_dist := 65;
        VAR bool is_upright := FALSE;
        VAR num current_approach := 0;

        is_upright := (Abs(mug_normal.z) > 0.5);

        ! Set base Z height: upright=65, upside_down=35, vertical/side=vert_base_z
        IF is_upright THEN
            IF Round(mug_normal.z \Dec:=1) = 1.0 THEN
                mug_position.z := mug_position.z ; !55
                TPWrite "upright fetch (Z=65)";
            ELSE
                mug_position.z :=  mug_position.z - 15; !35- x
                TPWrite "downward fetch (Z=35)";
                
                !safe controll
                IF mug_position.z < 32 THEN
                    mug_position.z := 32;
                ENDIF
            ENDIF
        ELSE
            mug_position.z := vert_base_z;
            TPWrite "vertical fetch Z=" \Num:=vert_base_z;
        ENDIF

        ! Clamp X and Y to prevent hitting the basket
        IF mug_position.y < -35 THEN
            mug_position.y := -35;
        ENDIF

        ! Default orientation from surface normal calculation
        hand_rotation := NormalToOrientationSemiOptimal(mug_position, mug_normal);

        ! Open gripper wide in advance
        cycleGripOutDone := FALSE;
        ClkReset cycleClock;
        ClkStart cycleClock;
        g_GripOut;
        gripper_position := g_GetPos();
        TPWrite "gripper before" \Num:=gripper_position;
        ErrWrite \I, "gripper before: " + NumToStr(gripper_position, 3) + " mm", "";

        ! Determine approach region first so switching approaches is done with safe retract
        target := CRobT(\Tool := tGripper);
        IF (prev_failed_approach <> 0 AND VectMagn(target.trans - prev_failed_entry_pos) > 20) THEN
            prev_failed_approach := 0;
        ENDIF

        IF (RobName() = "ROB_L" AND mug_position.x > xval AND mug_position.y <= 215) THEN
            current_approach := 1; !normal
        ELSEIF (RobName() = "ROB_L" AND mug_position.x <= xval AND mug_position.y <= 215) THEN
            current_approach := 2;
        ELSEIF (RobName() = "ROB_L" AND mug_position.y > 215) THEN
            current_approach := 3;
        ELSE
            current_approach := 4;
        ENDIF

        ! If retrying and switching between different approach methods, retract safely first
        IF (prev_failed_approach <> 0 AND prev_failed_approach <> current_approach) THEN
            TPWrite "Switching approach on retry - retracting to home first";
            IF target.trans.z < 200 THEN
                ConfL \Off;
                SingArea \Wrist;
                target.trans.z := 200;
                MoveL target, v1500, z50, tGripper;
                SingArea \Off;
            ENDIF
            moveToHomeTarget;
        ENDIF

        ! Select approach offsets and orientation based on mug workspace region
        IF current_approach = 1 THEN
            TPWrite "normal approach";

            IF is_upright THEN
                current_offset := offset_normal;
                hand_rotation := [0.653281, 0.653281, 0.270598, 0.270598];
                ! Safe transit waypoint in the corridor
                MoveJ pSafeEntryNormalC, transit_speed, fine, tGripper;
                
                 IF mug_position.x > 500 THEN
                     
                     way_point := target;
                     way_point.rot := hand_rotation;
                     way_point.trans := mug_position + [-100,100,50];
                 
                     MoveJ way_point, transit_speed, z10, tGripper;
                 ENDIF
    
            ELSE
                MoveJ pSafeEntryOvanC, transit_speed, z200, tGripper;
                TPWrite "From Above";
                hand_rotation := NormalToOrientationSemiOptimal(mug_position, mug_normal);
                MoveJ rRightroll2, vmax, z50, tGripper;

            ENDIF

        ELSEIF current_approach = 2 THEN
            TPWrite "Left side approach";

            IF is_upright THEN
                current_offset := offset_left;
                hand_rotation := [0.7, 0.71, 0, -0.036];
            ELSE
                TPWrite "Close From Above lower x-val";
                pSafeEntryOvanV2.trans.z := pSafeEntryOvanV2.trans.z-10; ! new vertical safe point for longer distance
                MoveJ pSafeEntryOvanV2, transit_speed,z200,tGripper;
                hand_rotation := NormalToOrientationSemiOptimal(mug_position, mug_normal);
            ENDIF

        ELSEIF current_approach = 3 THEN
            TPWrite"90 grad";
            current_offset := offset_center;
            
            IF is_upright THEN
                MoveJ pSafeEntryNormalC,transit_speed, z10, tGripper;
                hand_rotation := [0.5,0.5,0.5,0.5];
                
              !  pSafeEntryNormalV.trans := pSafeEntryNormalC.trans + [0,0,95];    only used if it for low 
              !  MoveJ pSafeEntryNormalV,transit_speed, z10, tGripper;

            ELSE
                TPWrite "From above, y>yval"; !xval
                MoveJ pSafeEntryOvanC, transit_speed,z200,tGripper;
                hand_rotation := NormalToOrientationSemiOptimal(mug_position,mug_normal);
            ENDIF

        ELSE
            TPWrite "Default approach";
            current_offset := offset_center;
            hand_rotation := NormalToOrientationSemiOptimal(mug_position, mug_normal);
        ENDIF

        ! Orientation-specific offsets for lying-down (vertical) mugs (replace region offset)
        IF NOT is_upright THEN
            IF Abs(mug_normal.x) >= Abs(mug_normal.y) THEN
                IF mug_normal.x >= 0 THEN
                    current_offset := offset_back;
                    TPWrite "Side offset: Back";
                ELSE
                    current_offset := offset_front;
                    TPWrite "Side offset: Front";
                ENDIF
            ELSE
                IF mug_normal.y < 0 THEN
                    current_offset := offset_left_side;
                    TPWrite "Side offset: left_side";
                ELSE
                    current_offset := offset_right_side;
                    TPWrite "Side offset: right_side";
                ENDIF
            ENDIF
            ! 35 mm Z clearance for 90 grad from-above picks
            IF current_approach = 3 THEN
                current_offset.z := current_offset.z + vert_z_extra_90grad;
            ENDIF
        ENDIF

        ! Keep legacy global offset variables updated
        x_offset := current_offset.x;
        y_offset := current_offset.y;
        z_offset := current_offset.z;

        ! Calculate unit approach direction collinear with the gripper tool Z-axis
        offset_dir := RotatePointUsingQuaternion([0, 0, 1], hand_rotation);
        IF is_upright THEN
            ! For upright mugs, force offset_dir.z = 0 to guarantee strictly constant Z horizontal glide
            offset_dir.z := 0;
        ENDIF
        IF VectMagn(offset_dir) > 0.0001 THEN
            offset_dir := offset_dir / VectMagn(offset_dir);
        ENDIF

        target := CRobT(\Tool := tGripper);
        ConfJ \Off;
        ConfL \Off;

        TPWrite "mugs raw pos:" \Pos:=mug_position;
        ! Apply calibration offset and normal-dependent height adjustment
        mug_position := mug_position + current_offset + [0, 0, 1] * zOffset(mug_normal);

        TPWrite "mugs after offsets pos:" \Pos:=mug_position;

        target.rot := hand_rotation;
        ! Position TCP slightly in front of mug center so jaws center cleanly
        IF is_upright THEN
            target.trans := mug_position - offset_dir * 10;
        ELSE
            target.trans := mug_position - offset_dir * vert_pullback;
        ENDIF
        TPWrite "target grasp pos:" \Pos:=target.trans;

        ! Compute collinear entry standoff point
        entry_point := target;
        entry_point.rot := hand_rotation;
        IF is_upright THEN
            ! Pure collinear standoff along gripper axis at constant Z
            entry_point.trans := target.trans - offset_dir * standoff_dist;
        ELSE
            ! Side mug: vertical standoff from above
            IF (mug_position.x > xval) THEN
                entry_point.trans := target.trans + [0, 0, vert_standoff_far];
            ELSE
                entry_point.trans := target.trans + [0, 0, vert_standoff_close];
            ENDIF
        ENDIF

        ! Update persistent robtargets for station/variable inspection
        IF (mug_position.x > xval) THEN
            pSafeEntryNormalV := entry_point;
        ELSE
            pSafeEntryLeft := entry_point;
        ENDIF

        ! Move to the collinear entry standoff point
        MoveJ entry_point, transit_speed, fine, tGripper;

        ! Refresh configuration after MoveJ (ConfJ \Off) so linear moves match settled posture
        settled_entry := CRobT(\Tool := tGripper);
        target.robconf := settled_entry.robconf;
        target.extax := settled_entry.extax;
        entry_point.robconf := settled_entry.robconf;
        entry_point.extax := settled_entry.extax;

        ! Smooth wrist interpolation when vertical side-mug lifts pass near Axis 5 = 0
        SingArea \Wrist;

        ! Smooth collinear linear glide directly into the mug along gripper axis at constant Z
        open_mm := g_GetPos();
        ErrWrite \I, "open_mm: " + NumToStr(open_mm, 3) + "mm", "";
        target.trans.z := target.trans.z + 10;
        MoveL target, v300, fine, tGripper; !150
        IF (mug_normal.z < -0.5) THEN
            target.trans.z := target.trans.z - 15;
            MoveL target, v1500, fine, tGripper; !150
        ENDIF

        ! Grip the mug
        WaitTime(0.5);
        IF is_upright THEN
            g_GripIn \HoldForce:=4;
        ELSE
            g_GripIn \HoldForce:=vert_hold_force;
        ENDIF

        !close_mm := g_GetPos();
        close_mm := 1;
        ErrWrite \I, "close_mm: " + NumToStr(close_mm, 3) + "mm", "";

        ! If empty grasp, open gripper and retract collinearly without bumping mug
        IF (close_mm = 0) THEN 
            TPWrite "Empty grasp, return collinearly to entry";
            g_GripOut;
            MoveL entry_point, v1500, z200, tGripper; !150 fine
            SingArea \Off;
            prev_failed_approach := current_approach;
            prev_failed_entry_pos := entry_point.trans;
            shared_movement_left.flag := flag_nothing;
            RETURN;
        ENDIF

        
        IF is_upright THEN
     ! Lift mug up slightly, then retract along tool approach axis
        MoveL Offs(target, 0, 0, 130), movement_speed, z50, tGripper;
        WaitTime(0.2);
        target.trans := mug_position - offset_dir * offset_lenght + [0, 20, 130];
        
        MoveL target, v1500, z50, tGripper; !500
        SingArea \Off;
        prev_failed_approach := 0;
        ENDIF
    ENDPROC

    
  
PROC handOverSequence(pos mug_normal)
        VAR robtarget target;
        VAR pos offset_dir;
        VAR pos target_pos;
        ! Stagger the height so the grippers are on different levels of the mug
        CONST num handover_z_offset := 23; 
        CONST num handover_y_offset := -8;
        VAR bool is_upside_down := FALSE;
       
        is_upside_down := (mug_normal.z < -0.5);
        target := CRobT(\Tool := tGripper);
        
        ! Force standard handover orientation (opposite Y direction to meet Right Arm)
        target.rot := MugHandOverOrient();
        offset_dir := RotatePointUsingQuaternion([0,0,1], target.rot);
        
        ConfJ \Off;

        IF (RobName() = "ROB_R") THEN
            ! RECEIVER ROLE (RIGHT ARM)
            target_pos := shared_movement_right.hand_over_pose.position;
            target_pos.z := target_pos.z + handover_z_offset;
            target_pos.y := target_pos.y + handover_y_offset;

            target.trans := target_pos - 100 * offset_dir;
            MovementProc target, step_size, max_magnitude, v800;
            g_GripOut; 
            MoveL target, v1500, fine, tGripper; !800
            shared_movement_right.wait_flag := FALSE; 
            
            WaitUntil shared_movement_right.wait_flag = TRUE;  
            
            target.trans := target_pos;
            MoveL target, v500, fine, tGripper; !300
            
            g_GripIn \HoldForce:=20;
            WaitTime 0.3; 
            shared_movement_right.wait_flag := FALSE; 

        ELSE
            ! GIVER ROLE (LEFT ARM)
            target_pos := shared_movement_left.hand_over_pose.position;
            target.trans := target_pos;
            
            ! Roll transition while lifting ONLY if NOT upside-down
            IF is_upside_down THEN
                TPWrite "ROB_L Handover: upside-down (skip wrist flip roll)";
            ELSE
                TPWrite "ROB_L Handover: upright (wrist flip roll)";
                MoveJ rRightroll, vmax, z200, tGripper;
            ENDIF
            
            ! Move to 15mm horizontal pre-dock at full Z=250 height first
            target.trans := target_pos + [0, 15, 0];
            MovementProc target, step_size, max_magnitude, movement_speed;
            MoveL target, movement_speed, z1, tGripper;
            
            ! Final 15mm horizontal docking move into meeting point
            target.trans := target_pos;
            MoveL target, movement_speed, fine, tGripper;
            
            ! Left arm is now stopped at meeting point; trigger Right arm's final 100mm approach
            shared_movement_left.wait_flag := FALSE; 
            
            ! Wait for Right arm to glide in and secure the grip
            WaitUntil shared_movement_left.wait_flag = TRUE; 
            g_GripOut;
            WaitTime 0.2;
            
            ! Retreat safely 150mm away from the mug
            target.trans := target.trans - offset_dir*150;
            MoveL target, movement_speed, fine, tGripper;
            
            ! Signal handover completion immediately so Right arm can proceed to tray
            shared_movement_left.wait_flag := FALSE;
            
            ! Move left arm completely back to home position in parallel with Right arm movement
            moveToHomeTarget;
            
            !end timer
            WaitUntil cycleGripOutDone;
            ClkStop cycleClock;
            TPWrite "Cycle time (s): "\Num:=ClkRead(cycleClock);
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