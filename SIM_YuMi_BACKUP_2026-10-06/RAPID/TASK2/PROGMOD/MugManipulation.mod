MODULE MugManipulation

    ! ==== FetchMug tuning ====
    CONST num tray_z := 55;                  ! fixed tray height - vision Z is not trusted
    CONST num tray_z_upside_down := 20;      ! upside-down mugs sit lower in the rack
    CONST num hover_distance := 50;          ! mm back from grasp point along the approach axis
    CONST num entry_y_offset := 80;          ! entry waypoint Y offset (level with the mug)
    CONST speeddata approach_speed := v100;  ! final linear approach (accuracy over speed)
    CONST speeddata hover_speed := v1000;    ! transit to the hover point

    ! fetch up mug
    ! Approach axis convention: offset_dir = tool +Z and points AWAY from the mug,
    ! so  pos - offset_dir*d  = further back,  pos + offset_dir*d  = deeper onto the mug.

    PROC FetchMug(pos mug_position, num offset_lenght, pos mug_normal)
        VAR robtarget target;
        VAR orient hand_rotation;
        VAR pos offset_dir;
        VAR pos grasp_point;
        VAR pos normal_vec;

        ! ---- 1. Clean the vision normal ----
        normal_vec := mug_normal;
        IF VectMagn(normal_vec) < 0.5 THEN
            TPWrite "[WARN] FetchMug: bad mug normal, assuming upright";
            normal_vec := [0, 0, 1];
        ENDIF
        normal_vec := normal_vec / VectMagn(normal_vec);

        ! ---- 2. Grasp height ----
        IF normal_vec.z < -0.9 THEN
            mug_position.z := tray_z_upside_down;   ! upside down sits 35mm lower
        ELSE
            mug_position.z := tray_z;
        ENDIF

        ! ---- 3. Final grasp point: mug shape + calibration offsets ----
        ! x_offset / y_offset / z_offset live in LeftArmMain and are shared with
        ! flag_move - only read them here, never write (writing corrupts flag_move).
        grasp_point := mug_position + [0, 0, 1] * ZOffset(normal_vec) + [x_offset, y_offset, z_offset];
        TPWrite "FetchMug grasp point:" \Pos:=grasp_point;

        ! ---- 4. Hand orientation ----
        ! Upright AND upside-down are both fed [0,0,1]: that frame gives a horizontal
        ! (side) approach axis for both, and stops the wrist flipping 180 deg for an
        ! upside-down mug. A mug lying on its side keeps its own normal - unchanged,
        ! it still comes in from above.
        IF Abs(normal_vec.z) > 0.9 THEN
            hand_rotation := NormalToOrientationSemiOptimal(grasp_point, [0, 0, 1]);
            TPWrite "Upright / upside down - side grasp";
        ELSE
            hand_rotation := NormalToOrientationSemiOptimal(grasp_point, normal_vec);
            TPWrite "Lying mug - unchanged approach";
        ENDIF
        offset_dir := RotatePointUsingQuaternion([0, 0, 1], hand_rotation);
        offset_dir := offset_dir / VectMagn(offset_dir);

        ! ---- 5. Hover point, on the approach axis ----
        target := CRobT(\Tool := tGripper);
        target.rot := hand_rotation;
        target.trans := grasp_point - offset_dir * hover_distance;

        ! ---- 6. Fail before moving anything if the pose is not reachable ----
        IF NOT checkJointValues(target) THEN
            TPWrite "[ERROR] FetchMug: hover point unreachable, skipping pick";
            RETURN;
        ENDIF
        target.trans := grasp_point + offset_dir * gripper_offset;
        IF NOT checkJointValues(target) THEN
            TPWrite "[ERROR] FetchMug: grasp point unreachable, skipping pick";
            RETURN;
        ENDIF
        target.trans := grasp_point - offset_dir * hover_distance;

        ! ---- 7. Entry waypoint: level with the mug, same for EVERY pick ----
        ! The zone no longer picks the entry - it only picks the orientation
        ! (SemiOptimalPickUpOrientation): left zone faces -Y, otherwise shoulder direction.
        IF RobName() = "ROB_L" AND mug_position.x < xval AND mug_position.y < yval THEN
            TPWrite "Zone: left side orientation";
        ELSE
            TPWrite "Zone: normal orientation";
        ENDIF

        pSafeEntryLeft.trans := mug_position + [0, entry_y_offset, 0];
        IF checkJointValues(pSafeEntryLeft) THEN
            MoveJ pSafeEntryLeft, transit_speed, z150, tGripper;
        ELSE
            TPWrite "[WARN] FetchMug: entry unreachable, going straight to hover";
        ENDIF

        ! ---- 8. Open gripper before entering the tray ----
        !g_GripOut;

        ! ---- 9. Hover: stop exactly on axis so the linear move stays straight ----
        ConfJ \Off;
        MoveJ target, hover_speed, fine, tGripper;
        TPWrite "At mug picking frame";

        ! ---- 10. Straight, slow approach to the grasp point ----
        target.trans := grasp_point + offset_dir * gripper_offset;
        ConfL \On;      ! no reconfiguration halfway into the mug
        MoveL target, approach_speed, fine, tGripper;

        ! ---- 11. Grip, then let the force build before moving ----
        !g_GripIn \HoldForce:=20;
        WaitTime 0.3;

        ! GRIP CHECK HOOK - enable once the gripper feedback API is confirmed:
        !   read jaw position / object detection after closing;
        !   if no object: g_GripOut;
        !                 TPWrite "[WARN] FetchMug: missed mug";
        !                 RETURN;

        ! ---- 12. Pull straight back out along the same axis ----
        target.trans := grasp_point - offset_dir * offset_lenght;
        MoveL target, movement_speed, fine, tGripper;

        ConfJ \On;      ! restore the state main() expects
        ConfL \On;
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
            
            !g_GripOut; 
            shared_movement_right.wait_flag := FALSE; 
            
            ! Wait for Sequencer to say the Left arm is ready and holding the cup
            WaitUntil shared_movement_right.wait_flag = TRUE; 
            
            ! Linear move into the mug
            target.trans := target_pos;
            MoveL target, movement_speed, fine, tGripper;
            
            !g_GripIn \HoldForce:=20;
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
            
           ! g_GripOut;
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
        
        !g_GripOut;
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
        
        !g_GripOut;
        WaitTime 0.2;
        
        ! 4. Retreat
        MoveL end_target, movement_speed, fine, tGripper;
   ENDPROC
    
ENDMODULE