MODULE movementFunctions
!***********************************************************
!
! Module:  movementFunctions
!
! Description:  Optimized for speed. Attempts direct moves
!               before falling back to discretization logic.
!
!***********************************************************

    PROC MovementProc(robtarget desired_target, num step_size, num max_magnitude, speeddata movement_speed)
        VAR robtarget current_target;
        VAR robtarget disc_target;
        VAR bool home := FALSE;
        
        ConfJ\Off;
        ConfL\Off;

        ! ===========================================================
        ! SPEED OPTIMIZATION: Direct Move Check
        ! ===========================================================
        ! If the move is mathematically safe, skip the segments and go full speed.
        IF checkJointValues(desired_target) THEN
            MoveJ desired_target, movement_speed, z50, tGripper;
            RETURN;
        ENDIF

        ! ===========================================================
        ! FALLBACK: Discretization (If Direct Move is impossible)
        ! ===========================================================
        current_target := CRobT(\Tool:=tGripper);
        
        WHILE VectMagn(desired_target.trans-current_target.trans) > max_magnitude DO
            disc_target := discretizeTarget(current_target,desired_target,step_size);
            
            ! Maintain momentum with z50 instead of stopping at segments
            MoveJ disc_target,movement_speed,z50,tGripper;
            current_target := CRobT(\Tool:=tGripper);
        ENDWHILE

        ! Final reach to target
        MoveJ desired_target,movement_speed,fine,tGripper;
        
        ConfJ\On;
        ConfL\On;

    ERROR
        IF ERRNO = ERR_ROBLIMIT THEN
            ResetRetryCount;

            IF VectMagn(desired_target.trans-current_target.trans) > max_magnitude THEN
                disc_target := discretizeTarget(current_target,desired_target,step_size);
                MoveJ disc_target,movement_speed,z50,tGripper;
                current_target := CRobT(\Tool:=tGripper);

                IF disc_target = home_target_v3 THEN
                    home := TRUE;
                ENDIF
                RETRY;
                
            ELSEIF NOT home THEN
                moveToHomeTarget;
                current_target := CRobT(\Tool:=tGripper);
                home := TRUE;
                RETRY;
            ELSE
                moveToHomeTarget;
                TPWrite "Failed to reach target - ROB_R halted";
                RETURN;
            ENDIF
        ELSEIF ERRNO = ERR_OUTSIDE_REACH THEN
            TPWrite "Target outside physical reach of ROB_R";
            RETURN;
        ENDIF
    ENDPROC
    
    FUNC robtarget discretizeTarget(robtarget current_target, robtarget desired_target,num step_size)
        VAR robtarget disc_target;
        VAR num fixed_step;
        VAR num vector_magn;
        VAR num orient_frac;
        
        fixed_step := step_size;
        vector_magn := VectMagn(desired_target.trans-current_target.trans);
        orient_frac := step_size / vector_magn;
        
        disc_target := current_target;
        disc_target.trans := discretizePosition(current_target.trans,desired_target.trans,step_size);
        disc_target.rot := discretizeOrient(current_target.rot,desired_target.rot,orient_frac);
        
        WHILE NOT checkJointValues(disc_target) DO
            step_size := step_size + fixed_step;
            IF step_size > vector_magn THEN
                RETURN home_target_v3;
            ENDIF
            disc_target.trans := discretizePosition(current_target.trans,desired_target.trans,step_size);
            disc_target.rot := discretizeOrient(current_target.rot,desired_target.rot,orient_frac);
        ENDWHILE
        RETURN disc_target;
    ENDFUNC
    
    FUNC pos discretizePosition(pos current_target, pos desired_target, num step_size)
        VAR pos dir_vector;
        VAR num magnitude;
        
        dir_vector.x := desired_target.x - current_target.x;
        dir_vector.y := desired_target.y - current_target.y;
        dir_vector.z := desired_target.z - current_target.z;
        
        magnitude := VectMagn(dir_vector);
        
        dir_vector.x := dir_vector.x / magnitude;
        dir_vector.y := dir_vector.y / magnitude;
        dir_vector.z := dir_vector.z / magnitude;
        
        RETURN [current_target.x + (dir_vector.x * step_size), 
                current_target.y + (dir_vector.y * step_size), 
                current_target.z + (dir_vector.z * step_size)];
    ENDFUNC
    
    FUNC orient discretizeOrient(orient current_orient,orient desired_orient,num step_size)
        VAR num angle;
        VAR num sin_angle;
        VAR num dot_prod;
        
        dot_prod := (current_orient.q1*desired_orient.q1)+(current_orient.q2*desired_orient.q2)+(current_orient.q3*desired_orient.q3)+(current_orient.q4*desired_orient.q4);
    
        ! Rounding error safety check
        IF (Abs(dot_prod) > 0.9999) RETURN desired_orient;
        
        angle := ACos(dot_prod);
        sin_angle := sin(angle);
        
        RETURN [ (sin((1-step_size)*angle)/sin_angle)*current_orient.q1 + (sin(step_size*angle)/sin_angle)*desired_orient.q1,
                 (sin((1-step_size)*angle)/sin_angle)*current_orient.q2 + (sin(step_size*angle)/sin_angle)*desired_orient.q2,
                 (sin((1-step_size)*angle)/sin_angle)*current_orient.q3 + (sin(step_size*angle)/sin_angle)*desired_orient.q3,
                 (sin((1-step_size)*angle)/sin_angle)*current_orient.q4 + (sin(step_size*angle)/sin_angle)*desired_orient.q4 ];
    ENDFUNC

    FUNC num QuaternionDotProd(orient q1,orient q2)
        RETURN q1.q1*q2.q1+q1.q2*q2.q2+q1.q3*q2.q3+q1.q4*q2.q4;
    ENDFUNC

    FUNC BOOL checkJointValues(robtarget desired_target)
        VAR jointtarget desired_joint_target;
        desired_joint_target := CalcJointT(desired_target,tGripper);
        RETURN TRUE;
    ERROR
        RETURN FALSE;
    ENDFUNC
    
    PROC moveToHomeTarget()
        ConfJ\On;
        ! Fast move to home using z50
        MoveJ home_target_v3, v500, z50, tGripper;
        ConfJ\Off;
    ENDPROC
ENDMODULE