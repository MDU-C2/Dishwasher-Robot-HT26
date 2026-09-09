MODULE movementFunctions
    !***********************************************************
    !
    ! Module:  movementFunctions
    !
    ! Description:
    !   These functions can be used for movement between two points and
    !   handle problems such as points far away from eachother and high joint values
    !
    ! Author: fjn20007 (Updated for runtime compilation fixes)
    !
    ! Version: 1.1
    !
    !***********************************************************

    PROC MovementProc(robtarget desired_target, num step_size, num orient_frac, num joint_threshold, speeddata movement_speed)
        VAR robtarget current_target;
        VAR robtarget disc_target;
        VAR jointtarget desired_joint_value;
        VAR bool home := FALSE;
        
        current_target := CRobT(\Tool:=tGripper);
        disc_target := current_target;
        
        desired_joint_value := CalcJointT(desired_target, tGripper);
        
        MoveJ desired_target, movement_speed, fine, tGripper;
        
    ERROR
        IF ERRNO = ERR_ROBLIMIT THEN

            IF home THEN
                STOP;
            ENDIF
            
            disc_target.trans := discretizePosition(current_target.trans, desired_target.trans, step_size);
            disc_target.rot := discretizeOrient(current_target.rot, desired_target.rot, orient_frac);
            
            WHILE NOT checkJointValues(current_target, disc_target, joint_threshold) DO
                step_size := step_size + 50;
                disc_target.trans := discretizePosition(current_target.trans, desired_target.trans, step_size);
                disc_target.rot := discretizeOrient(current_target.rot, desired_target.rot, orient_frac);
                
                ! Fixed: Using the Distance() function instead of trying to subtract positions directly
                IF step_size > Distance(desired_target.trans, current_target.trans) THEN
                    IF home THEN
                        RETRY;    
                    ENDIF
                    
                    ConfJ\On;
                    MoveJ home_target, movement_speed, fine, tGripper;
                    ConfJ\Off;
                    
                    current_target := CrobT(\Tool:=tGripper);
                    home := TRUE;
                    step_size := 50;
                ENDIF
            ENDWHILE
            
            MoveJ disc_target, movement_speed, z50, tGripper;
            current_target := CRobT(\Tool:=tGripper);

            RETRY;
        ENDIF
    ENDPROC
    
    FUNC pos discretizePosition(pos current_target, pos desired_target, num step_size)
        VAR pos dir_vector;
        VAR pos return_pos;
        VAR num mag;
        
        ! 1. Subtract X, Y, and Z individually
        dir_vector.x := desired_target.x - current_target.x;
        dir_vector.y := desired_target.y - current_target.y;
        dir_vector.z := desired_target.z - current_target.z;
        
        mag := VectMagn(dir_vector);
        
        ! 2. Divide X, Y, and Z individually
        IF mag > 0 THEN
            dir_vector.x := dir_vector.x / mag;
            dir_vector.y := dir_vector.y / mag;
            dir_vector.z := dir_vector.z / mag;
        ENDIF
        
        ! 3. Multiply and subtract X, Y, and Z individually
        return_pos.x := desired_target.x - (dir_vector.x * step_size);
        return_pos.y := desired_target.y - (dir_vector.y * step_size);
        return_pos.z := desired_target.z - (dir_vector.z * step_size);
        
        RETURN return_pos;
    ENDFUNC
    
    FUNC orient discretizeOrient(orient current_orient, orient desired_orient, num step_size)
        VAR orient return_orient;
        VAR num angle;
        VAR num sin_angle;
        VAR num coeff_1;
        VAR num coeff_2;
        VAR num dot_prod;
        
        dot_prod := QuaternionDotProd(current_orient, desired_orient);

        IF (dot_prod > 1) OR (dot_prod < -1) THEN
            RETURN current_orient;
        ENDIF
        
        angle := ACos(dot_prod);
        sin_angle := sin(angle);
        
        ! Added Safety Check: Prevents dividing by zero if orientations are already perfectly aligned
        IF sin_angle = 0 THEN
            RETURN desired_orient;
        ENDIF
        
        coeff_1 := sin((1 - step_size) * angle) / sin_angle;
        coeff_2 := sin(step_size * angle) / sin_angle;

        return_orient.q1 := coeff_1 * current_orient.q1 + coeff_2 * desired_orient.q1;
        return_orient.q2 := coeff_1 * current_orient.q2 + coeff_2 * desired_orient.q2;
        return_orient.q3 := coeff_1 * current_orient.q3 + coeff_2 * desired_orient.q3;
        return_orient.q4 := coeff_1 * current_orient.q4 + coeff_2 * desired_orient.q4;

        RETURN return_orient;
    ENDFUNC
    
    FUNC num QuaternionDotProd(orient q1, orient q2)
        VAR num dot_product;
        dot_product := q1.q1*q2.q1 + q1.q2*q2.q2 + q1.q3*q2.q3 + q1.q4*q2.q4;
        RETURN dot_product;
    ENDFUNC
    
    FUNC BOOL checkJointValues(robtarget target, robtarget desired_target, num threshold)
        VAR jointtarget desired_joint_target;
        
        desired_joint_target := CalcJointT(desired_target, tGripper);
        RETURN TRUE;
        
    ERROR
        IF ERRNO = ERR_ROBLIMIT THEN
            RETURN FALSE;
        ELSEIF ERRNO = ERR_OUTSIDE_REACH THEN
            RETURN FALSE;
        ENDIF
    ENDFUNC
ENDMODULE