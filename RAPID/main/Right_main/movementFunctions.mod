MODULE movementFunctions
    PROC MovementProc(robtarget desired_target, num step_size, num max_magnitude, speeddata movement_speed)
        VAR robtarget current_target;
        VAR robtarget disc_target;
        
        ConfJ\Off;
        ConfL\Off;

        ! SPEED OPTIMIZATION: Try direct move first
        IF checkJointValues(desired_target) THEN
            MoveJ desired_target, movement_speed, z50, tGripper;
            RETURN;
        ENDIF

        ! FALLBACK: Discretize if direct move hits joint limits
        current_target := CRobT(\Tool:=tGripper);
        WHILE VectMagn(desired_target.trans-current_target.trans) > max_magnitude DO
            disc_target := discretizeTarget(current_target,desired_target,step_size);
            MoveJ disc_target,movement_speed,z50,tGripper;
            current_target := CRobT(\Tool:=tGripper);
        ENDWHILE

        MoveJ desired_target,movement_speed,fine,tGripper;
        
    ERROR
        IF ERRNO = ERR_ROBLIMIT OR ERRNO = ERR_OUTSIDE_REACH THEN
            moveToHomeTarget;
            RETRY;
        ENDIF
    ENDPROC
    
!    ***********************************************************
!     Function: discretizeTarget

!     Description:  Returns a pos/orient between current and desired target, checking valid pos/orient iterating between the targets by step_size.
!                   Moves to predefined HomeTarget if no valid pos/orient.
    
!    ***********************************************************
    FUNC robtarget discretizeTarget(robtarget current_target, robtarget desired_target,num step_size)
        
        VAR robtarget disc_target;
        VAR jointtarget disc_joints;
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
                moveToHomeTarget;
                RETURN home_target;
            ENDIF
            
            disc_target.trans := discretizePosition(current_target.trans,desired_target.trans,step_size);
            disc_target.rot := discretizeOrient(current_target.rot,desired_target.rot,orient_frac);
            
        ENDWHILE
        
        RETURN disc_target;
        
    ENDFUNC
    
!    ***********************************************************
!     Function: discretizePosition

!     Description:  Returns the point located the given step_size away from current position towards desired position.
    
!    ***********************************************************
    FUNC pos discretizePosition(pos current_target, pos desired_target, num step_size)
    
        VAR pos dir_vector;
        VAR pos return_pos;
        VAR num magnitude;  ! Create a variable to hold the number
        
        ! 1. Calculate the direction vector individually
        dir_vector.x := desired_target.x - current_target.x;
        dir_vector.y := desired_target.y - current_target.y;
        dir_vector.z := desired_target.z - current_target.z;
        
        ! 2. Get the magnitude
        magnitude := VectMagn(dir_vector);
        
        ! 3. Divide by the magnitude individually
        dir_vector.x := dir_vector.x / magnitude;
        dir_vector.y := dir_vector.y / magnitude;
        dir_vector.z := dir_vector.z / magnitude;
        
        ! 4. Multiply by step size and add individually
        return_pos.x := current_target.x + (dir_vector.x * step_size);
        return_pos.y := current_target.y + (dir_vector.y * step_size);
        return_pos.z := current_target.z + (dir_vector.z * step_size);
        
        RETURN return_pos;
    ENDFUNC
    
!    ***********************************************************
!     Function: discretizePosition

!     Description:  Returns the orientation between current and desired orientation dependant on step_size.
    
!    ***********************************************************
    FUNC orient discretizeOrient(orient current_orient,orient desired_orient,num step_size)
    
        VAR orient dir_vector;
        VAR orient return_orient;
        VAR num angle;
        VAR num sin_angle;
        VAR num coeff_1;
        VAR num coeff_2;
        VAR num dot_prod;
        
        dot_prod := QuaternionDotProd(current_orient,desired_orient);
    
        IF (dot_prod > 1) OR (dot_prod < -1) THEN
            RETURN current_orient;
        ENDIF
        
        angle := ACos(dot_prod);
        sin_angle := sin(angle);
        coeff_1 := sin((1-step_size)*angle) / sin_angle;
        coeff_2 := sin(step_size*angle) / sin_angle;
    
        return_orient.q1 := coeff_1 * current_orient.q1 + coeff_2 * desired_orient.q1;
        return_orient.q2 := coeff_1 * current_orient.q2 + coeff_2 * desired_orient.q2;
        return_orient.q3 := coeff_1 * current_orient.q3 + coeff_2 * desired_orient.q3;
        return_orient.q4 := coeff_1 * current_orient.q4 + coeff_2 * desired_orient.q4;
    
        
        RETURN return_orient;
        ERROR
        IF ERRNO = ERR_DIVZERO THEN
            sin_angle := sin_angle + 0.000001;
            RETRY;
        ENDIF
    ENDFUNC
    
    !    ***********************************************************
!     Function: QuaternionDotProd

!     Description:  Returns the dot product two quaternions
    
!    ***********************************************************
    FUNC num QuaternionDotProd(orient q1,orient q2)
        VAR num dot_product;
        dot_product := q1.q1*q2.q1+q1.q2*q2.q2+q1.q3*q2.q3+q1.q4*q2.q4;
        RETURN dot_product;
    ENDFUNC

    
    !    ***********************************************************
!     Function: checkJointValues

!     Description:  Check wether calculated joint values from a target are valid
    
!    ***********************************************************
    FUNC BOOL checkJointValues(robtarget desired_target)
        
        VAR jointtarget desired_joint_target;
        
        desired_joint_target := CalcJointT(desired_target,tGripper);
        RETURN TRUE;
    ERROR
    IF ERRNO = ERR_ROBLIMIT THEN
        RETURN FALSE;
    ELSEIF ERRNO = ERR_OUTSIDE_REACH THEN
        RETURN FALSE;
    ENDIF
    ENDFUNC

    PROC moveToHomeTarget()
        ConfJ\On;
        ! SPEED OPTIMIZATION: Use z50 instead of fine so the robot doesn't "stop" at home
        MoveJ home_target_v3, v500, z50, tGripper;
        ConfJ\Off;
    ENDPROC
ENDMODULE