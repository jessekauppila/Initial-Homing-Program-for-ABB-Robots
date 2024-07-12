MODULE HomeLogicLists

    !definetool and workobject?

    !Left Home Sequence 

    CONST speeddata Medium:=v2000;

    CONST robtarget LeftHome:=[[-10.078042494,1824.668831942,802.431280345],[0.508337887,-0.501604166,0.508830774,0.480704791],[1,-2,1,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    CONST robtarget LeftHomeInt1:=[[-10.078042494,1824.668831942,802.431280345],[0.508337887,-0.501604166,0.508830774,0.480704791],[1,-2,1,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    CONST robtarget LeftHomeInt2:=[[-10.078042494,1824.668831942,802.431280345],[0.508337887,-0.501604166,0.508830774,0.480704791],[1,-2,1,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    CONST robtarget LeftHomeInt3:=[[-10.078042494,1824.668831942,802.431280345],[0.508337887,-0.501604166,0.508830774,0.480704791],[1,-2,1,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    CONST robtarget LeftHomeInt4:=[[-10.078042494,1824.668831942,802.431280345],[0.508337887,-0.501604166,0.508830774,0.480704791],[1,-2,1,0],[9E9,9E9,9E9,9E9,9E9,9E9]];

    CONST robtarget LeftLow:=[[689.414723237,1824.668910821,1159.28624599],[0.699324562,0.000122117,0.714800341,0.002369495],[0,-2,1,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    CONST robtarget LeftHigh:=[[689.415668582,1843.736962037,2261.41568216],[0.699324234,0.000122309,0.71480066,0.002369843],[0,-1,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    CONST robtarget LeftTop:=[[695.447397289,-80.906517867,3009.199597555],[0.789570357,-0.133382371,0.533031764,-0.273248848],[-1,0,-1,0],[9E9,9E9,9E9,9E9,9E9,9E9]];

    CONST robtarget Top:=[[695.447397289,-80.906517867,3009.199597555],[0.789570357,-0.133382371,0.533031764,-0.273248848],[-1,0,-1,0],[9E9,9E9,9E9,9E9,9E9,9E9]];

    CONST robtarget RightTop:=[[695.447397289,-80.906517867,3009.199597555],[0.789570357,-0.133382371,0.533031764,-0.273248848],[-1,0,-1,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    CONST robtarget RightHigh:=[[788.029576424,-2141.291550396,2089.025775678],[0.699324542,0.000122445,0.714800361,0.002369197],[-1,0,-1,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    CONST robtarget RightLow:=[[615.6264138,-1970.938238655,1091.520628553],[0.699324651,0.000122961,0.714800255,0.002369161],[-1,1,-2,0],[9E9,9E9,9E9,9E9,9E9,9E9]];


    CONST robtarget RightHomeInt4:=[[2.648006257,-1970.937085546,721.148890896],[0.506219517,0.495009538,0.515659034,-0.482496754],[-2,1,-2,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    CONST robtarget RightHomeInt3:=[[2.648006257,-1970.937085546,721.148890896],[0.506219517,0.495009538,0.515659034,-0.482496754],[-2,1,-2,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    CONST robtarget RightHomeInt2:=[[2.648006257,-1970.937085546,721.148890896],[0.506219517,0.495009538,0.515659034,-0.482496754],[-2,1,-2,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    CONST robtarget RightHomeInt1:=[[2.648006257,-1970.937085546,721.148890896],[0.506219517,0.495009538,0.515659034,-0.482496754],[-2,1,-2,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    CONST robtarget RightHome:=[[2.648006257,-1970.937085546,721.148890896],[0.506219517,0.495009538,0.515659034,-0.482496754],[-2,1,-2,0],[9E9,9E9,9E9,9E9,9E9,9E9]];

    !This is defined by the center of the working area, the nearest reach of the robot
    CONST robtarget RefTarget:=[[1134.287973351,-71.167734203,1622.521532533],[0.500079065,-0.473042279,0.528662994,-0.496656189],[-1,-1,-1,1],[9E9,9E9,9E9,9E9,9E9,9E9]];

    !This is for working with the waypointlist
    VAR robtarget targetWaypoint;
    VAR num i;

    VAR num waypointEnroute;

    !These are for within the PROC routeLeftHomeRightHome
    !VAR num beginningOfRoute;
    VAR num routeAssignedEnd;

    !These are chosen evertime in the whereTorouteAssignedEnd Proc
    VAR num routeAssignedBeginning;
    !VAR num route;
    !VAR num routeAssignedEnd:=1;


    !This is the Waypointlist...
    VAR robtarget waypoints{numWayPoints};
    CONST num numWayPoints:=17;




    Var robtarget EndTarget;


    !Get an offset from the Home robtarget to use to tell if already at home position or not
    VAR robtarget LeftHomeOffset;
    VAR robtarget RightHomeOffset;

    !These are the variables which need to be initiated to run the TPreadFK for left or right side choice
    VAR num Left_or_Right:=-1;
    VAR string Left_or_Right_Response:="";
    VAR num Left_or_Right_Answer:=0;

    !These are the variables which need to be initiated to run the TPreadFK for Home or Plate choice
    VAR num Home_or_Plate:=-1;
    VAR string Home_or_Plate_Response:="";
    VAR num Home_or_Plate_Answer:=0;

    !These are the VAR for getPositionInfo

    !instantiate variables to use in this proc...   
    !maybe break these down further...


    VAR NUM DistToolLocToHome;
    VAR NUM DistCenterOfWorkToHome;
    VAR NUM DistToolLocToPlateHome;
    VAR NUM DistCenterOfWorkToPlateHome;

    VAR num DistToolLocToEndTarget;
    VAR num DistRefTargetToEndTarget;

    !These "pos" are used to find where the 

    VAR pos RefPos;

    VAR pos LeftLowPos;
    VAR pos RightLowPos;

    VAR pos ToolPos;


    VAR pos HomePos;
    VAR pos HomeOffsetPos;
    VAR pos FarAwayHomeOffsetPos;

    VAR pos LeftHomePos;
    VAR pos RightHomePos;

    VAR pos LeftHomeOffsetPos;
    VAR pos RightHomeOffsetPos;

    VAR pos CenterOfWorkPos;

    VAR pos RefTargetPos;

    VAR pos EndTargetPos;

    !These are for setting their logical, boolean values in the logic tree at the end of this...
    VAR bool MoveLtoRorRtoL;
    VAR bool ToolPosLorR;
    VAR bool ToolPosLowOrHigh;
    VAR bool ToolPosFarFromPlateOrNear;

    VAR robtarget ToolRobtarget;

    !This is to try to move the robot out of the upper left or upper left areas away from the piece
    VAR robtarget UpperToolOffset;


    PROC Main()

        ! waypoints{0}:=LeftHome;
        waypoints{1}:=LeftHome;
        waypoints{2}:=LeftHomeInt1;
        waypoints{3}:=LeftHomeInt2;
        waypoints{4}:=LeftHomeInt3;
        waypoints{5}:=LeftHomeInt4;

        waypoints{6}:=LeftLow;
        waypoints{7}:=LeftHigh;
        waypoints{8}:=LeftTop;

        waypoints{9}:=Top;

        waypoints{10}:=RightTop;
        waypoints{11}:=RightHigh;
        waypoints{12}:=RightLow;

        waypoints{13}:=RightHomeInt4;
        waypoints{14}:=RightHomeInt3;
        waypoints{15}:=RightHomeInt2;
        waypoints{16}:=RightHomeInt1;
        waypoints{17}:=RightHome;
        
        TPReadFK Home_or_Plate,"Where do you want to move the tool?","Left Home","Left Plate","","Right Plate","Right Home";
        IF Home_or_Plate=1 THEN
            Home_or_Plate_Response:="the tool will go to Left Home";
            routeAssignedEnd:=1;
            writeEndOfRouteTarget;
            EndTarget:=waypoints{1};!Does this work?
            !writeEndOfRouteTarget;
        ELSEIF Home_or_Plate=2 THEN
            Home_or_Plate_Response:="the tool will go to Left Plate";
            routeAssignedEnd:=6;
            EndTarget:=waypoints{6};
            writeEndOfRouteTarget;  
        ELSEIF Home_or_Plate=3 THEN
            Home_or_Plate_Response:="";
        ELSEIF Home_or_Plate=4 THEN
            Home_or_Plate_Response:="the tool will go to Right Plate";
            routeAssignedEnd:=12;
            EndTarget:=waypoints{12};
            writeEndOfRouteTarget;
        ELSE
            Home_or_Plate_Response:="the tool will go to Right Home";
            routeAssignedEnd:=17;
            EndTarget:=waypoints{17};
            writeEndOfRouteTarget;
        ENDIF
        TPPrint "Ok, "+Home_or_Plate_Response+"!\0D\0A";
        
        

        !whereTorouteAssignedEnd;
        
        !sets routeAssignedEnd
        getPositionInfo;
        !calculates what side starting at and other values
        routeBeginningCalculator;
        !calculates where to begin rout...
        routeToEitherHomeLorR;
        !I took these parameters out to see if it might, just might run!routeAssignedBeginning,routeAssignedEnd;

    ENDPROC

    !These Below are all error finding devices...
    PROC writeEndOfRouteTarget()
        TPWrite "End of Route Target = "+ValToStr(routeAssignedEnd);
    EndProc

    PROC writeBeginCalcAssignedBegin()
        TPWrite " `routeAssignedBeginning' in 'routeBeginningCalculator' = "+ValToStr(routeAssignedBeginning);
    EndProc

    PROC writeCheckAssignedBegin()
        TPWrite "Check `routeAssignedBeginning' in `routeToEitherHomeLorR' = "+ValToStr(routeAssignedBeginning);
    EndProc

    PROC writeBeginningOfRoute()
        TPWrite "Check `beginningOfRoute' in `routeToEitherHomeLorR' = "+ValToStr(routeAssignedBeginning);
    EndProc

    PROC writeTargetWaypointEnroute()
        TPWrite "Next Target Location Enroute = "+ValToStr(waypointEnroute);
    ENDPROC

    PROC writeReachedWaypointEnroute()
        TPWrite "Reached Location Enroute = "+ValToStr(waypointEnroute);
    EndProc

    !These above are all error finding devices...!!!

    !PROC whereTorouteAssignedEnd()
        
        
        



    !ENDPROC

    PROC getPositionInfo()
        !Get Location of Tool... in the proverbial "grass"
        ToolRobtarget:=CRobT(\Tool:=tool0\WObj:=wobj0);
        ToolPos:=ToolRobtarget.trans;

        !Get an offset from the Home robtarget to use to tell if already at home position or not

        LeftHomePos:=LeftHome.trans;
        RightHomePos:=RightHome.trans;

        LeftHomeOffset:=Offs(LeftHome,250,0,0);
        RightHomeOffset:=Offs(RightHome,250,0,0);

        LeftHomeOffsetPos:=LeftHomeOffset.trans;
        RightHomeOffsetPos:=RightHomeOffset.trans;

        !LeftLowPos:=LeftLow.trans
        !RightLowPos:=RightLow.trans

        !Translate Robtargets to Pos positions...
        !ToolPos:=ToolLoc.trans;

        !What is this for?
        RefTargetPos:=RefTarget.trans;

        
        EndTargetPos:=EndTarget.trans;

        DistToolLocToEndTarget:=Distance(ToolPos,EndTargetPos);
        DistRefTargetToEndTarget:=Distance(RefTargetPos,EndTargetPos);
        
        
        
         !compare location of tool to destination of tool...
            !use routeAssignedEnd
            !compare this to 
            !ToolPos OR
            !
            ! If disToolLocPosToRouteAssignedEnd > distReftoRouteAssignedEnd
            !IF  DistToolLocToEndTarget > DistRefTargetToEndTarget THEN
                
        
    ENDPROC

    PROC routeToEitherHomeLorR()
        !was num waypointEnroute,num routeAssignedEnd
        !writeBeginningOfRouteTarget;
        !Left to Right!

        writeCheckAssignedBegin;
        writeBeginningOfRoute;

        !waypointEnroute := beginningOfRoute;
        !routeAssignedEnd

        waypointEnroute:=routeAssignedBeginning;
    !routeAssignedEnd;

        IF waypointEnroute<routeAssignedEnd THEN

            !This says if moving from left to right....
            WHILE waypointEnroute<=routeAssignedEnd DO
                targetWaypoint:=waypoints{waypointEnroute};
                writeTargetWaypointEnroute;
                MoveL targetWaypoint,Medium,z100,tool0\WObj:=wobj0;
                writeReachedWaypointEnroute;
                waypointEnroute:=waypointEnroute+1;
            ENDWHILE

            !Right to Left!
        ELSE
            WHILE waypointEnroute>routeAssignedEnd DO
                targetWaypoint:=waypoints{waypointEnroute};
                writeTargetWaypointEnroute;
                MoveL targetWaypoint,Medium,z100,tool0\WObj:=wobj0;
                writeReachedWaypointEnroute;
                waypointEnroute:= waypointEnroute-1;
            ENDWHILE

        ENDIF

    ENDPROC

    !This sets the beginning of the route...


    !This should be something like if

    !PROC setMoveLtoRorRtoL()
        !IF  DistToolLocToEndTarget > DistRefTargetToEndTarget THEN
        
        !IF routeAssignedBeginning<routeAssignedEnd THEN
       ! THEN
            !MoveLtoRorRtoL:=TRUE;
            !HomeOffsetPos:=LeftHomeOffsetPos;
        !ELSE
         !!   MoveLtoRorRtoL:=FALSE;
            !HomeOffsetPos:=RightHomeOffsetPos;
      !  ENDIF
   ! ENDPROC

    PROC setToolPosLorR()
        IF DistToolLocToEndTarget<DistRefTargetToEndTarget THEN
            ToolPosLorR:=TRUE;
        ELSE
            ToolPosLorR:=FALSE;
        ENDIF
    ENDPROC

    PROC setToolPosLowOrHigh()
        IF ToolPos.z<RefPos.z THEN
            ToolPosLowOrHigh:=TRUE;
        ELSE
            ToolPosLowOrHigh:=FALSE;
        ENDIF
    ENDPROC

    PROC setToolPosFarFromPlateOrNear()
        IF ToolPos.x<HomeOffsetPos.x THEN
            ToolPosFarFromPlateOrNear:=TRUE;
        ELSE
            ToolPosFarFromPlateOrNear:=FALSE;
        ENDIF
    ENDPROC


    PROC routeBeginningCalculator()
        !setMoveLtoRorRtoL;
        setToolPosLorR;
        setToolPosLoworHigh;
        setToolPosFarFromPlateOrNear;

        !IF MoveLtoRorRtoL THEN ! This is creating some sort of recursion because I am using the beginning value to calculate if I'm moveing from left ro write!
       
        IF ToolPosLorR THEN
            IF ToolPosLowOrHigh THEN
                IF ToolPosFarFromPlateOrNear THEN
                    !IF routeAssignedBeginning 
                    routeAssignedBeginning:=1;
                    writeBeginCalcAssignedBegin;
                ELSE
                    routeAssignedBeginning:=6;
                    writeBeginCalcAssignedBegin;
                ENDIF
            ELSE
                IF ToolPosFarFromPlateOrNear THEN
                    UpperToolOffset:=Offs(ToolRobtarget,10,10,0);
                    MoveL ToolRobtarget,Medium,z100,tool0\WObj:=wobj0;
                    !routeBeginningCalculator;
                ELSE
                    routeAssignedBeginning:=7;
                    writeBeginCalcAssignedBegin;
                ENDIF
            ENDIF
        ELSE
            IF ToolPosLowOrHigh THEN
                IF ToolPosFarFromPlateOrNear THEN
                    UpperToolOffset:=Offs(ToolRobtarget,10,-10,0);
                    MoveL ToolRobtarget,Medium,z100,tool0\WObj:=wobj0;
                    !routeBeginningCalculator;
                ELSE
                    routeAssignedBeginning:=11;
                    writeBeginCalcAssignedBegin;
                ENDIF
            ELSE
                IF ToolPosFarFromPlateOrNear THEN
                    routeAssignedBeginning:=12;
                    writeBeginCalcAssignedBegin;
                ELSE
                    routeAssignedBeginning:=17;
                    writeBeginCalcAssignedBegin;
                ENDIF
            ENDIF
        ENDIF
        !ELSE
        !IF ToolPosLorR THEN
        !   IF ToolPosLowOrHigh THEN
        !      IF ToolPosFarFromPlateOrNear THEN
        !         routeAssignedBeginning:=17;
        !        writeBeginCalcAssignedBegin;
        !   ELSE
        !      routeAssignedBeginning:=12;
        !     writeBeginCalcAssignedBegin;
        ! ENDIF
        !ELSE
        !   IF ToolPosFarFromPlateOrNear THEN
        !      routeAssignedBeginning:=11;
        !     writeBeginCalcAssignedBegin;
        !ELSE
        !   UpperToolOffset:=Offs(ToolRobtarget,10,-10,0);
        !  MoveL ToolRobtarget,Medium,z100,tool0\WObj:=wobj0;
        !routeBeginningCalculator;
        !ENDIF
        !   ENDIF
        !ELSE
        !   IF ToolPosLowOrHigh THEN
        !      IF ToolPosFarFromPlateOrNear THEN
        !         routeAssignedBeginning:=7;
        !        writeBeginCalcAssignedBegin;
        !   ELSE
        !      UpperToolOffset:=Offs(ToolRobtarget,10,10,0);
        !MoveL ToolRobtarget,Medium,z100,tool0\WObj:=wobj0;
        !routeBeginningCalculator;
        ! ENDIF
        !ELSE
        !   IF ToolPosFarFromPlateOrNear THEN
        !      routeAssignedBeginning:=6;
        !     writeBeginCalcAssignedBegin;
        !ELSE
        !   routeAssignedBeginning:=1;
        !  writeBeginCalcAssignedBegin;
        !ENDIF
        !ENDIF
        !ENDIF
        !ENDIF

    ENDPROC

ENDMODULE