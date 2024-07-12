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
    !These are for within the PROC routeLeftHomeRightHome
    VAR num beginningRoute;
    VAR num endRoute;

    CONST num numWayPoints:=17;
    !This is the Waypointlist...
    VAR robtarget waypoints{numWayPoints};

    !These are chosen evertime in the whereToPlace Proc
    VAR num routeAssignedBeginning;
    VAR num routeAssignedEnd;

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
    VAR bool ToolPosNearPlateorFar;

    VAR robtarget ToolRobtarget;

    !This is to try to move the robot out of the upper left or upper left areas away from the piece
    VAR robtarget UpperToolOffset;


    PROC Main()

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

        whereToPlace;
        
        getPositionInfo;
        
        routeBeginningCalculator;

        routeToEitherHomeLorR routeAssignedBeginning,routeAssignedEnd;

    ENDPROC

    PROC whereToPlace()

        TPReadFK Home_or_Plate,"Where do you want to move the tool?","Left Home","Left Plate","","Right Plate","Right Home";
        IF Home_or_Plate=1 THEN
            Home_or_Plate_Response:="the tool will go to Left Home.";
            Home_or_Plate_Answer:=1;
        ELSEIF Home_or_Plate=2 THEN
            Home_or_Plate_Response:="the tool will go to Left Plate.";
            Home_or_Plate_Answer:=2;
        ELSEIF Home_or_Plate=3 THEN
            Home_or_Plate_Response:="";
        ELSEIF Home_or_Plate=4 THEN
            Home_or_Plate_Response:="the tool will go to Right Plate .";
            Home_or_Plate_Answer:=3;
        ELSE
            Home_or_Plate_Response:="the tool will go to Right Home.";
            Home_or_Plate_Answer:=4;
        ENDIF

        TPPrint "Ok, "+Home_or_Plate_Response+" !\0D\0A";

        !Moves tool to  Home!
        IF Home_or_Plate=1 THEN
            routeAssignedEnd:=1;
            !Moves tool to the Plate!
        ELSEIF Home_or_Plate=2 THEN
            routeAssignedEnd:=6;
        ELSEIF Home_or_Plate=3 THEN
            routeAssignedEnd:=12;
        ELSEIF Home_or_Plate=4 THEN
            routeAssignedEnd:=17;
        ELSE
        ENDIF

    ENDPROC

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

        EndTarget:=waypoints{routeAssignedEnd};
        EndTargetPos:=EndTarget.trans;

        DistToolLocToEndTarget:=Distance(ToolPos,EndTargetPos);
        DistRefTargetToEndTarget:=Distance(RefTargetPos,EndTargetPos);
    ENDPROC

    PROC routeToEitherHomeLorR(num beginningRoute,num endRoute)

        !Left to Right!
        IF beginningRoute<endRoute THEN

            WHILE beginningRoute<=endRoute DO
                targetWaypoint:=waypoints{beginningRoute};
                MoveL targetWaypoint,Medium,z100,tool0\WObj:=wobj0;
                beginningRoute:=beginningRoute+1;
            ENDWHILE

            !Right to Left!
        ELSE
            WHILE beginningRoute>=endRoute DO
                targetWaypoint:=waypoints{beginningRoute};
                MoveL targetWaypoint,Medium,z100,tool0\WObj:=wobj0;
                beginningRoute:=beginningRoute-1;
            ENDWHILE

        ENDIF

    ENDPROC

    !This sets the beginning of the route...

    PROC setMoveLtoRorRtoL()
        IF beginningRoute<endRoute THEN
            MoveLtoRorRtoL:=TRUE;
            HomeOffsetPos:=LeftHomeOffsetPos;
        ELSE
            MoveLtoRorRtoL:=FALSE;
            HomeOffsetPos:=RightHomeOffsetPos;
        ENDIF
    ENDPROC


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

    PROC setToolPosNearPlateorFar()
        IF ToolPos.x<HomeOffsetPos.x THEN
            ToolPosNearPlateorFar:=TRUE;
        ELSE
            ToolPosNearPlateorFar:=FALSE;
        ENDIF
    ENDPROC


    PROC routeBeginningCalculator()
        setMoveLtoRorRtoL;
        setToolPosLorR;
        setToolPosLoworHigh;
        setToolPosNearPlateorFar;

        IF ToolPosLorR THEN
            IF ToolPosLowOrHigh THEN
                IF ToolPosNearPlateorFar THEN
                    beginningRoute:=1;
                ELSE
                    beginningRoute:=6;
                ENDIF
            ELSE
                IF ToolPosNearPlateorFar THEN
                    UpperToolOffset:=Offs(ToolRobtarget,10,10,0);
                    MoveL ToolRobtarget,Medium,z100,tool0\WObj:=wobj0;
                    routeBeginningCalculator;
                ELSE
                    beginningRoute:=7;
                ENDIF
            ENDIF
        ELSE
            IF ToolPosLowOrHigh THEN
                IF ToolPosNearPlateorFar THEN
                    UpperToolOffset:=Offs(ToolRobtarget,10,-10,0);
                    MoveL ToolRobtarget,Medium,z100,tool0\WObj:=wobj0;
                    routeBeginningCalculator;
                ELSE
                    beginningRoute:=11;
                ENDIF
            ELSE
                IF ToolPosNearPlateorFar THEN
                    beginningRoute:=12;
                ELSE
                    beginningRoute:=17;
                ENDIF
            ENDIF
        ENDIF
    ENDPROC

ENDMODULE