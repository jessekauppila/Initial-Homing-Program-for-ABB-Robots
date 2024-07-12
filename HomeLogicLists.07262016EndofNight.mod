MODULE HomeLogicLists

    !definetool and workobject?

    !Left Home Sequence 

    CONST speeddata Medium:=v2000;

    CONST robtarget Home;

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
    CONST robtarget CenterOfWork:=[[1134.287973351,-71.167734203,1622.521532533],[0.500079065,-0.473042279,0.528662994,-0.496656189],[-1,-1,-1,1],[9E9,9E9,9E9,9E9,9E9,9E9]];

    !This is for working with the waypointlist
    VAR robtarget targetWaypoint;
    VAR num i;
    VAR num beginningRoute;
    VAR num endRoute;
    
    
    CONST num numWayPoints:=17;
    !This is the Waypointlist...
    VAR robtarget waypoints{numWayPoints};

    !Get an offset from the Home robtarget to use to tell if already at home position or not
    VAR robtarget HomeOffset;
    VAR robtarget FarAwayHomeOffset;

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

    !These "pos" are used to find where the 

    VAR pos PlateNearPos;
    VAR pos NearSideLowAndPlatePos;
    VAR pos FarSideLowAndPlatePos;
    VAR pos ToolLocPos;

    VAR pos HomePos;
    VAR pos HomeOffsetPos;
    VAR pos FarAwayHomeOffsetPos;

    VAR pos CenterOfWorkPos;



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

    ENDPROC
    
    
    !This sets the end of the list!
    PROC placeOnLeftOrRight()

        TPReadFK Left_or_Right,"Would do you want tool to go","Left Home","Left Plate","","Right Plate","Right Home";
        IF Left_or_Right=1 THEN
            Left_or_Right_Response:="you want the tool to go the Left.";
            Left_or_Right_Answer:=1;
        ELSEIF Left_or_Right=2 THEN
            Left_or_Right_Response:="";
            Left_or_Right_Answer:=2;
        ELSEIF Left_or_Right=3 THEN
            Left_or_Right_Response:="";
        ELSEIF Left_or_Right=4THEN
            Left_or_Right_Response:="";
            Left_or_Right_Answer:=3;
        ELSE
            Left_or_Right_Response:="arm will move to the Right home.";
            Left_or_Right_Answer:=4;
        ENDIF

        TPPrint "Ok, "+Left_or_Right_Response+" !\0D\0A";

        !Moves things to the Left!
        IF Left_or_Right_Answer=1 THEN
            
            !EndTarget = RightHome

            !Moves things to the Right!
        ELSEIF Left_or_Right_Answer=2 THEN
            homeToRightSideRoute;

        ELSE

        ENDIF

    ENDPROC
    
    PROC moveFromLeftHomeToRightHome(num beginningRoute,num endRoute)
        WHILE beginningRoute <= endRoute DO
            targetWaypoint:=waypoints{beginningRoute};
            MoveL targetWaypoint,Medium,z100,tool0\WObj:=wobj0;
            beginningRoute:=beginningRoute+1;
        ENDWHILE
    ENDPROC
    
    PROC moveFromRightHomeToLeftHome(num beginningRoute,num endRoute)
        WHILE beginningRoute >= endRoute DO
            targetWaypoint := waypoints{beginningRoute};
            MoveL targetWaypoint,Medium,z100,tool0\WObj := wobj0;
            beginningRoute := beginningRoute-1;
        ENDWHILE

    ENDPROC

    PROC getPositionInfo()
        !Get Location of Tool... in the proverbial "grass"
        ToolLocPos:=CPos(\Tool:=tool0\WObj:=wobj0);

        !Get an offset from the Home robtarget to use to tell if already at home position or not
        HomeOffset:=Offs(Home,250,0,0);
        !Convert this Offset to a Pos
        HomeOffsetPos:=HomeOffset.trans;

        FarAwayHomeOffset:=Offs(FarHome,250,0,0);
        FarAwayHomeOffsetPos:=FarAwayHomeOffset.trans;

        !Translate Robtargets to Pos positions...
        !ToolLocPos:=trans.ToolLoc;
        HomePos:=Home.trans;
        CenterOfWorkPos:=CenterofWork.trans;
        NearSideLowAndPlatePos:=NearSideLowAndPlate.trans;
        !This one might not be necessary...
        FarSideLowAndPlatePos:=FarSideLowAndPlate.trans;

        DistToolLocToPlateHome:=Distance(ToolLocPos,NearSideLowAndPlatePos);
        DistCenterOfWorkToPlateHome:=Distance(CenterOfWorkPos,NearSideLowAndPlatePos);

        DistToolLocToHome:=Distance(ToolLocPos,HomePos);
        DistCenterOfWorkToHome:=Distance(CenterOfWorkPos,HomePos);
    ENDPROC

    !Could this be something like route calculator or something like that?
    
    PROC routeCalculator()
        IF (DistToolLocToHome<DistCenterOfWorkToHome) AND (ToolLocPos.z<CenterOfWorkPos.z) AND (ToolLocPos.x>HomeOffsetPos.x) THEN
            nearLowToHome;
        ELSEIF (DistToolLocToHome<DistCenterOfWorkToHome) AND (ToolLocPos.z>=CenterOfWorkPos.z) AND (ToolLocPos.x>HomeOffsetPos.x) THEN
            nearHighToHome;
        ELSEIF (DistToolLocToHome>=DistCenterOfWorkToHome) AND (ToolLocPos.z>=CenterOfWorkPos.z) AND (ToolLocPos.x>FarAwayHomeOffsetPos.x) THEN
            farHighToHome;
        ELSEIF (DistToolLocToHome>=DistCenterOfWorkToHome) AND (ToolLocPos.z<CenterOfWorkPos.z) AND (ToolLocPos.x>FarAwayHomeOffsetPos.x) THEN
            farLowToHome;
            !Calculates if the tool is near far from home, low to the ground AND far from the play, probably means its at the opposite side home position.    
        ELSEIF (DistToolLocToHome>=DistCenterOfWorkToHome) AND (ToolLocPos.z<CenterOfWorkPos.z) AND (ToolLocPos.x<FarAwayHomeOffsetPos.x) THEN
            farAwayHomeToNearHome;
        ELSEIF (DistToolLocToHome>=DistCenterOfWorkToHome) AND (ToolLocPos.z<CenterOfWorkPos.z) AND (ToolLocPos.x<HomeOffsetPos.x) THEN
            MoveJ Home,Medium,z100,tool0\WObj:=wobj0;

        ELSE
            MoveJ Top,Medium,z100,tool0\WObj:=wobj0;
        ENDIF
    ENDPROC

ENDMODULE