MODULE HomeLogic

    !definetool and workobject?

    !Left Home Sequence 

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

    !This is where the robot is located, where it starts this routine..

    VAR robtarget Home;
    VAR robtarget HomeInt1;
    VAR robtarget HomeInt2;
    VAR robtarget HomeInt3;
    VAR robtarget HomeInt4;

    VAR robtarget NearSideTop;
    VAR robtarget NearSideHigh;
    VAR robtarget NearSideLowAndPlate;

    VAR robtarget FarSideTop;
    VAR robtarget FarSideHigh;
    VAR robtarget FarSideLowAndPlate;

    VAR robtarget FarHome;
    VAR robtarget FarHomeInt1;
    VAR robtarget FarHomeInt2;
    VAR robtarget FarHomeInt3;
    VAR robtarget FarHomeInt4;

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

    CONST speeddata Fast:=v200;
    CONST speeddata Medium:=v100;


    PROC Main()
        placeOnLeftOrRight;
        getPositionInfo;
        placeOnHomeOrPlate;
    
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

    !Proc Back Home 4321 
    PROC plateToHome()
        MoveL HomeInt1,Medium,z100,tool0\WObj:=wobj0;
        MoveL HomeInt2,Medium,z100,tool0\WObj:=wobj0;
        MoveL HomeInt3,Medium,z100,tool0\WObj:=wobj0;
        MoveL HomeInt4,Medium,z100,tool0\WObj:=wobj0;
        MoveL Home,Medium,z100,tool0\WObj:=wobj0;
    ENDPROC

    !Proc Away from Home 1234 
    PROC homeToPlate()
        MoveL Home,Medium,z100,tool0\WObj:=wobj0;
        MoveL HomeInt4,Medium,z100,tool0\WObj:=wobj0;
        MoveL HomeInt3,Medium,z100,tool0\WObj:=wobj0;
        MoveL HomeInt2,Medium,z100,tool0\WObj:=wobj0;
        MoveL HomeInt1,Medium,z100,tool0\WObj:=wobj0;
    ENDPROC

    !This is if for some reason you want to switch home positions from one side to another
    PROC farPlateToFarAwayHome()
        MoveL FarHomeInt1,Medium,z100,tool0\WObj:=wobj0;
        MoveL FarHomeInt2,Medium,z100,tool0\WObj:=wobj0;
        MoveL FarHomeInt3,Medium,z100,tool0\WObj:=wobj0;
        MoveL FarHomeInt4,Medium,z100,tool0\WObj:=wobj0;
        MoveL FarHome,Medium,z100,tool0\WObj:=wobj0;
    ENDPROC

    !This is if for some reason you want to switch home or plate positions from one side to another
    PROC farAwayHomeToFarPlate()
        MoveL FarHome,Medium,z100,tool0\WObj:=wobj0;
        MoveL FarHomeInt4,Medium,z100,tool0\WObj:=wobj0;
        MoveL FarHomeInt3,Medium,z100,tool0\WObj:=wobj0;
        MoveL FarHomeInt2,Medium,z100,tool0\WObj:=wobj0;
        MoveL FarHomeInt1,Medium,z100,tool0\WObj:=wobj0;
    ENDPROC

    PROC nearLowToPlate()
        MoveL NearSideLowAndPlate,Medium,z100,tool0\WObj:=wobj0;
    ENDPROC

    PROC nearHighToPlate()
        MoveL NearSideHigh,Medium,z100,tool0\WObj:=wobj0;
        nearLowToPlate;
    ENDPROC

    PROC farHighToPlate()
        MoveL FarSideHigh,Medium,z100,tool0\WObj:=wobj0;
        farTopToNear;
        nearHighToPlate;
    ENDPROC

    PROC farLowToPlate()
        MoveL FarSideLowAndPlate,Medium,z100,tool0\WObj:=wobj0;
        farHighToPlate;
    ENDPROC  

    PROC nearLowToHome()
        MoveL NearSideLowAndPlate,Medium,z100,tool0\WObj:=wobj0;
        plateToHome;
    ENDPROC

    PROC nearHighToHome()
        MoveL NearSideHigh,Medium,z100,tool0\WObj:=wobj0;
        nearLowToHome;
    ENDPROC

    PROC farTopToNear()
        MoveL FarSideTop,Medium,z100,tool0\WObj:=wobj0;
        MoveL Top,Medium,z100,tool0\WObj:=wobj0;
        MoveL NearSideTop,Medium,z100,tool0\WObj:=wobj0;
    ENDPROC

    PROC nearTopToFar()
        MoveL NearSideTop,Medium,z100,tool0\WObj:=wobj0;
        MoveL Top,Medium,z100,tool0\WObj:=wobj0;
        MoveL FarSideTop,Medium,z100,tool0\WObj:=wobj0;
    ENDPROC

    PROC farHighToHome()
        MoveL FarSideHigh,Medium,z100,tool0\WObj:=wobj0;
        farTopToNear;
        nearHighToHome;
    ENDPROC

    PROC farLowToHome()
        MoveL FarSideLowAndPlate,Medium,z100,tool0\WObj:=wobj0;
        farHighToHome;
    ENDPROC
    
    PROC farAwayHometoPlate()
        farAwayHomeToFarPlate;
        farLowToPlate;
    ENDPROC
        
    PROC farAwayHomeToNearHome()
        farAwayHometoPlate;
        plateToHome;
        
        
    ENDPROC

    
    
    PROC homeToLeftSideRoute()
        Home:=LeftHome;
        HomeInt1:=LeftHomeInt1;
        HomeInt2:=LeftHomeInt2;
        HomeInt3:=LeftHomeInt3;
        HomeInt4:=LeftHomeInt4;

        NearSideTop:=LeftTop;
        NearSideHigh:=LeftHigh;
        NearSideLowAndPlate:=LeftLow;

        FarSideTop:=RightTop;
        FarSideHigh:=RightHigh;
        FarSideLowAndPlate:=RightLow;

        FarHome:=RightHome;
        FarHomeInt1:=RightHomeInt1;
        FarHomeInt2:=RightHomeInt2;
        FarHomeInt3:=RightHomeInt3;
        FarHomeInt4:=RightHomeInt4;

    ENDPROC

    PROC homeToRightSideRoute()

        Home:=RightHome;
        HomeInt1:=RightHomeInt1;
        HomeInt2:=RightHomeInt2;
        HomeInt3:=RightHomeInt3;
        HomeInt4:=RightHomeInt4;

        NearSideLowAndPlate:=RightLow;
        NearSideHigh:=RightHigh;
        NearSideTop:=RightTop;

        FarSideTop:=LeftTop;
        FarSideHigh:=LeftHigh;
        FarSideLowAndPlate:=LeftLow;

        FarHome:=LeftHome;
        FarHomeInt1:=LeftHomeInt1;
        FarHomeInt2:=LeftHomeInt2;
        FarHomeInt3:=LeftHomeInt3;
        FarHomeInt4:=LeftHomeInt4;

    ENDPROC

    PROC placeOnLeftOrRight()

        TPReadFK Left_or_Right,"Would you like to return the tool to the Left OR Right.","Left","","","","Right";
        IF Left_or_Right=1 THEN
            Left_or_Right_Response:="you want the tool to go the Left.";
            Left_or_Right_Answer:=1;
        ELSEIF Left_or_Right=2 THEN
            Left_or_Right_Response:="";
        ELSEIF Left_or_Right=3 THEN
            Left_or_Right_Response:="";
        ELSEIF Left_or_Right=4 THEN
            Left_or_Right_Response:="";
        ELSE
            Left_or_Right_Response:="arm will move to the Right home.";
            Left_or_Right_Answer:=2;
        ENDIF

        TPPrint "Ok, "+Left_or_Right_Response+" !\0D\0A";

        !Moves things to the Left!
        IF Left_or_Right_Answer=1 THEN
            homeToLeftSideRoute;

            !Moves things to the Right!
        ELSEIF Left_or_Right_Answer=2 THEN
            homeToRightSideRoute;

        ELSE

        ENDIF

    ENDPROC


    !These are the variables which need to be initiated to run the TPreadFK for Home or Plate choice
    !VAR num Home_or_Plate:=-1;
    ! VAR string Home_or_Plate_Response:="";
    !VAR num Home_or_Plate_Answer:=0;


    PROC placeOnHomeOrPlate()

        TPReadFK Home_or_Plate,"Would you like to a maintenance Home or to the Plate?","Home","","","","Plate";
        IF Home_or_Plate=1 THEN
            Home_or_Plate_Response:="the tool will go Home.";
            Home_or_Plate_Answer:=1;
        ELSEIF Home_or_Plate=2 THEN
            Home_or_Plate_Response:="";
        ELSEIF Home_or_Plate=3 THEN
            Home_or_Plate_Response:="";
        ELSEIF Home_or_Plate=4 THEN
            Home_or_Plate_Response:="";
        ELSE
            Home_or_Plate_Response:="the tool will go to the Plate.";
            Home_or_Plate_Answer:=2;
        ENDIF

        TPPrint "Ok, "+Home_or_Plate_Response+" !\0D\0A";

        !Moves tool to  Home!
        IF Home_or_Plate=1 THEN
            moveHome;

            !Moves tool to the Plate!
        ELSEIF Home_or_Plate=2 THEN
            moveToPlate;

        ELSE

        ENDIF

    ENDPROC

    PROC moveHome()
        ! Calculates if the tool is near home, low to the ground, and near the plate.
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

    PROC moveToPlate()
        !Route to nearside plate from nearside Home
        IF (DistToolLocToPlateHome<DistCenterOfWorkToPlateHome) AND (ToolLocPos.z<CenterOfWorkPos.z) AND (ToolLocPos.x<HomeOffsetPos.x) THEN
            homeToPlate;
            ! Calculates if the tool is near plate home in y dimension, low to the ground, and near the plate in x direction.
        ELSEIF (DistToolLocToPlateHome<DistCenterOfWorkToPlateHome) AND (ToolLocPos.z<CenterOfWorkPos.z) AND (ToolLocPos.x>HomeOffsetPos.x) THEN
            nearLowToPlate;
        ELSEIF (DistToolLocToPlateHome<DistCenterOfWorkToPlateHome) AND (ToolLocPos.z>=CenterOfWorkPos.z) AND (ToolLocPos.x>HomeOffsetPos.x) THEN
            nearHighToPlate;
        ELSEIF (DistToolLocToPlateHome>=DistCenterOfWorkToPlateHome) AND (ToolLocPos.z>=CenterOfWorkPos.z) AND (ToolLocPos.x>FarAwayHomeOffsetPos.x) THEN
            farHighToPlate;
        ELSEIF (DistToolLocToPlateHome>=DistCenterOfWorkToPlateHome) AND (ToolLocPos.z<CenterOfWorkPos.z) AND (ToolLocPos.x>FarAwayHomeOffsetPos.x) THEN
            farLowToPlate;
            !Calculates if the tool is near or far from plate home in y dimension, low to the ground AND far from the plate in x dimension, probably means its at the opposite side home position.    
        ELSEIF (DistToolLocToPlateHome>=DistCenterOfWorkToPlateHome) AND (ToolLocPos.z<CenterOfWorkPos.z) AND (ToolLocPos.x<FarAwayHomeOffsetPos.x) THEN
            farAwayHomeToNearHome;
        ELSE
            MoveJ Top,Medium,z100,tool0\WObj:=wobj0;
        ENDIF

    ENDPROC

ENDMODULE