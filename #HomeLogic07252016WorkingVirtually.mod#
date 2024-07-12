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
    !VAR pos RobotStartPos;

    VAR robtarget Home;
    VAR robtarget HomeInt1;
    VAR robtarget HomeInt2;
    VAR robtarget HomeInt3;
    VAR robtarget HomeInt4;

    VAR robtarget NearSideTop;
    VAR robtarget NearSideHigh;
    VAR robtarget NearSideLow;

    VAR robtarget FarSideTop;
    VAR robtarget FarSideHigh;
    VAR robtarget FarSideLow;

    !These are the variables which need to be initiated to run the TPreadFK
    VAR num Left_or_Right:=-1;
    VAR string Left_or_Right_Response:="";
    VAR num Left_or_Right_Answer:=0;

    !These are the VAR for getPositionInfo

    !instantiate variables to use in this proc...   
    !maybe break these down further...

    VAR NUM DistToolLocToHome;
    VAR NUM DistCenterOfWorkToHome;

    !These "pos" are used to find where the 

    VAR pos ToolLocPos;
    VAR pos HomePos;
    VAR pos CenterOfWorkPos;

    CONST speeddata Fast:=v200;
    CONST speeddata Medium:=v100;


    PROC Main()
        placeOnLeftOrRight;
        getPositionInfo;
        routeHome;
    ENDPROC

    !Proc Back Home 4321  
    PROC backHome()
        MoveL Home,Medium,z100,tool0\WObj:=wobj0;
        MoveL HomeInt1,Medium,z100,tool0\WObj:=wobj0;
        MoveL HomeInt2,Medium,z100,tool0\WObj:=wobj0;
        MoveL HomeInt3,Medium,z100,tool0\WObj:=wobj0;
        MoveL HomeInt4,Medium,z100,tool0\WObj:=wobj0;
    ENDPROC

    !Proc Away from Home 1234
    PROC awayFromHome()
        MoveL HomeInt4,Medium,z100,tool0\WObj:=wobj0;
        MoveL HomeInt3,Medium,z100,tool0\WObj:=wobj0;
        MoveL HomeInt2,Medium,z100,tool0\WObj:=wobj0;
        MoveL HomeInt1,Medium,z100,tool0\WObj:=wobj0;
        MoveL Home,Medium,z100,tool0\WObj:=wobj0;
    ENDPROC

    PROC nearLowToHome()
        MoveL NearSideLow,Medium,z100,tool0\WObj:=wobj0;
        backHome;
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
        MoveL FarSideLow,Medium,z100,tool0\WObj:=wobj0;
        farHighToHome;
    ENDPROC

    PROC getPositionInfo()
        !Get Location of Tool... in the proverbial "grass"
        ToolLocPos:=CPos(\Tool:=tool0\WObj:=wobj0);

        !Translate Robtargets to Pos positions...
        !ToolLocPos:=trans.ToolLoc;
        HomePos:=Home.trans;
        CenterOfWorkPos:=CenterofWork.trans;

        DistToolLocToHome:=Distance(ToolLocPos,HomePos);
        DistCenterOfWorkToHome:=Distance(CenterOfWorkPos,HomePos);
    ENDPROC

    PROC placeOnLeftOrRight()

        TPReadFK Left_or_Right,"Where do you want to place the tool","Left","","","","Right";
        IF Left_or_Right=1 THEN
            Left_or_Right_Response:="arm will move to a home position on the LEFT.";
            Left_or_Right_Answer:=1;
        ELSEIF Left_or_Right=2 THEN
            Left_or_Right_Response:="";
        ELSEIF Left_or_Right=3 THEN
            Left_or_Right_Response:="";
        ELSEIF Left_or_Right=4 THEN
            Left_or_Right_Response:="";
        ELSE
            Left_or_Right_Response:="arm will move to a home position on the RIGHT.";
            Left_or_Right_Answer:=2;
        ENDIF

        TPPrint "Great, "+Left_or_Right_Response+" !\0D\0A";

        !Moves things to the Left!
        IF Left_or_Right_Answer=1 THEN

            Home:=LeftHome;
            HomeInt1:=LeftHomeInt1;
            HomeInt2:=LeftHomeInt2;
            HomeInt3:=LeftHomeInt3;
            HomeInt4:=LeftHomeInt4;

            NearSideTop:=LeftTop;
            NearSideHigh:=LeftHigh;
            NearSideLow:=LeftLow;

            FarSideTop:=RightTop;
            FarSideHigh:=RightHigh;
            FarSideLow:=RightLow;

            !Moves things to the Right!
        ELSEIF Left_or_Right_Answer=2 THEN

            Home:=RightHome;
            HomeInt1:=RightHomeInt1;
            HomeInt2:=RightHomeInt2;
            HomeInt3:=RightHomeInt3;
            HomeInt4:=RightHomeInt4;

            NearSideLow:=RightLow;
            NearSideHigh:=RightHigh;
            NearSideTop:=RightTop;

            FarSideTop:=LeftTop;
            FarSideHigh:=LeftHigh;
            FarSideLow:=LeftLow;

        ELSE

        ENDIF

    ENDPROC

    PROC routeHome()

        IF (DistToolLocToHome<DistCenterOfWorkToHome) AND (ToolLocPos.z<CenterOfWorkPos.z) THEN
            nearLowToHome;
        ELSEIF (DistToolLocToHome<DistCenterOfWorkToHome) AND (ToolLocPos.z>=CenterOfWorkPos.z) THEN
            nearHighToHome;
        ELSEIF (DistToolLocToHome>=DistCenterOfWorkToHome) AND (ToolLocPos.z>CenterOfWorkPos.z) THEN
            farHighToHome;
        ELSE
            farLowToHome;
        ENDIF

    ENDPROC


ENDMODULE