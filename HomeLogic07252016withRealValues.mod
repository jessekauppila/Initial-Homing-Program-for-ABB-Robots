    MODULE HomeLogic

    !definetool and workobject?

    !Left Home Sequence 
    CONST robtarget LeftHome:=[[576.85,1394.56,-612.51],[0.509771,0.539227,-0.479592,-0.468359],[0,-2,0,1],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget LeftHomeInt1:=[[576.85,1394.56,-612.51],[0.114649,0.144984,-0.689844,-0.699965],[0,-1,0,1],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget LeftHomeInt2:=[[848.55,1394.57,-101.1],[0.156024,0.241541,0.825171,0.486219],[0,0,0,1],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget LeftHomeInt3:=[[848.55,1394.57,18.07],[0.262988,0.312453,0.79354,0.451115],[0,0,0,1],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget LeftHomeInt4:=[[1167.86,1394.59,188.72],[0.267191,0.437633,0.678207,0.526424],[0,0,1,1],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];

    CONST robtarget LeftLow:=[[1304.39,1181.14,656.35],[0.041734,0.725074,0.686734,0.030393],[0,1,-1,1],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget LeftHigh:=[[1434.23,1442.35,2349.62],[0.033258,0.713022,0.699985,0.022699],[0,1,-1,1],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget LeftTop:=[[1073,1056.21,2720.53],[0.360733,0.617475,0.611089,0.339363],[0,1,-1,1],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];

    CONST robtarget Top:=[[589.15,-232.43,3269.58],[0.573092,0.594807,0.417225,0.37907],[-1,2,-2,1],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];

    CONST robtarget RightTop:=[[76.05,-1322.06,2968.83],[0.37033,0.788742,0.472236,0.133176],[-2,2,-2,1],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget RightHigh:=[[61.04,-2103.13,2035.96],[0.116709,0.878807,0.459988,0.04988],[-1,2,-2,1],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget RightLow:=[[-287.84,-2070.36,65.02],[0.026826,0.901845,0.431199,0.004971],[-2,2,-2,1],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];

    CONST robtarget RightHomeInt4:=[[-322.21,-1658.14,-677.51],[0.334256,0.336793,0.589508,0.6537],[-2,2,-3,1],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget RightHomeInt3:=[[-322.21,-1658.14,-677.51],[0.334256,0.336793,0.589508,0.6537],[-2,2,-3,1],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget RightHomeInt2:=[[-322.21,-1658.14,-677.51],[0.334256,0.336793,0.589508,0.6537],[-2,2,-3,1],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget RightHomeInt1:=[[-322.21,-1658.14,-677.51],[0.334256,0.336793,0.589508,0.6537],[-2,2,-3,1],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget RightHome:=[[-721.44,-1589.68,-584.17],[0.312267,0.264728,0.705871,0.578062],[-2,2,-4,1],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];

    !This is defined by the center of the working area, the nearest reach of the robot
    CONST robtarget CenterOfWork:=[[589.15,-232.43,1750],[0.573092,0.594807,0.417225,0.37907],[-1,2,-2,1],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];

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
        getPositionInfo;
        placeOnLeftOrRight;
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
        MoveL NearSideHigh,Medium,z100,tool0\WObj:=wobj0;
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