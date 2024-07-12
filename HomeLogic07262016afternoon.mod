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

    CONST robtarget CenterOfWork:=[[1239.97,-742.06,1268.67],[0.048654,-0.800652,-0.597151,-0.000577],[-1,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];

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
        routeHome;
    ENDPROC
    
    PROC getPositionInfo()
        !Get Location of Tool... in the proverbial "grass"
        ToolLocPos:=CPos(\Tool:=tool0\WObj:=wobj0);

        !Get an offset from the Home robtarget to use to tell if already at home position or not
        HomeOffset:= Offs(Home,250,0,0);
        !Convert this Offset to a Pos
        HomeOffsetPos:= HomeOffset.trans
        
        FarAwayHomeOffset:= Offs(FarHome,250,0,0);
        FarAwayHomeOffsetPos =
        
        
        !Translate Robtargets to Pos positions...
        !ToolLocPos:=trans.ToolLoc;
        HomePos:=Home.trans;
        CenterOfWorkPos:=CenterofWork.trans;
        NearSideLowAndPlatePos = NearSideLowAndPlate.trans;
        !This one might not be necessary...
        FarSideLowAndPlatePos = FarSideLowAndPlate.trans;
        
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
        MoveL HomeInt4,Medium,z100,tool0\WObj:=wobj0
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

    PROC nearLowToPlate ()
    	MoveL NearLowAndPlate,Medium,z100,tool0\WObj:=wobj0;
    ENDPROC
    
    PROC nearHighToPlate ()
    	MoveL NearHigh,Medium,z100,tool0\WObj:=wobj0;
    	nearLowToPlate
    ENDPROC
    
    PROC farHighToPlate
    	MoveL FarSideHigh,Medium,z100,tool0\WObj:=wobj0;
    	farTopToNear;
    	nearHighToPlate;
    ENDPROC
    
    PROC farLowToPlate()
        MoveL FarSideLowAndPlate,Medium,z100,tool0\WObj:=wobj0;
        farHighToPlate;
    ENDPROC
    
    PROC farAwayHometoPlate()
    	awayFromFarAwayHome;
    	farLowToPlate;
    	
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
    	awayFromFarAwayHome;
    	farLowToPlate;

    PROC getPositionInfo()
        !Get Location of Tool... in the proverbial "grass"
        ToolLocPos:=CPos(\Tool:=tool0\WObj:=wobj0);

        !Get an offset from the Home robtarget to use to tell if already at home position or not
        HomeOffset:= Offs(Home,250,0,0);
        !Convert this Offset to a Pos
        HomeOffsetPos:= HomeOffset.trans
        
        !Translate Robtargets to Pos positions...
        !ToolLocPos:=trans.ToolLoc;
        HomePos:=Home.trans;
        CenterOfWorkPos:=CenterofWork.trans;
        NearSideLowAndPlatePos = NearSideLowAndPlate.trans;
        !This one might not be necessary...
        FarSideLowAndPlatePos = FarSideLowAndPlate.trans;
        
        DistToolLocToPlateHome:=Distance(ToolLocPos,NearSideLowAndPlatePos);
        DistCenterOfWorkToPlateHome:=Distance(CenterOfWorkPos,NearSideLowAndPlatePos);
        
        DistToolLocToHome:=Distance(ToolLocPos,HomePos);
        DistCenterOfWorkToHome:=Distance(CenterOfWorkPos,HomePos);
 
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
    
    PROC
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
        IF (DistToolLocToHome<DistCenterOfWorkToHome) AND (ToolLocPos.z<CenterOfWorkPos.z) AND (ToolLocPos.x > HomeOffsetPos.x) THEN
            nearLowToHome;
        ELSEIF (DistToolLocToHome<DistCenterOfWorkToHome) AND (ToolLocPos.z>=CenterOfWorkPos.z) AND (ToolLocPos.x > HomeOffsetPos.x)THEN
            nearHighToHome;
        ELSEIF (DistToolLocToHome>=DistCenterOfWorkToHome) AND (ToolLocPos.z>=CenterOfWorkPos.z) AND (ToolLocPos.x > FarAwayHomeOffsetPos.x)THEN
            farHighToHome;
        ELSEIF (DistToolLocToHome>=DistCenterOfWorkToHome) AND (ToolLocPos.z<CenterOfWorkPos.z) AND (ToolLocPos.x > FarAwayHomeOffsetPos.x)THEN
            farLowToHome;
            
        !Calculates if the tool is near far from home, low to the ground AND far from the play, probably means its at the opposite side home position.    
        ELSEIF (DistToolLocToHome>=DistCenterOfWorkToHome) AND (ToolLocPos.z<CenterOfWorkPos.z) AND (ToolLocPos.x < FarAwayHomeOffsetPos.x)THEN
        	farAwayHomeToHome;
        ELSEIF (DistToolLocToHome>=DistCenterOfWorkToHome) AND (ToolLocPos.z<CenterOfWorkPos.z) AND (ToolLocPos.x < HomeOffsetPos.x)THEN
        	MoveJ Home,Medium,z100,tool0\WObj:=wobj0;
        ELSE
        	MoveJ Top,Medium,z100,tool0\WObj:=wobj0;
    ENDPROC
    
    PROC routeToPlate()
    	!Route to nearside plate from nearside Home
    	IF (DistToolLocToPlateHome<DistCenterOfWorkToPlateHome)AND (ToolLocPos.z<CenterOfWorkPos.z) AND (ToolLocPos.x < HomeOffsetPos.x) THEN
            homeToPlate;
    	    ! Calculates if the tool is near plate home in y dimension, low to the ground, and near the plate in x direction.
        IF (DistToolLocToPlateHome<DistCenterOfWorkToPlateHome) AND (ToolLocPos.z<CenterOfWorkPos.z) AND (ToolLocPos.x > HomeOffsetPos.x) THEN
            nearLowToPlate;
        ELSEIF (DistToolLocToPlateHome<DistCenterOfWorkToPlateHome) AND (ToolLocPos.z>=CenterOfWorkPos.z) AND (ToolLocPos.x > HomeOffsetPos.x)THEN
            nearHighToPlate;
        ELSEIF (DistToolLocToPlateHome>=DistCenterOfWorkToPlateHome) AND (ToolLocPos.z>=CenterOfWorkPos.z) AND (ToolLocPos.x > FarAwayHomeOffsetPos.x)THEN
            farHighToPlate;
        ELSEIF (DistToolLocToPlateHome>=DistCenterOfWorkToPlateHome) AND (ToolLocPos.z<CenterOfWorkPos.z) AND (ToolLocPos.x > FarAwayHomeOffsetPos.x)THEN
            farLowToPlate;      
        !Calculates if the tool is near or far from plate home in y dimension, low to the ground AND far from the plate in x dimension, probably means its at the opposite side home position.    
        ELSEIF (DistToolLocToPlateHome>=DistCenterOfWorkToPlateHome) AND (ToolLocPos.z<CenterOfWorkPos.z) AND (ToolLocPos.x < FarAwayHomeOffsetPos.x)THEN
        	farAwayHometoPlate;
        ELSE
        	MoveJ Top,Medium,z100,tool0\WObj:=wobj0;
    ENDPROC

ENDMODULE