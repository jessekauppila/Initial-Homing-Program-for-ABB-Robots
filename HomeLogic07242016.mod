MODULE HomeLogic
    
    !definetool and workobject?

    CONST robtarget LeftHome:=[[-10.078042494,1824.668831942,802.431280345],[0.508337887,-0.501604166,0.508830774,0.480704791],[1,-2,1,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    CONST robtarget LeftLow:=[[689.414723237,1824.668910821,1159.28624599],[0.699324562,0.000122117,0.714800341,0.002369495],[0,-2,1,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    CONST robtarget LeftHigh:=[[689.415668582,1843.736962037,2261.41568216],[0.699324234,0.000122309,0.71480066,0.002369843],[0,-1,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    CONST robtarget LeftTop:=[[689.414975299,1053.174954768,2837.483396634],[0.69932429,0.000121762,0.714800608,0.002369449],[0,-1,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    CONST robtarget Top:=[[695.447397289,-80.906517867,3009.199597555],[0.789570357,-0.133382371,0.533031764,-0.273248848],[-1,0,-1,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    CONST robtarget RightTop:=[[930.843689193,-1249.46499759,2732.440078441],[0.699324741,0.000122762,0.714800168,0.002368753],[-1,0,-1,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    CONST robtarget RightHigh:=[[788.029576424,-2141.291550396,2089.025775678],[0.699324542,0.000122445,0.714800361,0.002369197],[-1,0,-1,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    CONST robtarget RightLow:=[[615.6264138,-1970.938238655,1091.520628553],[0.699324651,0.000122961,0.714800255,0.002369161],[-1,1,-2,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    CONST robtarget RightHome:=[[2.648006257,-1970.937085546,721.148890896],[0.506219517,0.495009538,0.515659034,-0.482496754],[-2,1,-2,0],[9E9,9E9,9E9,9E9,9E9,9E9]];

    !This is defined by the center of the working area, the nearest reach of the robot
    CONST robtarget CenterOfWork:=[[1134.287973351,-71.167734203,1622.521532533],[0.500079065,-0.473042279,0.528662994,-0.496656189],[-1,-1,-1,1],[9E9,9E9,9E9,9E9,9E9,9E9]];

    !This is where the robot is located, where it starts this routine..
    !VAR pos RobotStartPos;

    VAR robtarget NearSideTop;
    VAR robtarget NearSideHigh;
    VAR robtarget NearSideLow;

    VAR robtarget FarSideTop;
    VAR robtarget FarSideHigh;
    VAR robtarget FarSideLow;

    VAR robtarget Home;

    !These are the variables which need to be initiated to run the TPreadFK
    VAR num Left_or_Right:=-1;
    VAR string Left_or_Right_Response:="";
    VAR num Left_or_Right_Answer:=0;

    !These are the VAR for getPositionInfo
    
     !instantiate variables to use in this proc...   
        !maybe break these down further...
        
        VAR NUM DistToolLocToHome;
        VAR NUM DistToolLocToCenterOfWorkToHome;
        
        !These "pos" are used to find where the 
        
        VAR pos ToolLocPos;
        VAR pos HomePos;
        VAR pos CenterOfWorkPos;
        
        


    PROC Main()

        getPositionInfo;

        leftOrRightHome;
    
        routeHome;
        
    ENDPROC
    
    PROC getPositionInfo()
        
        !Translate Robtargets to Pos positions...
        ToolLocPos := trans.ToolLocPos;
        HomePos := trans.Home;
        CenterOfWorkPos := trans.CenterofWork;
       
        ToolLoc:=CPos(\Tool:=tool0\WObj:=wobj0);
        
        DistToolLocToHome := Distance(ToolLocPos,HomePos);
        DistToolLocToCenterOfWorkToHome := Distance(CenterOfWorkPos,HomePos);
         
    ENDPROC
    
    


    PROC leftOrRightHome()

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

        IF Left_or_Right_Answer=1 THEN

            NearSideTop:=LeftTop;
            NearSideHigh:=LeftHigh;
            NearSideLow:=LeftLow;

            FarSideTop:=RightTop;
            FarSideHigh:=RightHigh;
            FarSideLow:=RightLow;

            Home:=LeftHome;

        ELSEIF Left_or_Right_Answer=2 THEN

            NearSideTop:=RightTop;
            NearSideHigh:=RightHigh;
            NearSideLow:=RightLow;

            FarSideTop:=LeftTop;
            FarSideHigh:=LeftHigh;
            FarSideLow:=LeftLow;

            Home:=RightHome;
        ELSE

        ENDIF

    ENDPROC

    PROC routeHome()
        
        IF DistToolLocToHome < DistToolLocToCenterofWorkToHome AND ToolLocPos.z < CenterOfWorkPos.z
        
        MoveL NearSideLow,v1000,z100,tool0\WObj:=wobj0;
        MoveL Home,v1000,z100,tool0\WObj:=wobj0;
        
        ELSEIF DistToolLocToHome < DistToolLocToCenterofWorkToHome AND ToolLocPos.z < CenterOfWorkPos.z
        
        ELSE
        
        MoveL FarSideLow,v1000,z100,tool0\WObj:=wobj0;
        MoveL FarSideHigh,v1000,z100,tool0\WObj:=wobj0;
        MoveL FarSideTop,v1000,z100,tool0\WObj:=wobj0;
        MoveL Top,v1000,z100,tool0\WObj:=wobj0;
        MoveL NearSideTop,v1000,z100,tool0\WObj:=wobj0;
        MoveL NearSideHigh,v1000,z100,tool0\WObj:=wobj0;
        MoveL NearSideLow,v1000,z100,tool0\WObj:=wobj0;
        MoveL Home,v1000,z100,tool0\WObj:=wobj0;

    ENDPROC




ENDMODULE