MODULE HomeLogicLists

    !definetool and workobject?

    !Left Home Sequence 

    CONST speeddata Medium:=v2000;
    !!

    CONST robtarget LeftHome:=[[-302.658186353,1928.793039868,853.821807468],[0.4740388,-0.59558332,-0.428922395,0.486408577],[1,-3,1,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    CONST robtarget LeftHomeInt1:=[[-120.757907588,2301.052168385,975.47337927],[0.794375915,-0.500091671,0.176899579,0.295942166],[1,-2,1,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    CONST robtarget LeftHomeInt2:=[[69.878422646,2190.395876792,889.262927503],[0.709034897,-0.226131266,0.667664527,0.018927373],[1,-2,1,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    CONST robtarget LeftHomeInt3:=[[387.611185172,1858.836061626,933.900896261],[0.640121845,-0.105228291,0.752578167,-0.113124419],[0,-1,1,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    CONST robtarget LeftHomeInt4:=[[688.863718443,1812.0142454,960.100673973],[0.689122052,-0.121916995,0.705041624,-0.11473165],[0,-2,1,0],[9E9,9E9,9E9,9E9,9E9,9E9]];

    CONST robtarget LeftLow:=[[915.138584814,1520.566976595,1118.336794154],[0.739266034,-0.109136719,0.654614229,-0.114258998],[0,-2,1,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    CONST robtarget LeftHigh:=[[932.355096194,1557.499065889,2189.512825695],[0.621096131,0.280780119,0.678796056,0.273199625],[0,-1,1,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    CONST robtarget LeftTop:=[[1166.00538664,1009.312410792,2785.776227393],[0.021731562,0.507455004,0.520203705,0.686589588],[0,0,1,0],[9E9,9E9,9E9,9E9,9E9,9E9]];

    CONST robtarget Top:=[[788.963232892,-129.350925083,2974.184058033],[0.369138985,0.565008868,0.217420071,0.705145305],[-1,0,1,0],[9E9,9E9,9E9,9E9,9E9,9E9]];

    CONST robtarget RightTop:=[[944.54676107,-1244.677654795,2812.254483498],[0.067607469,0.668127955,-0.002842736,0.740963012],[-1,0,1,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    CONST robtarget RightHigh:=[[476.58391291,-1786.11978634,2486.590234982],[0.029212053,-0.581176654,0.234319397,-0.778764902],[-1,1,1,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
	CONST robtarget RightLow:=[[469.18851562,-1742.679809936,1244.384809238],[0.287716006,-0.5562151,0.583816604,-0.516722784],[-1,1,1,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
	
    CONST robtarget RightHomeInt4:=[[122.917433222,-2043.681471833,1069.659175047],[0.057830838,0.501387856,-0.769217905,0.391879608],[-1,0,1,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
	CONST robtarget RightHomeInt3:=[[-266.079574566,-2069.073109573,925.625829738],[0.411748897,0.312314395,-0.842194687,0.15372272],[-2,0,1,1],[9E9,9E9,9E9,9E9,9E9,9E9]];
	CONST robtarget RightHomeInt2:=[[-609.030763449,-1944.499857738,1017.40926892],[0.648334487,0.119996966,-0.750316134,-0.047841603],[-2,0,1,1],[9E9,9E9,9E9,9E9,9E9,9E9]];
	CONST robtarget RightHomeInt1:=[[-835.069486205,-1453.356126771,1247.636561163],[0.82862309,-0.068120828,-0.555266912,-0.020542266],[-2,0,1,1],[9E9,9E9,9E9,9E9,9E9,9E9]];
	CONST robtarget RightHome:=[[-709.298655402,-1224.189885559,1387.058152156],[0.896131094,-0.198236624,-0.390558167,-0.071523565],[-2,0,1,1],[9E9,9E9,9E9,9E9,9E9,9E9]];
    
    
    
    !This is defined by the center of the working area, the nearest reach of the robot
    CONST robtarget RefTarget:=[[1134.287973351,-71.167734203,1622.521532533],[0.500079065,-0.473042279,0.528662994,-0.496656189],[-1,-1,-1,1],[9E9,9E9,9E9,9E9,9E9,9E9]];

    !This is for working with the waypointlist
    VAR robtarget targetWaypoint;
    VAR num i;

    VAR num waypointEnroute;

    !These are for within the PROC routeLeftHomeRightHome
    !VAR num beginningOfRoute;
    VAR num endRoute;

    !These are chosen evertime in the whereToEndRoute Proc
    VAR num routeAssignedBeginning;
    VAR num routeAssignedEnd;

    !This is the Waypointlist...
    VAR robtarget waypoints{numWayPoints};
    CONST num numWayPoints:=17;
    
    Var robtarget EndTarget;

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

     !Get an offset from the Home robtarget to use to tell if already at home position or not
    VAR robtarget LeftHomeOffset;
    VAR robtarget RightHomeOffset;

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
    VAR bool ToolPosonLeftSide;
    VAR bool ToolPosLow;
    VAR bool ToolPosNearPlateorFar;
    
    VAR bool LeftToolPosFarFromPlate;
    VAR bool RightToolPosFarFromPlate;

    VAR robtarget ToolRobtarget;

    !This is to try to move the robot out of the upper left or upper left areas away from the piece
    VAR robtarget UpperToolOffset;

!
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

        whereToEndRoute;
        !sets routeAssignedEnd
        getPositionInfo;
        !calculates what side starting at and other values
        routeBeginningCalculator;
        !calculates where to begin rout...
        routeToEitherHomeLorR;
        !I took these parameters out to see if it might, just might run!routeAssignedBeginning,routeAssignedEnd;
!
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

    PROC whereToEndRoute()

        TPReadFK Home_or_Plate,"Where do you want to move the tool?","Left Home","Left Plate","","Right Plate","Right Home";
        IF Home_or_Plate=1 THEN
            Home_or_Plate_Response:="the tool will go to Left Home";
            Home_or_Plate_Answer:=1;
        ELSEIF Home_or_Plate=2 THEN
            Home_or_Plate_Response:="the tool will go to Left Plate";
            Home_or_Plate_Answer:=2;
        ELSEIF Home_or_Plate=3 THEN
            Home_or_Plate_Response:="";
        ELSEIF Home_or_Plate=4 THEN
            Home_or_Plate_Response:="the tool will go to Right Plate";
            Home_or_Plate_Answer:=3;
        ELSE
            Home_or_Plate_Response:="the tool will go to Right Home";
            Home_or_Plate_Answer:=4;
        ENDIF

        TPPrint "Ok, "+Home_or_Plate_Response+"!\0D\0A";

        !Moves tool to  Home!



        IF Home_or_Plate=1 THEN
            routeAssignedEnd:=1;
            writeEndOfRouteTarget;
        ELSEIF Home_or_Plate=2 THEN
            routeAssignedEnd:=6;
            writeEndOfRouteTarget;
        ELSEIF Home_or_Plate=3 THEN
            routeAssignedEnd:=12;
            writeEndOfRouteTarget;
        ELSEIF Home_or_Plate=4 THEN
            routeAssignedEnd:=17;
            writeEndOfRouteTarget;
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

        LeftHomeOffset:=Offs(LeftHome,50,0,0);
        RightHomeOffset:=Offs(RightHome,50,0,0);

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

    PROC routeToEitherHomeLorR()
        !was num waypointEnroute,num endRoute
        !writeBeginningOfRouteTarget;
        !Left to Right!

        writeCheckAssignedBegin;
        writeBeginningOfRoute;

        !waypointEnroute := beginningOfRoute;

        !routeAssignedEnd


        endRoute:=routeAssignedEnd;

        IF routeAssignedBeginning<endRoute THEN
            
            waypointEnroute:=routeAssignedBeginning;
            !This says if moving from left to right....
            WHILE waypointEnroute<=endRoute DO
                targetWaypoint:=waypoints{waypointEnroute};
                writeTargetWaypointEnroute;
                MoveL targetWaypoint,Medium,z100,tool0\WObj:=wobj0;
                writeReachedWaypointEnroute;
                waypointEnroute:=waypointEnroute+1;
            ENDWHILE

            !Right to Left!
        ELSE
            
            waypointEnroute:=routeAssignedBeginning;
            WHILE waypointEnroute>endRoute DO
                targetWaypoint:=waypoints{waypointEnroute};
                writeTargetWaypointEnroute;
                MoveL targetWaypoint,Medium,z100,tool0\WObj:=wobj0;
                writeReachedWaypointEnroute;
                waypointEnroute:=waypointEnroute-1;
            ENDWHILE

        ENDIF

    ENDPROC

    !This sets the beginning of the route...


    !This should be something like if

    PROC isToolPosonLeftSide()
        !This should detect if the tool is on the left side or on the right side....
        IF ToolPos.y>RefPos.y THEN
            ToolPosonLeftSide:=TRUE;
            ! This means if its on the right
        ELSE
            ToolPosonLeftSide:=FALSE;
            !This means tool is on the 
        ENDIF
    ENDPROC

    PROC isToolPosLow()
        IF ToolPos.z<RefPos.z THEN
            ToolPosLow:=TRUE;
        ELSE
            ToolPosLow:=FALSE;
        ENDIF
    ENDPROC

    
    
    PROC isLeftToolPosFarFromPlate()
       
        IF ToolPos.x<LeftHomeOffsetPos.x THEN !This is Tricky, Its based off of where the home position is...
            LeftToolPosFarFromPlate:=TRUE;
        ELSE
            LeftToolPosFarFromPlate:=FALSE;
        ENDIF
    ENDPROC
    
    PROC isRightToolPosFarFromPlate()
        
        IF ToolPos.x<RightHomeOffsetPos.x THEN !This is Tricky, Its based off of where the home position is...
            RightToolPosFarFromPlate:=TRUE;
        ELSE
            RightToolPosFarFromPlate:=FALSE;
        ENDIF
    ENDPROC
    
    PROC routeBeginningCalculator()
        !setMoveLtoRorRtoL;
        isToolPosonLeftSide;
        isToolPosLow;
        isLeftToolPosFarFromPlate;
        isRightToolPosFarFromPlate;

        !IF MoveLtoRorRtoL THEN ! This is creating some sort of recursion because I am using the beginning value to calculate if I'm moveing from left ro write!
        IF ToolPosonLeftSide THEN 
            TPWRITE "Tool on Left Side";
            IF ToolPosLow THEN
                TPWRITE "Tool on Left Side, Low";
                IF LeftToolPosFarFromPlate THEN
                TPWrite "Tool on Left, Low, Far From Plate";
                    routeAssignedBeginning:=1;
                    writeBeginCalcAssignedBegin;
                ELSE !LeftSideHigh
                TPWrite "Tool on Left, Low, Near Plate";
                    routeAssignedBeginning:=6;
                    writeBeginCalcAssignedBegin;
                ENDIF
            ELSE
                TPWRITE "Tool on Left Side, High";
                IF LeftToolPosFarFromPlate THEN
                    TPWrite "Tool on Left, High, Far From Plate";
                    UpperToolOffset:=Offs(ToolRobtarget,10,10,0);
                    MoveL ToolRobtarget,Medium,z100,tool0\WObj:=wobj0;
                    !routeBeginningCalculator;
                ELSE
                    TPWrite "Tool on Left, High, Near Plate";
                    routeAssignedBeginning:=7;
                    writeBeginCalcAssignedBegin;
                ENDIF
            ENDIF
        ELSE 
            TPWRITE "Tool on Right Side";
            IF ToolPosLow THEN
                TPWRITE "Tool on Right Side, Low";
                IF RightToolPosFarFromPlate THEN
                    TPWRITE "Tool on Right Side, Low, Far from Plate";
                    routeAssignedBeginning:=17;
                    writeBeginCalcAssignedBegin;
                ELSE
                    TPWRITE "Tool on Right Side, Low, Near Plate";
                    routeAssignedBeginning:=12;
                    writeBeginCalcAssignedBegin;
                ENDIF
            ELSE
                TPWRITE "Tool on Right Side, High";
                IF RightToolPosFarFromPlate THEN
                    TPWRITE "Tool on Right Side, High, Far from Plate";
                    UpperToolOffset:=Offs(ToolRobtarget,10,-10,0);
                    MoveL ToolRobtarget,Medium,z100,tool0\WObj:=wobj0;
                    !routeBeginningCalculator;
                ELSE
                    TPWRITE "Tool on Right Side, High, Near Plate";
                    routeAssignedBeginning:=11;
                    writeBeginCalcAssignedBegin;
                ENDIF
            ENDIF
        ENDIF  
    ENDPROC

ENDMODULE