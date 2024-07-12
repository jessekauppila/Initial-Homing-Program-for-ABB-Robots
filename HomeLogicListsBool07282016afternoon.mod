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
	CONST robtarget RightLow:=[[496.404249706,-1902.542870718,1660.145754417],[0.203729583,-0.582235016,0.476241896,-0.626650062],[-1,1,1,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
	
    CONST robtarget RightHomeInt4:=[[124.723921943,-1908.146618401,1528.631600537],[0.018360267,-0.835920966,0.442521729,-0.324150519],[-1,0,1,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
	CONST robtarget RightHomeInt3:=[[-273.629461112,-2045.541202358,1151.833820754],[0.025265439,-0.925055462,0.378730856,0.014035243],[-2,-1,1,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
	CONST robtarget RightHomeInt2:=[[-646.933119793,-2045.758260271,1088.021687357],[0.014533041,0.917780276,-0.15986647,-0.363195358],[-2,-1,1,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
	CONST robtarget RightHomeInt1:=[[-905.888258753,-1730.438708361,1168.658010435],[0.002317719,0.822769551,0.020438458,-0.568002785],[-2,-2,1,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
	CONST robtarget RightHome:=[[-923.675626367,-1280.770376731,1287.035178312],[0.001729858,-0.81123975,0.050895461,0.582491827],[-2,-2,1,0],[9E9,9E9,9E9,9E9,9E9,9E9]];

    !This is defined by the center of the working area, the nearest reach of the robot
    CONST robtarget RefTarget:=[[1098.263343424,-145.392716349,1876.132558611],[0.71281506,0.23070134,0.648579001,0.134226904],[-1,-2,1,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    
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
!
    !These are for setting their logical, boolean values in the logic tree at the end of this...
    !VAR bool MoveLtoRorRtoL;
    !VAR bool ToolPosonLeftSide;
    !VAR bool ToolPosLow;
    !VAR bool ToolPosNearPlateorFar;
    
    !VAR bool LeftToolPosFarFromPlate;
    !VAR bool RightToolPosFarFromPlate;

    VAR robtarget ToolRobtarget;

    !This is to try to move the robot out of the upper left or upper left areas away from the piece
    VAR robtarget UpperToolOffset;
    !

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
        TPWrite " Val`routeAssignedBeginning' in 'routeBeginningCalculator' = "+ValToStr(routeAssignedBeginning);
    EndProc

    PROC writeCheckAssignedBegin()
        TPWrite "Val `routeAssignedBeginning' in `routeToEitherHomeLorR' = "+ValToStr(routeAssignedBeginning);
    EndProc

    PROC writeBeginningOfRoute()
        TPWrite "Val `beginningOfRoute' in `routeToEitherHomeLorR' = "+ValToStr(routeAssignedBeginning);
    EndProc

    PROC writeTargetWaypointEnroute()
        TPWrite "Next Target Location Enroute = "+ValToStr(waypointEnroute);
    ENDPROC

    PROC writeReachedWaypointEnroute()
        TPWrite "Reached Location Enroute = "+ValToStr(waypointEnroute);
    EndProc
    
    PROC movingLeftToRight()
        TPWrite "Moving Left to Right";
    EndProc
    
    PROC movingRightToLeft()
        TPWrite "Moving Right to Left";
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
        ELSE 
        !Home_or_Plate=4 THEN
            routeAssignedEnd:=17;
            writeEndOfRouteTarget;
        !ELSE
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
        !writeCheckAssignedBegin;
        !writeBeginningOfRoute;
        !waypointEnroute := beginningOfRoute;
        !routeAssignedEnd
        
        endRoute:=routeAssignedEnd;

        IF routeAssignedBeginning<endRoute THEN
            
            waypointEnroute:=routeAssignedBeginning;
            !This says if moving from left to right....
            WHILE waypointEnroute<=endRoute DO
               ! movingLeftToRight;
                !targetWaypoint:=waypoints{waypointEnroute};
               ! writeTargetWaypointEnroute;
                MoveL waypoints{waypointEnroute},Medium,z100,tool0\WObj:=wobj0;
               ! writeReachedWaypointEnroute;
                waypointEnroute:=waypointEnroute+1;
            ENDWHILE

            !Right to Left!
        ELSE
            
            waypointEnroute:=routeAssignedBeginning;
            WHILE waypointEnroute>=endRoute DO
                !movingRightToLeft;
                !targetWaypoint:=waypoints{waypointEnroute};
                !writeTargetWaypointEnroute;
                MoveL waypoints{waypointEnroute},Medium,z100,tool0\WObj:=wobj0;
               ! writeReachedWaypointEnroute;
                waypointEnroute:=waypointEnroute-1;
            ENDWHILE

        ENDIF

    ENDPROC
    
    PROC routeBeginningCalculator()

        !IF MoveLtoRorRtoL THEN ! This is creating some sort of recursion because I am using the beginning value to calculate if I'm moveing from left ro write!
        IF ToolPos.y>RefPos.y  THEN 
            TPWRITE "Tool on Left Side";
            IF ToolPos.z<RefPos.z THEN
                TPWRITE "Tool on Left Side, Low";
                IF ToolPos.x >= LeftHomeOffsetPos.x THEN
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
                IF ToolPos.x >= LeftHomeOffsetPos.x THEN
                    TPWrite "Tool on Left, High, Far From Plate";
                    TPWRITE "This trys to move the tool to path using moveLs";
                    UpperToolOffset:=Offs(ToolRobtarget,10,-10,0);
                    MoveL UpperToolOffset,Medium,z100,tool0\WObj:=wobj0;
                    !routeBeginningCalculator;
                ELSE
                    TPWrite "Tool on Left, High, Near Plate";
                    routeAssignedBeginning:=7;
                    writeBeginCalcAssignedBegin;
                ENDIF
            ENDIF
        ELSE 
            TPWRITE "Tool on Right Side";
            IF ToolPos.z<RefPos.z THEN
                TPWRITE "Tool on Right Side, Low";
                IF ToolPos.x >= RightHomeOffsetPos.x THEN
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
                IF ToolPos.x >= RightHomeOffsetPos.x THEN
                    TPWRITE "Tool on Right Side, High, Far from Plate";
                    TPWRITE "This trys to move the tool to path using moveLs";
                    UpperToolOffset:=Offs(ToolRobtarget,10,-10,0);
                    MoveL UpperToolOffset,Medium,z100,tool0\WObj:=wobj0;
                    writeBeginCalcAssignedBegin;
                ELSE
                    TPWRITE "Tool on Right Side, High, Near Plate";
                    routeAssignedBeginning:=11;
                    writeBeginCalcAssignedBegin;
                ENDIF
            ENDIF
        ENDIF  
    ENDPROC

ENDMODULE