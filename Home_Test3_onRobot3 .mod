MODULE HomeLogicLists
	!L_Home_Off_Pos too long L_Home_Off_Pos
	!R_Home_Off_Pos too long R_Home_Off_Pos
	!L_or_R_Resp too long L_or_R_Resp
	!L_or_R_Ans too long L_or_R_Ans
	!Hm_or_Plt_Rsp too long Hm_or_Plt_Rsp
	!rtAsgndBegin
	!L_or_R_Ans 
	!Hm_or_Plt_Ans
	!DistToolHome
	!DistCntWrkHm
	!DistCnt2PltHm
	!DistTlToPltHm

	
	
	
    !definetool and workobject?

    !Left Home Sequence 

    CONST speeddata Medium:=v2000;

    !------------------ROBTARGETS are here------------------
    
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
    
    CONST robtarget RightHomeInt4:=[[356.431358218,-1834.956540582,842.785759123],[0.007765182,0.893437216,-0.426866418,0.139623437],[-1,0,1,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
	CONST robtarget RightHomeInt3:=[[66.573358057,-2096.138894919,881.711829832],[0.151417366,0.97932588,-0.100505266,-0.088838586],[-1,-1,1,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
	CONST robtarget RightHomeInt2:=[[-266.699991395,-2099.430293126,934.064600728],[0.18235385,0.954073205,0.042266151,-0.233890925],[-2,-1,1,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
	CONST robtarget RightHomeInt1:=[[-646.291931243,-2023.194070237,1026.021179395],[0.036760555,0.814200579,0.138665346,-0.56258155],[-2,-2,1,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
	CONST robtarget RightHome:=[[-830.106681529,-1786.23065868,1110.614894364],[0.011624349,0.818706496,0.08359959,-0.567975049],[-2,-2,1,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    
    
    
    
    !This is defined by the center of the working area, the nearest reach of the robot
    CONST robtarget RefTarget:=[[1098.263343424,-145.392716349,1876.132558611],[0.71281506,0.23070134,0.648579001,0.134226904],[-1,-2,1,0],[9E9,9E9,9E9,9E9,9E9,9E9]];



    !-------------------- POS's ARE HERE ----------------------------------

    !These "pos" are used to find where the 

    VAR robtarget ToolRobtarget;
    VAR pos ToolPos;

    VAR pos LeftHomePos;
    VAR pos RightHomePos;

    VAR pos L_Home_Off_Pos;
    VAR pos R_Home_Off_Pos;

    VAR robtarget LeftHomeOffset;
    VAR robtarget RightHomeOffset;

    VAR pos EndTargetPos;

    VAR pos RefTargetPos:=RefTarget.trans;

    VAR num i;
    VAR num waypointEnroute;

    !These are chosen evertime in the whereToEndRoute Proc
    VAR num rtAsgndBegin;
    VAR num routeAssignedEnd;

    !This is the Waypointlist...
    VAR robtarget waypoints{numWayPoints};
    CONST num numWayPoints:=17;

    Var robtarget EndTarget;

    !These are the variables which need to be initiated to run the TPreadFK for left or right side choice
    VAR num Left_or_Right:=-1;
    VAR string L_or_R_Resp:="";
    VAR num L_or_R_Ans:=0;

    !These are the variables which need to be initiated to run the TPreadFK for Home or Plate choice
    VAR num Home_or_Plate:=-1;
    VAR string Hm_or_Plt_Rsp:="";
    VAR num Hm_or_Plt_Ans:=0;

    !These are the VAR for getPositionInfo

    !instantiate variables to use in this proc...   
    !maybe break these down further...
    !!

    VAR NUM DistToolHome;
    VAR NUM DistCntWrkHm;
    VAR NUM DistTlToPltHm;
    VAR NUM DistCnt2PltHm;

    VAR num DistToolLocToEndTarget;
    VAR num DistRefTargetToEndTarget;


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
            Hm_or_Plt_Rsp:="the tool will go to Left Home";
            !Hm_or_Plt_Ans:=1;
            routeAssignedEnd:=1;
            writeEndOfRouteTarget;
        ELSEIF Home_or_Plate=2 THEN
            Hm_or_Plt_Rsp:="the tool will go to Left Plate";
            !Hm_or_Plt_Ans:=2;
            routeAssignedEnd:=6;
            writeEndOfRouteTarget;
        ELSEIF Home_or_Plate=3 THEN
            Hm_or_Plt_Rsp:="";
        ELSEIF Home_or_Plate=4 THEN
            Hm_or_Plt_Rsp:="the tool will go to Right Plate";
            !Hm_or_Plt_Ans:=3;
            routeAssignedEnd:=12;
            writeEndOfRouteTarget;
        ELSE
            Hm_or_Plt_Rsp:="the tool will go to Right Home";
            ! Hm_or_Plt_Ans:=4;
            routeAssignedEnd:=17;
            writeEndOfRouteTarget;
        ENDIF
        TPPrint "Ok, "+Hm_or_Plt_Rsp+"!\0D\0A";

        !whereToEndRoute;
        !sets routeAssignedEnd
        getPositionInfo;
        !calculates what side starting at and other values
        routeBeginningCalculator;
        !calculates where to begin rout...
        routeToEitherHomeLorR;
        !I took these parameters out to see if it might, just might run!rtAsgndBegin,routeAssignedEnd;
        !
    ENDPROC



    !These above are all error finding devices...!!!

    PROC whereToEndRoute()

        TPReadFK Home_or_Plate,"Where do you want to move the tool?","Left Home","Left Plate","","Right Plate","Right Home";
        IF Home_or_Plate=1 THEN
            Hm_or_Plt_Rsp:="the tool will go to Left Home";
            !Hm_or_Plt_Ans:=1;
            routeAssignedEnd:=1;
            writeEndOfRouteTarget;
        ELSEIF Home_or_Plate=2 THEN
            Hm_or_Plt_Rsp:="the tool will go to Left Plate";
            !Hm_or_Plt_Ans:=2;
            routeAssignedEnd:=6;
            writeEndOfRouteTarget;
        ELSEIF Home_or_Plate=3 THEN
            Hm_or_Plt_Rsp:="";
        ELSEIF Home_or_Plate=4 THEN
            Hm_or_Plt_Rsp:="the tool will go to Right Plate";
            !Hm_or_Plt_Ans:=3;
            routeAssignedEnd:=12;
            writeEndOfRouteTarget;
        ELSE
            Hm_or_Plt_Rsp:="the tool will go to Right Home";
            ! Hm_or_Plt_Ans:=4;
            routeAssignedEnd:=17;
            writeEndOfRouteTarget;
        ENDIF
        TPPrint "Ok, "+Hm_or_Plt_Rsp+"!\0D\0A";

        !Moves tool to  Home!

        !IF Home_or_Plate=1 THEN
        ! routeAssignedEnd:=1;
        !writeEndOfRouteTarget;
        !ELSEIF Home_or_Plate=2 THEN
        ! routeAssignedEnd:=6;
        ! writeEndOfRouteTarget;
        !ELSEIF Home_or_Plate=3 THEN
        !   routeAssignedEnd:=12;
        !  writeEndOfRouteTarget;
        !ELSE
        !Home_or_Plate=4 THEN
        ! routeAssignedEnd:=17;
        ! writeEndOfRouteTarget;
        !ELSE
        !ENDIF


    ENDPROC

    !
    PROC getPositionInfo()


        ToolRobtarget:=CRobT(\Tool:=tool0\WObj:=wobj0);
        ToolPos:=ToolRobtarget.trans;

        LeftHomePos:=LeftHome.trans;
        RightHomePos:=RightHome.trans;

        LeftHomeOffset:=Offs(LeftHome,50,0,0);
        RightHomeOffset:=Offs(RightHome,50,0,0);

        L_Home_Off_Pos:=LeftHomeOffset.trans;
        R_Home_Off_Pos:=RightHomeOffset.trans;

        RefTargetPos:=RefTarget.trans;

        EndTarget:=waypoints{routeAssignedEnd};
        EndTargetPos:=EndTarget.trans;

        DistToolLocToEndTarget:=Distance(ToolPos,EndTargetPos);
        DistRefTargetToEndTarget:=Distance(RefTargetPos,EndTargetPos);
    ENDPROC

    PROC routeToEitherHomeLorR()

        IF rtAsgndBegin<routeAssignedEnd THEN
            movingLeftToRight;
            waypointEnroute:=rtAsgndBegin;
            !This says if moving from left to right....
            WHILE waypointEnroute<=routeAssignedEnd DO
                 
                !targetWaypoint:=waypoints{waypointEnroute};
                !writeTargetWaypointEnroute;
                MoveL waypoints{waypointEnroute},Medium,z100,tool0\WObj:=wobj0;
                !writeReachedWaypointEnroute;
                waypointEnroute:=waypointEnroute+1;
            ENDWHILE

            !Right to Left!
        ELSE
            movingRightToLeft;
            waypointEnroute:=rtAsgndBegin;
            WHILE waypointEnroute>=routeAssignedEnd DO
                
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
        IF ToolPos.y>RefTargetPos.y THEN
            TPWRITE "Tool on Left Side";
            IF ToolPos.z<RefTargetPos.z THEN
                TPWRITE "Tool on Left Side, Low";
                IF ToolPos.x>=L_Home_Off_Pos.x THEN
                    TPWrite "Tool on Left, Low, Far From Plate";
                    rtAsgndBegin:=6;
                    writeBeginCalcAssignedBegin;
                ELSE
                    !LeftSideHigh
                    TPWrite "Tool on Left, Low, Near Plate";
                    rtAsgndBegin:=1;
                    writeBeginCalcAssignedBegin;
                ENDIF
            ELSE
                TPWRITE "Tool on Left Side, High";
                IF ToolPos.x>=L_Home_Off_Pos.x THEN
                    TPWrite "Tool on Left, High, Far From Plate";
                    TPWRITE "This trys to move the tool to path using moveLs";
                    UpperToolOffset:=Offs(ToolRobtarget,50,50,0);
                    MoveL UpperToolOffset,Medium,z100,tool0\WObj:=wobj0;
                    getPositionInfo;
                    routeBeginningCalculator;
                ELSE
                    TPWrite "Tool on Left, High, Near Plate";
                    rtAsgndBegin:=7;
                    writeBeginCalcAssignedBegin;
                ENDIF
            ENDIF
        ELSE
            TPWRITE "Tool on Right...";
            IF ToolPos.z<RefTargetPos.z THEN
                TPWRITE "........Low";
                IF ToolPos.x<=R_Home_Off_Pos.x THEN
                    TPWRITE "............Far from Plate.";
                    rtAsgndBegin:=17;
                    writeBeginCalcAssignedBegin;
                ELSE
                    TPWRITE "............Near Plate.";
                    rtAsgndBegin:=12;
                    writeBeginCalcAssignedBegin;
                ENDIF
            ELSE
                TPWRITE "High";
                IF ToolPos.x<=R_Home_Off_Pos.x THEN
                    TPWRITE "..............Far from Plate.";
                    TPWRITE "This trys to move the tool to path using moveLs";
                    UpperToolOffset:=Offs(ToolRobtarget,50,-50,0);
                    MoveL UpperToolOffset,Medium,z100,tool0\WObj:=wobj0;
                    getPositionInfo;
                    routeBeginningCalculator;
                    writeBeginCalcAssignedBegin;
                ELSE
                    TPWRITE ".............Near Plate.";
                    rtAsgndBegin:=11;
                    writeBeginCalcAssignedBegin;
                ENDIF
            ENDIF
        ENDIF
    ENDPROC

    !These Below are all error finding devices...
    PROC writeEndOfRouteTarget()
        TPWrite "End of Route Target = "+ValToStr(routeAssignedEnd);
    EndProc

    PROC writeBeginCalcAssignedBegin()
        TPWrite " Val`rtAsgndBegin' in 'routeBeginningCalculator' = "+ValToStr(rtAsgndBegin);
    EndProc

    PROC writeCheckAssignedBegin()
        TPWrite "Val `rtAsgndBegin' in `routeToEitherHomeLorR' = "+ValToStr(rtAsgndBegin);
    EndProc

    PROC writeBeginningOfRoute()
        TPWrite "Val `beginningOfRoute' in `routeToEitherHomeLorR' = "+ValToStr(rtAsgndBegin);
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

    PROC HomePath2()
        MoveL RightHigh,v1000,z100,Tooldata_1\WObj:=wobj0;
        MoveL RightLow,v1000,z100,Tooldata_1\WObj:=wobj0;
        MoveL RightHomeInt4,v1000,z100,Tooldata_1\WObj:=wobj0;
        MoveL RightHomeInt3,v1000,z100,Tooldata_1\WObj:=wobj0;
        MoveL RightHomeInt2,v1000,z100,Tooldata_1\WObj:=wobj0;
        MoveL RightHomeInt1,v1000,z100,Tooldata_1\WObj:=wobj0;
        MoveL RightHome,v1000,z100,Tooldata_1\WObj:=wobj0;
    ENDPROC

ENDMODULE