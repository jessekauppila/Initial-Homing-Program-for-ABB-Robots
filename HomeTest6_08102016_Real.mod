MODULE HomeLogicLists
	!L_Home_Off_Pos too long L_Home_Off_Pos
	!R_Home_Off_Pos too long R_Home_Off_Pos
	!L_or_R_Resp too long L_or_R_Resp
	!     too long L_or_R_Ans
	!Hm_or_Plt_Rsp too long Hm_or_Plt_Rsp
	!rtAsgndBegin
	!Left_or_Right_Answer 
	!Hm_or_Plt_Ans
	!DistToolHome
	!DistCntWrkHm  
	!DistCnt2PltHm
	!DistTlToPltHm

	
	
	
    !definetool and workobject?

    !Left Home Sequence 

    CONST speeddata Medium:=v100;
	
	  !------------------ Real ROBTARGETS are here------------------
	
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

    VAR num DistTLtoEoT;
    VAR num DistRTtoET;


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
            wrtEndORtTarg;
        ELSEIF Home_or_Plate=2 THEN
            Hm_or_Plt_Rsp:="the tool will go to Left Plate";
            !Hm_or_Plt_Ans:=2;
            routeAssignedEnd:=6;
            wrtEndORtTarg;
        ELSEIF Home_or_Plate=3 THEN
            Hm_or_Plt_Rsp:="";
        ELSEIF Home_or_Plate=4 THEN
            Hm_or_Plt_Rsp:="the tool will go to Right Plate";
            !Hm_or_Plt_Ans:=3;
            routeAssignedEnd:=12;
            wrtEndORtTarg;
        ELSE
            Hm_or_Plt_Rsp:="the tool will go to Right Home";
            ! Hm_or_Plt_Ans:=4;
            routeAssignedEnd:=17;
            wrtEndORtTarg;
        ENDIF
        !TPPrint "Ok, "+Hm_or_Plt_Rsp+"!\0D\0A";

        !whereToEndRoute;
        !sets routeAssignedEnd
        getPositionInfo;
        !calculates what side starting at and other values
        rtBeginCalc;
        !calculates where to begin rout...
        rtToHmLoR;
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
            wrtEndORtTarg;
        ELSEIF Home_or_Plate=2 THEN
            Hm_or_Plt_Rsp:="the tool will go to Left Plate";
            !Hm_or_Plt_Ans:=2;
            routeAssignedEnd:=6;
            wrtEndORtTarg;
        ELSEIF Home_or_Plate=3 THEN
            Hm_or_Plt_Rsp:="";
        ELSEIF Home_or_Plate=4 THEN
            Hm_or_Plt_Rsp:="the tool will go to Right Plate";
            !Hm_or_Plt_Ans:=3;
            routeAssignedEnd:=12;
            wrtEndORtTarg;
        ELSE
            Hm_or_Plt_Rsp:="the tool will go to Right Home";
            ! Hm_or_Plt_Ans:=4;
            routeAssignedEnd:=17;
            wrtEndORtTarg;
        ENDIF
        !TPPrint "Ok, "+Hm_or_Plt_Rsp+"!\0D\0A";

        !Moves tool to  Home!

        !IF Home_or_Plate=1 THEN
        ! routeAssignedEnd:=1;
        !wrtEndORtTarg;
        !ELSEIF Home_or_Plate=2 THEN
        ! routeAssignedEnd:=6;
        ! wrtEndORtTarg;
        !ELSEIF Home_or_Plate=3 THEN
        !   routeAssignedEnd:=12;
        !  wrtEndORtTarg;
        !ELSE
        !Home_or_Plate=4 THEN
        ! routeAssignedEnd:=17;
        ! wrtEndORtTarg;
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

        DistTLtoEoT:=Distance(ToolPos,EndTargetPos);
        DistRTtoET:=Distance(RefTargetPos,EndTargetPos);
    ENDPROC

    PROC rtToHmLoR()

        IF rtAsgndBegin<routeAssignedEnd THEN
            MoveLtoR;
            waypointEnroute:=rtAsgndBegin;
            !This says if moving from left to right....
            WHILE waypointEnroute<=routeAssignedEnd DO
                 
                !targetWaypoint:=waypoints{waypointEnroute};
                !wrtWyPtEnRt;
                MoveL waypoints{waypointEnroute},Medium,z100,tool0\WObj:=wobj0;
                !wrtRchdWyPt;
                waypointEnroute:=waypointEnroute+1;
            ENDWHILE

            !Right to Left!
        ELSE
            MoveLtoR;
            waypointEnroute:=rtAsgndBegin;
            WHILE waypointEnroute>=routeAssignedEnd DO
                
                !targetWaypoint:=waypoints{waypointEnroute};
                !wrtWyPtEnRt;
                MoveL waypoints{waypointEnroute},Medium,z100,tool0\WObj:=wobj0;
                ! wrtRchdWyPt;
                waypointEnroute:=waypointEnroute-1;
            ENDWHILE

        ENDIF

    ENDPROC

    PROC rtBeginCalc()

        !IF rtToHmLoRorRtoL THEN ! This is creating some sort of recursion because I am using the beginning value to calculate if I'm moveing from left ro write!
        IF ToolPos.y>RefTargetPos.y THEN
            TPWRITE "Tool on Left Side";
            IF ToolPos.z<RefTargetPos.z THEN
                TPWRITE "Tool on Left Side, Low";
                IF ToolPos.x>=L_Home_Off_Pos.x THEN
                    TPWrite "Tool on Left, Low, Far From Plate";
                    rtAsgndBegin:=6;
                    wrtRtAsgndBegin;
                ELSE
                    !LeftSideHigh
                    TPWrite "Tool on Left, Low, Near Plate";
                    rtAsgndBegin:=1;
                    wrtRtAsgndBegin;
                ENDIF
            ELSE
                TPWRITE "Tool on Left Side, High";
                WHILE ToolPos.x>=L_Home_Off_Pos.x DO
                    TPWrite "Tool on Left, High, Far From Plate";
                    TPWrite "This trys to move the tool to path using moveLs";
                    UpperToolOffset:=Offs(ToolRobtarget,50,50,0);
                    MoveL UpperToolOffset,Medium,z100,tool0\WObj:=wobj0;
                    getPositionInfo;
					!rtBeginCalc;
                    !wrtRtAsgndBegin;
				ENDWHILE

                TPWrite "Tool on Left, High, Near Plate";
                rtAsgndBegin:=7;
                wrtRtAsgndBegin;
                
            ENDIF
        ELSE
            TPWRITE "Tool on Right.";
            IF ToolPos.z<RefTargetPos.z THEN
                TPWRITE "Tool on Right Side, Low";
                IF ToolPos.x<=R_Home_Off_Pos.x THEN
                    TPWRITE "Tool on Right Side, Low, Far from Plate.";
                    rtAsgndBegin:=17;
                    wrtRtAsgndBegin;
                ELSE
                    TPWRITE "Tool on Right Side, Low, Near Plate.";
                    rtAsgndBegin:=12;
                    wrtRtAsgndBegin;
                ENDIF
            ELSE
                TPWRITE "Tool High";
                WHILE ToolPos.x<=R_Home_Off_Pos.x DO
                    TPWRITE "Tool on Right Side, Low, Far from Plate.";
                    TPWRITE "This trys to move the tool to path using moveLs";
                    UpperToolOffset:=Offs(ToolRobtarget,50,-50,0);
                    MoveL UpperToolOffset,Medium,z100,tool0\WObj:=wobj0;
                    getPositionInfo;
                    !wrtRtAsgndBegin;
				ENDWHILE
                
				TPWRITE "Tool on Right Side, Low, Near Plate.";
                rtAsgndBegin:=11;
				wrtRtAsgndBegin;
				
                
            ENDIF
        ENDIF
    ENDPROC

    !These Below are all error finding devices...
    PROC wrtEndORtTarg()
        TPWrite "End of Route Target = "+ValToStr(routeAssignedEnd);
    EndProc

    PROC wrtRtAsgndBegin()
        TPWrite " Val`rtAsgndBegin' in 'rtBeginCalc' = "+ValToStr(rtAsgndBegin);
    EndProc

    PROC wrtChkAssBgn()
        TPWrite "Val `rtAsgndBegin' in `rtToHmLoR' = "+ValToStr(rtAsgndBegin);
    EndProc

    PROC wrtBgnORt()
        TPWrite "Val `beginningOfRoute' in `rtToHmLoR' = "+ValToStr(rtAsgndBegin);
    EndProc

    PROC wrtWyPtEnRt()
        TPWrite "Next Target Location Enroute = "+ValToStr(waypointEnroute);
    ENDPROC

    PROC wrtRchdWyPt()
        TPWrite "Reached Location Enroute = "+ValToStr(waypointEnroute);
    EndProc
   
    PROC MoveLtoR()
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