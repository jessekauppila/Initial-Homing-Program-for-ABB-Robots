MODULE TestHomeCircuit

    CONST robtarget LeftHome:=[[-10.078042494,1824.668831942,802.431280345],[0.508337887,-0.501604166,0.508830774,0.480704791],[1,-2,1,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    CONST robtarget LeftLow:=[[689.414723237,1824.668910821,1159.28624599],[0.699324562,0.000122117,0.714800341,0.002369495],[0,-2,1,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    CONST robtarget LeftHigh:=[[689.415668582,1843.736962037,2261.41568216],[0.699324234,0.000122309,0.71480066,0.002369843],[0,-1,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    CONST robtarget LeftTop:=[[689.414975299,1053.174954768,2837.483396634],[0.69932429,0.000121762,0.714800608,0.002369449],[0,-1,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    CONST robtarget Top:=[[695.447397289,-80.906517867,3009.199597555],[0.789570357,-0.133382371,0.533031764,-0.273248848],[-1,0,-1,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    CONST robtarget RightTop:=[[930.843689193,-1249.46499759,2732.440078441],[0.699324741,0.000122762,0.714800168,0.002368753],[-1,0,-1,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    CONST robtarget RightHigh:=[[788.029576424,-2141.291550396,2089.025775678],[0.699324542,0.000122445,0.714800361,0.002369197],[-1,0,-1,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    CONST robtarget RightLow:=[[615.6264138,-1970.938238655,1091.520628553],[0.699324651,0.000122961,0.714800255,0.002369161],[-1,1,-2,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    CONST robtarget RightHome:=[[2.648006257,-1970.937085546,721.148890896],[0.506219517,0.495009538,0.515659034,-0.482496754],[-2,1,-2,0],[9E9,9E9,9E9,9E9,9E9,9E9]];

    PROC Main()

        homePath;

    ENDPROC



    PROC homePath()
        MoveL LeftHome,v1000,z100,tool0\WObj:=wobj0;
        MoveL LeftLow,v1000,z100,tool0\WObj:=wobj0;
        MoveL LeftHigh,v1000,z100,tool0\WObj:=wobj0;
        MoveL LeftTop,v1000,z100,tool0\WObj:=wobj0;
        MoveL Top,v1000,z100,tool0\WObj:=wobj0;
        MoveL RightTop,v1000,z100,tool0\WObj:=wobj0;
        MoveL RightHigh,v1000,z100,tool0\WObj:=wobj0;
        MoveL RightLow,v1000,z100,tool0\WObj:=wobj0;
        MoveL RightHome,v1000,z100,tool0\WObj:=wobj0;
        MoveL RightLow,v1000,z100,tool0\WObj:=wobj0;
        MoveL RightHigh,v1000,z100,tool0\WObj:=wobj0;
        MoveL RightTop,v1000,z100,tool0\WObj:=wobj0;
        MoveL Top,v1000,z100,tool0\WObj:=wobj0;
        MoveL LeftTop,v1000,z100,tool0\WObj:=wobj0;
        MoveL LeftHigh,v1000,z100,tool0\WObj:=wobj0;
        MoveL LeftLow,v1000,z100,tool0\WObj:=wobj0;
        MoveL LeftHome,v1000,z100,tool0\WObj:=wobj0;

    ENDPROC

ENDMODULE
