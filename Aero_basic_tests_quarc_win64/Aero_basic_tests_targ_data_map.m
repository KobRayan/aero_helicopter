    function targMap = targDataMap(),

    ;%***********************
    ;% Create Parameter Map *
    ;%***********************
    
        nTotData      = 0; %add to this count as we go
        nTotSects     = 6;
        sectIdxOffset = 0;

        ;%
        ;% Define dummy sections & preallocate arrays
        ;%
        dumSection.nData = -1;
        dumSection.data  = [];

        dumData.logicalSrcIdx = -1;
        dumData.dtTransOffset = -1;

        ;%
        ;% Init/prealloc paramMap
        ;%
        paramMap.nSections           = nTotSects;
        paramMap.sectIdxOffset       = sectIdxOffset;
            paramMap.sections(nTotSects) = dumSection; %prealloc
        paramMap.nTotData            = -1;

        ;%
        ;% Auto data (Aero_basic_tests_P)
        ;%
            section.nData     = 6;
            section.data(6)  = dumData; %prealloc

                    ;% Aero_basic_tests_P.BaseLEDColour_color
                    section.data(1).logicalSrcIdx = 0;
                    section.data(1).dtTransOffset = 0;

                    ;% Aero_basic_tests_P.BiasRemoval_end_time
                    section.data(2).logicalSrcIdx = 1;
                    section.data(2).dtTransOffset = 3;

                    ;% Aero_basic_tests_P.SecondOrderLowPassFilter_input_
                    section.data(3).logicalSrcIdx = 2;
                    section.data(3).dtTransOffset = 4;

                    ;% Aero_basic_tests_P.SecondOrderLowPassFilter_inpu_m
                    section.data(4).logicalSrcIdx = 3;
                    section.data(4).dtTransOffset = 5;

                    ;% Aero_basic_tests_P.BiasRemoval_start_time
                    section.data(5).logicalSrcIdx = 4;
                    section.data(5).dtTransOffset = 6;

                    ;% Aero_basic_tests_P.BiasRemoval_switch_id
                    section.data(6).logicalSrcIdx = 5;
                    section.data(6).dtTransOffset = 7;

            nTotData = nTotData + section.nData;
            paramMap.sections(1) = section;
            clear section

            section.nData     = 6;
            section.data(6)  = dumData; %prealloc

                    ;% Aero_basic_tests_P.HILRead_analog_channels
                    section.data(1).logicalSrcIdx = 6;
                    section.data(1).dtTransOffset = 0;

                    ;% Aero_basic_tests_P.HILWrite_analog_channels
                    section.data(2).logicalSrcIdx = 7;
                    section.data(2).dtTransOffset = 2;

                    ;% Aero_basic_tests_P.HILWrite_digital_channels
                    section.data(3).logicalSrcIdx = 8;
                    section.data(3).dtTransOffset = 4;

                    ;% Aero_basic_tests_P.HILRead_encoder_channels
                    section.data(4).logicalSrcIdx = 9;
                    section.data(4).dtTransOffset = 6;

                    ;% Aero_basic_tests_P.HILRead_other_channels
                    section.data(5).logicalSrcIdx = 10;
                    section.data(5).dtTransOffset = 10;

                    ;% Aero_basic_tests_P.HILWrite_other_channels
                    section.data(6).logicalSrcIdx = 11;
                    section.data(6).dtTransOffset = 18;

            nTotData = nTotData + section.nData;
            paramMap.sections(2) = section;
            clear section

            section.nData     = 42;
            section.data(42)  = dumData; %prealloc

                    ;% Aero_basic_tests_P.x_avg_n_Y0
                    section.data(1).logicalSrcIdx = 12;
                    section.data(1).dtTransOffset = 0;

                    ;% Aero_basic_tests_P.unity_Value
                    section.data(2).logicalSrcIdx = 13;
                    section.data(2).dtTransOffset = 1;

                    ;% Aero_basic_tests_P.UnitDelay_InitialCondition
                    section.data(3).logicalSrcIdx = 14;
                    section.data(3).dtTransOffset = 2;

                    ;% Aero_basic_tests_P.Sumk1n1xk_InitialCondition
                    section.data(4).logicalSrcIdx = 15;
                    section.data(4).dtTransOffset = 3;

                    ;% Aero_basic_tests_P.zero_Y0
                    section.data(5).logicalSrcIdx = 16;
                    section.data(5).dtTransOffset = 4;

                    ;% Aero_basic_tests_P.Constant_Value
                    section.data(6).logicalSrcIdx = 17;
                    section.data(6).dtTransOffset = 5;

                    ;% Aero_basic_tests_P.Vbiased_Y0
                    section.data(7).logicalSrcIdx = 18;
                    section.data(7).dtTransOffset = 6;

                    ;% Aero_basic_tests_P.Vunbiased_Y0
                    section.data(8).logicalSrcIdx = 19;
                    section.data(8).dtTransOffset = 7;

                    ;% Aero_basic_tests_P.Stepstart_time_Y0
                    section.data(9).logicalSrcIdx = 20;
                    section.data(9).dtTransOffset = 8;

                    ;% Aero_basic_tests_P.Stepstart_time_YFinal
                    section.data(10).logicalSrcIdx = 21;
                    section.data(10).dtTransOffset = 9;

                    ;% Aero_basic_tests_P.Stepend_time_Y0
                    section.data(11).logicalSrcIdx = 22;
                    section.data(11).dtTransOffset = 10;

                    ;% Aero_basic_tests_P.Stepend_time_YFinal
                    section.data(12).logicalSrcIdx = 23;
                    section.data(12).dtTransOffset = 11;

                    ;% Aero_basic_tests_P.HILInitialize_OOTerminate
                    section.data(13).logicalSrcIdx = 24;
                    section.data(13).dtTransOffset = 12;

                    ;% Aero_basic_tests_P.HILInitialize_OOExit
                    section.data(14).logicalSrcIdx = 25;
                    section.data(14).dtTransOffset = 13;

                    ;% Aero_basic_tests_P.HILInitialize_OOStart
                    section.data(15).logicalSrcIdx = 26;
                    section.data(15).dtTransOffset = 14;

                    ;% Aero_basic_tests_P.HILInitialize_OOEnter
                    section.data(16).logicalSrcIdx = 27;
                    section.data(16).dtTransOffset = 15;

                    ;% Aero_basic_tests_P.HILInitialize_AOFinal
                    section.data(17).logicalSrcIdx = 28;
                    section.data(17).dtTransOffset = 16;

                    ;% Aero_basic_tests_P.HILInitialize_OOFinal
                    section.data(18).logicalSrcIdx = 29;
                    section.data(18).dtTransOffset = 17;

                    ;% Aero_basic_tests_P.HILInitialize_AIHigh
                    section.data(19).logicalSrcIdx = 30;
                    section.data(19).dtTransOffset = 20;

                    ;% Aero_basic_tests_P.HILInitialize_AILow
                    section.data(20).logicalSrcIdx = 31;
                    section.data(20).dtTransOffset = 21;

                    ;% Aero_basic_tests_P.HILInitialize_AOHigh
                    section.data(21).logicalSrcIdx = 32;
                    section.data(21).dtTransOffset = 22;

                    ;% Aero_basic_tests_P.HILInitialize_AOLow
                    section.data(22).logicalSrcIdx = 33;
                    section.data(22).dtTransOffset = 23;

                    ;% Aero_basic_tests_P.HILInitialize_AOInitial
                    section.data(23).logicalSrcIdx = 34;
                    section.data(23).dtTransOffset = 24;

                    ;% Aero_basic_tests_P.HILInitialize_AOWatchdog
                    section.data(24).logicalSrcIdx = 35;
                    section.data(24).dtTransOffset = 25;

                    ;% Aero_basic_tests_P.HILInitialize_OOInitial
                    section.data(25).logicalSrcIdx = 36;
                    section.data(25).dtTransOffset = 26;

                    ;% Aero_basic_tests_P.HILInitialize_OOWatchdog
                    section.data(26).logicalSrcIdx = 37;
                    section.data(26).dtTransOffset = 29;

                    ;% Aero_basic_tests_P.x0_Value
                    section.data(27).logicalSrcIdx = 38;
                    section.data(27).dtTransOffset = 32;

                    ;% Aero_basic_tests_P.Pitchcountstodeg_Gain
                    section.data(28).logicalSrcIdx = 39;
                    section.data(28).dtTransOffset = 33;

                    ;% Aero_basic_tests_P.Yawcountstodeg_Gain
                    section.data(29).logicalSrcIdx = 40;
                    section.data(29).dtTransOffset = 34;

                    ;% Aero_basic_tests_P.GyroCal_Gain
                    section.data(30).logicalSrcIdx = 41;
                    section.data(30).dtTransOffset = 35;

                    ;% Aero_basic_tests_P.countsstorads_Gain
                    section.data(31).logicalSrcIdx = 42;
                    section.data(31).dtTransOffset = 38;

                    ;% Aero_basic_tests_P.countsstorads1_Gain
                    section.data(32).logicalSrcIdx = 43;
                    section.data(32).dtTransOffset = 39;

                    ;% Aero_basic_tests_P.Constant_Value_m
                    section.data(33).logicalSrcIdx = 44;
                    section.data(33).dtTransOffset = 40;

                    ;% Aero_basic_tests_P.Integrator2_IC
                    section.data(34).logicalSrcIdx = 45;
                    section.data(34).dtTransOffset = 41;

                    ;% Aero_basic_tests_P.PitchCmdV_Value
                    section.data(35).logicalSrcIdx = 46;
                    section.data(35).dtTransOffset = 42;

                    ;% Aero_basic_tests_P.P24V_UpperSat
                    section.data(36).logicalSrcIdx = 47;
                    section.data(36).dtTransOffset = 43;

                    ;% Aero_basic_tests_P.P24V_LowerSat
                    section.data(37).logicalSrcIdx = 48;
                    section.data(37).dtTransOffset = 44;

                    ;% Aero_basic_tests_P.YawCmdV_Value
                    section.data(38).logicalSrcIdx = 49;
                    section.data(38).dtTransOffset = 45;

                    ;% Aero_basic_tests_P.Y24V_UpperSat
                    section.data(39).logicalSrcIdx = 50;
                    section.data(39).dtTransOffset = 46;

                    ;% Aero_basic_tests_P.Y24V_LowerSat
                    section.data(40).logicalSrcIdx = 51;
                    section.data(40).dtTransOffset = 47;

                    ;% Aero_basic_tests_P.PitchMotorEnable_Value
                    section.data(41).logicalSrcIdx = 52;
                    section.data(41).dtTransOffset = 48;

                    ;% Aero_basic_tests_P.YawMotorEnable_Value
                    section.data(42).logicalSrcIdx = 53;
                    section.data(42).dtTransOffset = 49;

            nTotData = nTotData + section.nData;
            paramMap.sections(3) = section;
            clear section

            section.nData     = 3;
            section.data(3)  = dumData; %prealloc

                    ;% Aero_basic_tests_P.HILInitialize_CKChannels
                    section.data(1).logicalSrcIdx = 54;
                    section.data(1).dtTransOffset = 0;

                    ;% Aero_basic_tests_P.HILInitialize_DOWatchdog
                    section.data(2).logicalSrcIdx = 55;
                    section.data(2).dtTransOffset = 1;

                    ;% Aero_basic_tests_P.HILInitialize_EIInitial
                    section.data(3).logicalSrcIdx = 56;
                    section.data(3).dtTransOffset = 2;

            nTotData = nTotData + section.nData;
            paramMap.sections(4) = section;
            clear section

            section.nData     = 6;
            section.data(6)  = dumData; %prealloc

                    ;% Aero_basic_tests_P.HILInitialize_AIChannels
                    section.data(1).logicalSrcIdx = 57;
                    section.data(1).dtTransOffset = 0;

                    ;% Aero_basic_tests_P.HILInitialize_AOChannels
                    section.data(2).logicalSrcIdx = 58;
                    section.data(2).dtTransOffset = 2;

                    ;% Aero_basic_tests_P.HILInitialize_DOChannels
                    section.data(3).logicalSrcIdx = 59;
                    section.data(3).dtTransOffset = 4;

                    ;% Aero_basic_tests_P.HILInitialize_EIChannels
                    section.data(4).logicalSrcIdx = 60;
                    section.data(4).dtTransOffset = 6;

                    ;% Aero_basic_tests_P.HILInitialize_EIQuadrature
                    section.data(5).logicalSrcIdx = 61;
                    section.data(5).dtTransOffset = 10;

                    ;% Aero_basic_tests_P.HILInitialize_OOChannels
                    section.data(6).logicalSrcIdx = 62;
                    section.data(6).dtTransOffset = 11;

            nTotData = nTotData + section.nData;
            paramMap.sections(5) = section;
            clear section

            section.nData     = 37;
            section.data(37)  = dumData; %prealloc

                    ;% Aero_basic_tests_P.HILInitialize_Active
                    section.data(1).logicalSrcIdx = 63;
                    section.data(1).dtTransOffset = 0;

                    ;% Aero_basic_tests_P.HILInitialize_AOTerminate
                    section.data(2).logicalSrcIdx = 64;
                    section.data(2).dtTransOffset = 1;

                    ;% Aero_basic_tests_P.HILInitialize_AOExit
                    section.data(3).logicalSrcIdx = 65;
                    section.data(3).dtTransOffset = 2;

                    ;% Aero_basic_tests_P.HILInitialize_DOTerminate
                    section.data(4).logicalSrcIdx = 66;
                    section.data(4).dtTransOffset = 3;

                    ;% Aero_basic_tests_P.HILInitialize_DOExit
                    section.data(5).logicalSrcIdx = 67;
                    section.data(5).dtTransOffset = 4;

                    ;% Aero_basic_tests_P.HILInitialize_POTerminate
                    section.data(6).logicalSrcIdx = 68;
                    section.data(6).dtTransOffset = 5;

                    ;% Aero_basic_tests_P.HILInitialize_POExit
                    section.data(7).logicalSrcIdx = 69;
                    section.data(7).dtTransOffset = 6;

                    ;% Aero_basic_tests_P.HILInitialize_CKPStart
                    section.data(8).logicalSrcIdx = 70;
                    section.data(8).dtTransOffset = 7;

                    ;% Aero_basic_tests_P.HILInitialize_CKPEnter
                    section.data(9).logicalSrcIdx = 71;
                    section.data(9).dtTransOffset = 8;

                    ;% Aero_basic_tests_P.HILInitialize_CKStart
                    section.data(10).logicalSrcIdx = 72;
                    section.data(10).dtTransOffset = 9;

                    ;% Aero_basic_tests_P.HILInitialize_CKEnter
                    section.data(11).logicalSrcIdx = 73;
                    section.data(11).dtTransOffset = 10;

                    ;% Aero_basic_tests_P.HILInitialize_AIPStart
                    section.data(12).logicalSrcIdx = 74;
                    section.data(12).dtTransOffset = 11;

                    ;% Aero_basic_tests_P.HILInitialize_AIPEnter
                    section.data(13).logicalSrcIdx = 75;
                    section.data(13).dtTransOffset = 12;

                    ;% Aero_basic_tests_P.HILInitialize_AOPStart
                    section.data(14).logicalSrcIdx = 76;
                    section.data(14).dtTransOffset = 13;

                    ;% Aero_basic_tests_P.HILInitialize_AOPEnter
                    section.data(15).logicalSrcIdx = 77;
                    section.data(15).dtTransOffset = 14;

                    ;% Aero_basic_tests_P.HILInitialize_AOStart
                    section.data(16).logicalSrcIdx = 78;
                    section.data(16).dtTransOffset = 15;

                    ;% Aero_basic_tests_P.HILInitialize_AOEnter
                    section.data(17).logicalSrcIdx = 79;
                    section.data(17).dtTransOffset = 16;

                    ;% Aero_basic_tests_P.HILInitialize_AOReset
                    section.data(18).logicalSrcIdx = 80;
                    section.data(18).dtTransOffset = 17;

                    ;% Aero_basic_tests_P.HILInitialize_DOPStart
                    section.data(19).logicalSrcIdx = 81;
                    section.data(19).dtTransOffset = 18;

                    ;% Aero_basic_tests_P.HILInitialize_DOPEnter
                    section.data(20).logicalSrcIdx = 82;
                    section.data(20).dtTransOffset = 19;

                    ;% Aero_basic_tests_P.HILInitialize_DOStart
                    section.data(21).logicalSrcIdx = 83;
                    section.data(21).dtTransOffset = 20;

                    ;% Aero_basic_tests_P.HILInitialize_DOEnter
                    section.data(22).logicalSrcIdx = 84;
                    section.data(22).dtTransOffset = 21;

                    ;% Aero_basic_tests_P.HILInitialize_DOReset
                    section.data(23).logicalSrcIdx = 85;
                    section.data(23).dtTransOffset = 22;

                    ;% Aero_basic_tests_P.HILInitialize_EIPStart
                    section.data(24).logicalSrcIdx = 86;
                    section.data(24).dtTransOffset = 23;

                    ;% Aero_basic_tests_P.HILInitialize_EIPEnter
                    section.data(25).logicalSrcIdx = 87;
                    section.data(25).dtTransOffset = 24;

                    ;% Aero_basic_tests_P.HILInitialize_EIStart
                    section.data(26).logicalSrcIdx = 88;
                    section.data(26).dtTransOffset = 25;

                    ;% Aero_basic_tests_P.HILInitialize_EIEnter
                    section.data(27).logicalSrcIdx = 89;
                    section.data(27).dtTransOffset = 26;

                    ;% Aero_basic_tests_P.HILInitialize_POPStart
                    section.data(28).logicalSrcIdx = 90;
                    section.data(28).dtTransOffset = 27;

                    ;% Aero_basic_tests_P.HILInitialize_POPEnter
                    section.data(29).logicalSrcIdx = 91;
                    section.data(29).dtTransOffset = 28;

                    ;% Aero_basic_tests_P.HILInitialize_POStart
                    section.data(30).logicalSrcIdx = 92;
                    section.data(30).dtTransOffset = 29;

                    ;% Aero_basic_tests_P.HILInitialize_POEnter
                    section.data(31).logicalSrcIdx = 93;
                    section.data(31).dtTransOffset = 30;

                    ;% Aero_basic_tests_P.HILInitialize_POReset
                    section.data(32).logicalSrcIdx = 94;
                    section.data(32).dtTransOffset = 31;

                    ;% Aero_basic_tests_P.HILInitialize_OOReset
                    section.data(33).logicalSrcIdx = 95;
                    section.data(33).dtTransOffset = 32;

                    ;% Aero_basic_tests_P.HILInitialize_DOFinal
                    section.data(34).logicalSrcIdx = 96;
                    section.data(34).dtTransOffset = 33;

                    ;% Aero_basic_tests_P.HILInitialize_DOInitial
                    section.data(35).logicalSrcIdx = 97;
                    section.data(35).dtTransOffset = 34;

                    ;% Aero_basic_tests_P.HILRead_Active
                    section.data(36).logicalSrcIdx = 98;
                    section.data(36).dtTransOffset = 35;

                    ;% Aero_basic_tests_P.HILWrite_Active
                    section.data(37).logicalSrcIdx = 99;
                    section.data(37).dtTransOffset = 36;

            nTotData = nTotData + section.nData;
            paramMap.sections(6) = section;
            clear section


            ;%
            ;% Non-auto Data (parameter)
            ;%


        ;%
        ;% Add final counts to struct.
        ;%
        paramMap.nTotData = nTotData;



    ;%**************************
    ;% Create Block Output Map *
    ;%**************************
    
        nTotData      = 0; %add to this count as we go
        nTotSects     = 1;
        sectIdxOffset = 0;

        ;%
        ;% Define dummy sections & preallocate arrays
        ;%
        dumSection.nData = -1;
        dumSection.data  = [];

        dumData.logicalSrcIdx = -1;
        dumData.dtTransOffset = -1;

        ;%
        ;% Init/prealloc sigMap
        ;%
        sigMap.nSections           = nTotSects;
        sigMap.sectIdxOffset       = sectIdxOffset;
            sigMap.sections(nTotSects) = dumSection; %prealloc
        sigMap.nTotData            = -1;

        ;%
        ;% Auto data (Aero_basic_tests_B)
        ;%
            section.nData     = 22;
            section.data(22)  = dumData; %prealloc

                    ;% Aero_basic_tests_B.x0
                    section.data(1).logicalSrcIdx = 0;
                    section.data(1).dtTransOffset = 0;

                    ;% Aero_basic_tests_B.Integrator1
                    section.data(2).logicalSrcIdx = 1;
                    section.data(2).dtTransOffset = 1;

                    ;% Aero_basic_tests_B.HILRead_o1
                    section.data(3).logicalSrcIdx = 2;
                    section.data(3).dtTransOffset = 4;

                    ;% Aero_basic_tests_B.HILRead_o2
                    section.data(4).logicalSrcIdx = 3;
                    section.data(4).dtTransOffset = 5;

                    ;% Aero_basic_tests_B.HILRead_o3
                    section.data(5).logicalSrcIdx = 4;
                    section.data(5).dtTransOffset = 6;

                    ;% Aero_basic_tests_B.HILRead_o4
                    section.data(6).logicalSrcIdx = 5;
                    section.data(6).dtTransOffset = 7;

                    ;% Aero_basic_tests_B.Pitchcountstodeg
                    section.data(7).logicalSrcIdx = 6;
                    section.data(7).dtTransOffset = 8;

                    ;% Aero_basic_tests_B.Yawcountstodeg
                    section.data(8).logicalSrcIdx = 7;
                    section.data(8).dtTransOffset = 9;

                    ;% Aero_basic_tests_B.GyroCal
                    section.data(9).logicalSrcIdx = 8;
                    section.data(9).dtTransOffset = 10;

                    ;% Aero_basic_tests_B.countsstorads
                    section.data(10).logicalSrcIdx = 9;
                    section.data(10).dtTransOffset = 13;

                    ;% Aero_basic_tests_B.countsstorads1
                    section.data(11).logicalSrcIdx = 10;
                    section.data(11).dtTransOffset = 14;

                    ;% Aero_basic_tests_B.Product
                    section.data(12).logicalSrcIdx = 11;
                    section.data(12).dtTransOffset = 15;

                    ;% Aero_basic_tests_B.Product1
                    section.data(13).logicalSrcIdx = 12;
                    section.data(13).dtTransOffset = 18;

                    ;% Aero_basic_tests_B.P24V
                    section.data(14).logicalSrcIdx = 13;
                    section.data(14).dtTransOffset = 21;

                    ;% Aero_basic_tests_B.Y24V
                    section.data(15).logicalSrcIdx = 14;
                    section.data(15).dtTransOffset = 22;

                    ;% Aero_basic_tests_B.Switch
                    section.data(16).logicalSrcIdx = 15;
                    section.data(16).dtTransOffset = 23;

                    ;% Aero_basic_tests_B.VVavg
                    section.data(17).logicalSrcIdx = 16;
                    section.data(17).dtTransOffset = 26;

                    ;% Aero_basic_tests_B.Vin
                    section.data(18).logicalSrcIdx = 17;
                    section.data(18).dtTransOffset = 29;

                    ;% Aero_basic_tests_B.Constant
                    section.data(19).logicalSrcIdx = 18;
                    section.data(19).dtTransOffset = 32;

                    ;% Aero_basic_tests_B.Count
                    section.data(20).logicalSrcIdx = 19;
                    section.data(20).dtTransOffset = 33;

                    ;% Aero_basic_tests_B.Sum
                    section.data(21).logicalSrcIdx = 20;
                    section.data(21).dtTransOffset = 34;

                    ;% Aero_basic_tests_B.div
                    section.data(22).logicalSrcIdx = 21;
                    section.data(22).dtTransOffset = 37;

            nTotData = nTotData + section.nData;
            sigMap.sections(1) = section;
            clear section


            ;%
            ;% Non-auto Data (signal)
            ;%


        ;%
        ;% Add final counts to struct.
        ;%
        sigMap.nTotData = nTotData;



    ;%*******************
    ;% Create DWork Map *
    ;%*******************
    
        nTotData      = 0; %add to this count as we go
        nTotSects     = 9;
        sectIdxOffset = 1;

        ;%
        ;% Define dummy sections & preallocate arrays
        ;%
        dumSection.nData = -1;
        dumSection.data  = [];

        dumData.logicalSrcIdx = -1;
        dumData.dtTransOffset = -1;

        ;%
        ;% Init/prealloc dworkMap
        ;%
        dworkMap.nSections           = nTotSects;
        dworkMap.sectIdxOffset       = sectIdxOffset;
            dworkMap.sections(nTotSects) = dumSection; %prealloc
        dworkMap.nTotData            = -1;

        ;%
        ;% Auto data (Aero_basic_tests_DW)
        ;%
            section.nData     = 12;
            section.data(12)  = dumData; %prealloc

                    ;% Aero_basic_tests_DW.UnitDelay_DSTATE
                    section.data(1).logicalSrcIdx = 0;
                    section.data(1).dtTransOffset = 0;

                    ;% Aero_basic_tests_DW.Sumk1n1xk_DSTATE
                    section.data(2).logicalSrcIdx = 1;
                    section.data(2).dtTransOffset = 1;

                    ;% Aero_basic_tests_DW.HILInitialize_AIMinimums
                    section.data(3).logicalSrcIdx = 2;
                    section.data(3).dtTransOffset = 4;

                    ;% Aero_basic_tests_DW.HILInitialize_AIMaximums
                    section.data(4).logicalSrcIdx = 3;
                    section.data(4).dtTransOffset = 6;

                    ;% Aero_basic_tests_DW.HILInitialize_AOMinimums
                    section.data(5).logicalSrcIdx = 4;
                    section.data(5).dtTransOffset = 8;

                    ;% Aero_basic_tests_DW.HILInitialize_AOMaximums
                    section.data(6).logicalSrcIdx = 5;
                    section.data(6).dtTransOffset = 10;

                    ;% Aero_basic_tests_DW.HILInitialize_AOVoltages
                    section.data(7).logicalSrcIdx = 6;
                    section.data(7).dtTransOffset = 12;

                    ;% Aero_basic_tests_DW.HILInitialize_FilterFrequency
                    section.data(8).logicalSrcIdx = 7;
                    section.data(8).dtTransOffset = 14;

                    ;% Aero_basic_tests_DW.HILRead_AnalogBuffer
                    section.data(9).logicalSrcIdx = 8;
                    section.data(9).dtTransOffset = 18;

                    ;% Aero_basic_tests_DW.HILRead_OtherBuffer
                    section.data(10).logicalSrcIdx = 9;
                    section.data(10).dtTransOffset = 20;

                    ;% Aero_basic_tests_DW.HILWrite_AnalogBuffer
                    section.data(11).logicalSrcIdx = 10;
                    section.data(11).dtTransOffset = 28;

                    ;% Aero_basic_tests_DW.HILWrite_OtherBuffer
                    section.data(12).logicalSrcIdx = 11;
                    section.data(12).dtTransOffset = 30;

            nTotData = nTotData + section.nData;
            dworkMap.sections(1) = section;
            clear section

            section.nData     = 1;
            section.data(1)  = dumData; %prealloc

                    ;% Aero_basic_tests_DW.HILInitialize_Card
                    section.data(1).logicalSrcIdx = 12;
                    section.data(1).dtTransOffset = 0;

            nTotData = nTotData + section.nData;
            dworkMap.sections(2) = section;
            clear section

            section.nData     = 14;
            section.data(14)  = dumData; %prealloc

                    ;% Aero_basic_tests_DW.Accelerometerms2_PWORK.LoggedData
                    section.data(1).logicalSrcIdx = 13;
                    section.data(1).dtTransOffset = 0;

                    ;% Aero_basic_tests_DW.HILRead_PWORK
                    section.data(2).logicalSrcIdx = 14;
                    section.data(2).dtTransOffset = 1;

                    ;% Aero_basic_tests_DW.Gyroscoperads_PWORK.LoggedData
                    section.data(3).logicalSrcIdx = 15;
                    section.data(3).dtTransOffset = 2;

                    ;% Aero_basic_tests_DW.MotorEncoders_PWORK.LoggedData
                    section.data(4).logicalSrcIdx = 16;
                    section.data(4).dtTransOffset = 3;

                    ;% Aero_basic_tests_DW.PitchEncoderdeg_PWORK.LoggedData
                    section.data(5).logicalSrcIdx = 17;
                    section.data(5).dtTransOffset = 4;

                    ;% Aero_basic_tests_DW.PitchMotorA_PWORK.LoggedData
                    section.data(6).logicalSrcIdx = 18;
                    section.data(6).dtTransOffset = 5;

                    ;% Aero_basic_tests_DW.PitchMotorSpeedrads_PWORK.LoggedData
                    section.data(7).logicalSrcIdx = 19;
                    section.data(7).dtTransOffset = 6;

                    ;% Aero_basic_tests_DW.Saveofthedatainthedata_step_Qub.FilePtr
                    section.data(8).logicalSrcIdx = 20;
                    section.data(8).dtTransOffset = 7;

                    ;% Aero_basic_tests_DW.YawEncoderdeg_PWORK.LoggedData
                    section.data(9).logicalSrcIdx = 21;
                    section.data(9).dtTransOffset = 8;

                    ;% Aero_basic_tests_DW.YawMotorA_PWORK.LoggedData
                    section.data(10).logicalSrcIdx = 22;
                    section.data(10).dtTransOffset = 9;

                    ;% Aero_basic_tests_DW.YawMotorSpeedrads_PWORK.LoggedData
                    section.data(11).logicalSrcIdx = 23;
                    section.data(11).dtTransOffset = 10;

                    ;% Aero_basic_tests_DW.HILWrite_PWORK
                    section.data(12).logicalSrcIdx = 24;
                    section.data(12).dtTransOffset = 11;

                    ;% Aero_basic_tests_DW.PitchMotorV_PWORK.LoggedData
                    section.data(13).logicalSrcIdx = 25;
                    section.data(13).dtTransOffset = 12;

                    ;% Aero_basic_tests_DW.YawMotorV_PWORK.LoggedData
                    section.data(14).logicalSrcIdx = 26;
                    section.data(14).dtTransOffset = 13;

            nTotData = nTotData + section.nData;
            dworkMap.sections(3) = section;
            clear section

            section.nData     = 5;
            section.data(5)  = dumData; %prealloc

                    ;% Aero_basic_tests_DW.HILInitialize_ClockModes
                    section.data(1).logicalSrcIdx = 27;
                    section.data(1).dtTransOffset = 0;

                    ;% Aero_basic_tests_DW.HILInitialize_DOStates
                    section.data(2).logicalSrcIdx = 28;
                    section.data(2).dtTransOffset = 1;

                    ;% Aero_basic_tests_DW.HILInitialize_QuadratureModes
                    section.data(3).logicalSrcIdx = 29;
                    section.data(3).dtTransOffset = 3;

                    ;% Aero_basic_tests_DW.HILInitialize_InitialEICounts
                    section.data(4).logicalSrcIdx = 30;
                    section.data(4).dtTransOffset = 7;

                    ;% Aero_basic_tests_DW.HILRead_EncoderBuffer
                    section.data(5).logicalSrcIdx = 31;
                    section.data(5).dtTransOffset = 11;

            nTotData = nTotData + section.nData;
            dworkMap.sections(4) = section;
            clear section

            section.nData     = 1;
            section.data(1)  = dumData; %prealloc

                    ;% Aero_basic_tests_DW.Saveofthedatainthedata_step_Q_a.Count
                    section.data(1).logicalSrcIdx = 32;
                    section.data(1).dtTransOffset = 0;

            nTotData = nTotData + section.nData;
            dworkMap.sections(5) = section;
            clear section

            section.nData     = 5;
            section.data(5)  = dumData; %prealloc

                    ;% Aero_basic_tests_DW.SwitchCase_ActiveSubsystem
                    section.data(1).logicalSrcIdx = 33;
                    section.data(1).dtTransOffset = 0;

                    ;% Aero_basic_tests_DW.SwitchCaseActionSubsystem2_Subs
                    section.data(2).logicalSrcIdx = 34;
                    section.data(2).dtTransOffset = 1;

                    ;% Aero_basic_tests_DW.SwitchCaseActionSubsystem1_Subs
                    section.data(3).logicalSrcIdx = 35;
                    section.data(3).dtTransOffset = 2;

                    ;% Aero_basic_tests_DW.SwitchCaseActionSubsystem_Subsy
                    section.data(4).logicalSrcIdx = 36;
                    section.data(4).dtTransOffset = 3;

                    ;% Aero_basic_tests_DW.EnabledMovingAverage_SubsysRanB
                    section.data(5).logicalSrcIdx = 37;
                    section.data(5).dtTransOffset = 4;

            nTotData = nTotData + section.nData;
            dworkMap.sections(6) = section;
            clear section

            section.nData     = 2;
            section.data(2)  = dumData; %prealloc

                    ;% Aero_basic_tests_DW.HILInitialize_DOBits
                    section.data(1).logicalSrcIdx = 38;
                    section.data(1).dtTransOffset = 0;

                    ;% Aero_basic_tests_DW.Integrator1_DWORK1
                    section.data(2).logicalSrcIdx = 39;
                    section.data(2).dtTransOffset = 2;

            nTotData = nTotData + section.nData;
            dworkMap.sections(7) = section;
            clear section

            section.nData     = 1;
            section.data(1)  = dumData; %prealloc

                    ;% Aero_basic_tests_DW.HILWrite_DigitalBuffer
                    section.data(1).logicalSrcIdx = 40;
                    section.data(1).dtTransOffset = 0;

            nTotData = nTotData + section.nData;
            dworkMap.sections(8) = section;
            clear section

            section.nData     = 1;
            section.data(1)  = dumData; %prealloc

                    ;% Aero_basic_tests_DW.EnabledMovingAverage_MODE
                    section.data(1).logicalSrcIdx = 41;
                    section.data(1).dtTransOffset = 0;

            nTotData = nTotData + section.nData;
            dworkMap.sections(9) = section;
            clear section


            ;%
            ;% Non-auto Data (dwork)
            ;%


        ;%
        ;% Add final counts to struct.
        ;%
        dworkMap.nTotData = nTotData;



    ;%
    ;% Add individual maps to base struct.
    ;%

    targMap.paramMap  = paramMap;
    targMap.signalMap = sigMap;
    targMap.dworkMap  = dworkMap;

    ;%
    ;% Add checksums to base struct.
    ;%


    targMap.checksum0 = 1559603274;
    targMap.checksum1 = 1409213396;
    targMap.checksum2 = 2156159343;
    targMap.checksum3 = 408196979;

