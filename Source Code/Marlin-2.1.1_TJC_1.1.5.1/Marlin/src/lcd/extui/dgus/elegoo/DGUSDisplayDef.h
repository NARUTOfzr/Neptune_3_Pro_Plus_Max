/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */
#pragma once

#if ENABLED(RTS_AVAILABLE)

  extern int power_off_type_yes;

  #define FHONE   (0x5A)
  #define FHTWO   (0xA5)
  #define FHLENG  (0x06)
  //#define TEXTBYTELEN     20
  #define TEXTBYTELEN    40
  //#define MaxFileNumber   20
  #define MaxFileNumber   25

  #define FileNum             MaxFileNumber
  #define FileNameLen         TEXTBYTELEN
  #define RTS_UPDATE_INTERVAL 2000
  #define RTS_UPDATE_VALUE    (2 * RTS_UPDATE_INTERVAL)

  #define SizeofDatabuf       26

  /*************Register and Variable addr*****************/
  #define RegAddr_W   0x80
  #define RegAddr_R   0x81
  #define VarAddr_W   0x82
  #define VarAddr_R   0x83
  #define ExchangePageBase    ((unsigned long)0x5A010000)
  #define StartSoundSet       ((unsigned long)0x060480A0)
  #define FONT_EEPROM         0

  /*variable addr*/
  #define ExchangepageAddr      0x0084
  #define SoundAddr             0x00A0

  #define START1_PROCESS_ICON_VP             0x1000
  #define PRINT_SPEED_RATE_VP                0x1006
  #define PRINT_PROCESS_ICON_VP              0x100E
  #define PRINT_TIME_HOUR_VP                 0x1010
  #define PRINT_TIME_MIN_VP                  0x1012
  #define PRINT_PROCESS_VP                   0x1016
  #define HEAD0_FAN_ICON_VP                  0x101E
  #define HEAD1_FAN_ICON_VP                  0x101F
  #define CHANGE_FILAMENT0_TEMP_VP           0x1020
  #define CHANGE_FILAMENT1_TEMP_VP           0x1022
  #define AUTO_BED_LEVEL_ZOFFSET_VP          0x1026

  #define HEAD0_SET_TEMP_VP                  0x1034
  #define HEAD0_CURRENT_TEMP_VP              0x1036
  #define HEAD1_SET_TEMP_VP                  0x1038
  #define SPEED_SET_VP                       0x1038
  #define HEAD1_CURRENT_TEMP_VP              0x1052
  #define BED_SET_TEMP_VP                    0x103A
  #define BED_CURRENT_TEMP_VP                0x103C
  #define AUTO_HOME_DISPLAY_ICON_VP          0x1042
  #define AXIS_X_COORD_VP                    0x1048
  #define AXIS_Y_COORD_VP                    0x104A
  #define AXIS_Z_COORD_VP                    0x104C
  #define HEAD0_FILAMENT_LOAD_DATA_VP        0x1054
  #define HEAD1_FILAMENT_LOAD_DATA_VP        0x1058
  #define PRINTER_MACHINE_TEXT_VP            0x1060
  #define PRINTER_VERSION_TEXT_VP            0x106A
  #define PRINTER_PRINTSIZE_TEXT_VP          0x1074
  #define PRINTER_WEBSITE_TEXT_VP            0x107E
  #define AUTO_BED_LEVEL_ICON_VP             0x108D
  #define CHANGE_FILAMENT_ICON_VP            0x108E
  #define TWO_EXTRUDER_HOTEND_XOFFSET_VP     0x1092
  #define TWO_EXTRUDER_HOTEND_YOFFSET_VP     0x1094
  #define TWO_EXTRUDER_HOTEND_ZOFFSET_VP     0x1096

  #define AUTO_BED_LEVEL_1POINT_VP           0x1100
  #define AUTO_BED_LEVEL_2POINT_VP           0x1102
  #define AUTO_BED_LEVEL_3POINT_VP           0x1104
  #define AUTO_BED_LEVEL_4POINT_VP           0x1106
  #define AUTO_BED_LEVEL_5POINT_VP           0x1108
  #define AUTO_BED_LEVEL_6POINT_VP           0x110A
  #define AUTO_BED_LEVEL_7POINT_VP           0x110C
  #define AUTO_BED_LEVEL_8POINT_VP           0x110E
  #define AUTO_BED_LEVEL_9POINT_VP           0x1110
  #define AUTO_BED_LEVEL_10POINT_VP          0x1112
  #define AUTO_BED_LEVEL_11POINT_VP          0x1114
  #define AUTO_BED_LEVEL_12POINT_VP          0x1116
  #define AUTO_BED_LEVEL_13POINT_VP          0x1118
  #define AUTO_BED_LEVEL_14POINT_VP          0x111A
  #define AUTO_BED_LEVEL_15POINT_VP          0x111C
  #define AUTO_BED_LEVEL_16POINT_VP          0x111E

  #define PRINT_SURPLUS_TIME_HOUR_VP         0x1162
  #define PRINT_SURPLUS_TIME_MIN_VP          0x1164
  #define SELECT_MODE_ICON_VP                0x1166
  #define CHANGE_SDCARD_ICON_VP              0x1168

  #define VP_LANGUGE_SELECT_1                0x1169
  #define VP_LANGUGE_SELECT_2                0x116A
  #define VP_LANGUGE_SELECT_3                0x116B
  #define VP_LANGUGE_SELECT_4                0x116C
  #define VP_LANGUGE_SELECT_5                0x116D
  #define VP_LANGUGE_SELECT_6                0x116E
  #define VP_LANGUGE_SELECT_7                0x116F
  #define VP_LANGUGE_SELECT_8                0x1170

  #define MOTOR_FREE_ICON_VP                 0x1200
  #define FILE1_SELECT_ICON_VP               0x1221
  #define FILE2_SELECT_ICON_VP               0x1222
  #define FILE3_SELECT_ICON_VP               0x1223
  #define FILE4_SELECT_ICON_VP               0x1224
  #define FILE5_SELECT_ICON_VP               0x1225
  #define FILE6_SELECT_ICON_VP               0x1226
  #define FILE7_SELECT_ICON_VP               0x1227
  #define FILE8_SELECT_ICON_VP               0x1228
  #define FILE9_SELECT_ICON_VP               0x1229
  #define FILE10_SELECT_ICON_VP              0x122A
  #define FILE11_SELECT_ICON_VP              0x122B
  #define FILE12_SELECT_ICON_VP              0x122C
  #define FILE13_SELECT_ICON_VP              0x122D
  #define FILE14_SELECT_ICON_VP              0x122E
  #define FILE15_SELECT_ICON_VP              0x122F
  #define FILE16_SELECT_ICON_VP              0x1230
  #define FILE17_SELECT_ICON_VP              0x1231
  #define FILE18_SELECT_ICON_VP              0x1232
  #define FILE19_SELECT_ICON_VP              0x1233
  #define FILE20_SELECT_ICON_VP              0x1234

  #define PRHEAT_NOZZLE_TEMP_VP              0x1236
  #define PRHEAT_BED_TEMP_VP                 0x1238

  #define FILE1_TEXT_VP                      0x200A
  #define FILE2_TEXT_VP                      0x201E
  #define FILE3_TEXT_VP                      0x2032
  #define FILE4_TEXT_VP                      0x2046
  #define FILE5_TEXT_VP                      0x205A
  #define FILE6_TEXT_VP                      0x206E
  #define FILE7_TEXT_VP                      0x2082
  #define FILE8_TEXT_VP                      0x2096
  #define FILE9_TEXT_VP                      0x20AA
  #define FILE10_TEXT_VP                     0x20BE
  #define FILE11_TEXT_VP                     0x20D2
  #define FILE12_TEXT_VP                     0x20E6
  #define FILE13_TEXT_VP                     0x20FA
  #define FILE14_TEXT_VP                     0x210E
  #define FILE15_TEXT_VP                     0x2122
  #define FILE16_TEXT_VP                     0x2136
  #define FILE17_TEXT_VP                     0x214A
  #define FILE18_TEXT_VP                     0x215E
  #define FILE19_TEXT_VP                     0x2172
  #define FILE20_TEXT_VP                     0x2186

  #define SELECT_FILE_TEXT_VP                0x219A
  #define TWO_COLOR_MODE_ICON_VP             0x21B8
  #define COPY_MODE_ICON_VP                  0x21B9
  #define MIRROR_MODE_ICON_VP                0x21BA
  #define SINGLE_MODE_ICON_VP                0x21BB
  #define EXCHANGE_NOZZLE_ICON_VP            0x21BC
  #define PRINT_MODE_ICON_VP                 0x21BD
  #define PRINT_FILE_TEXT_VP                 0x21C0

  #define FilenameNature                     0x6003

  #define ICON_ADJUST_PRINTING_EXTRUDER_OR_BED  0x2400
  #define ICON_ADJUST_PRINTING_TEMP_UNIT        0x2402
  #define ICON_ADJUST_PRINTING_SPEED_FLOW       0x2404
  #define ICON_ADJUST_PRINTING_S_F_UNIT         0x2406
  #define ICON_ADJUST_Z_OFFSET_UNIT             0x2408
  #define ICON_ADJUST_LED2                      0x240A
  #define ICON_ADJUST_LED3                      0x240C
  #define ICON_LEVEL_SELECT                     0x240E
  #define ICON_MOVE_DISTANCE_SELECT             0x2403
  #define ICON_FILMENT_DETACT                   0x2410

  extern celsius_t pla_extrusion_temp;
  extern celsius_t pla_bed_temp;
    
  extern celsius_t petg_extrusion_temp;
  extern celsius_t petg_bed_temp;

  extern celsius_t abs_extrusion_temp;
  extern celsius_t abs_bed_temp;

  extern celsius_t tpu_extrusion_temp;
  extern celsius_t tpu_bed_temp;

  extern celsius_t probe_extrusion_temp;
  extern celsius_t probe_bed_temp;

  extern uint8_t showcount;
  extern bool abortSD_flag;
  extern bool RTS_M600_Flag;
  extern bool Home_stop_flag; 

  #define PIC_TXT_GOCDE     

  /************struct**************/
  typedef struct DataBuf
  {
    unsigned char len;
    unsigned char head[2];
    unsigned char command;
    unsigned long addr;
    unsigned long bytelen;
    unsigned short data[32];
    unsigned char reserv[4];
  } DB;

  typedef struct CardRecord
  {
    int recordcount;
    int Filesum;
    unsigned long addr[FileNum];
    char Cardshowfilename[FileNum][FileNameLen];
    char Cardfilename[FileNum][FileNameLen];
  }CRec;

  class RTSUI
  {
    public:
    #if ENABLED(LCD_BED_LEVELING) && EITHER(PROBE_MANUALLY, MESH_BED_LEVELING)
      static bool wait_for_bl_move;
    #else
      static constexpr bool wait_for_bl_move = false;
    #endif
  };

  extern RTSUI rtsui;

  class RTSSHOW
  {
    public:
      RTSSHOW();
      int RTS_RecData();
      void RTS_SDCardInit(void);
      bool RTS_SD_Detected(void);
      void RTS_SDCardUpate(void);
      void RTS_SndData(void);
      void RTS_SndData(const String &, unsigned long, unsigned char = VarAddr_W);
      void RTS_SndData(const char[], unsigned long, unsigned char = VarAddr_W);
      void RTS_SndData(char, unsigned long, unsigned char = VarAddr_W);
      void RTS_SndData(unsigned char*, unsigned long, unsigned char = VarAddr_W);
      void RTS_SndData(int, unsigned long, unsigned char = VarAddr_W);
      void RTS_SndData(float, unsigned long, unsigned char = VarAddr_W);
      void RTS_SndData(unsigned int,unsigned long, unsigned char = VarAddr_W);
      void RTS_SndData(long,unsigned long, unsigned char = VarAddr_W);
      void RTS_SndData(unsigned long,unsigned long, unsigned char = VarAddr_W);
      void RTS_SDcard_Stop();
      void RTS_HandleData();
      void RTS_Init();

      DB recdat;
      DB snddat;
    private:
      unsigned char databuf[SizeofDatabuf];
  };

  extern RTSSHOW rtscheck;

  enum PROC_COM
  {
    MainPageKey = 0,
    AdjustmentKey,
    PrintSpeedKey,
    StopPrintKey,
    PausePrintKey,
    ResumePrintKey,
    ZOffsetKey,
    TempScreenKey,
    CoolScreenKey,
    Heater0TempEnterKey,
    Heater1TempEnterKey,
    HotBedTempEnterKey,
    SettingScreenKey,
    SettingBackKey,
    BedLevelFunKey,
    AxisPageSelectKey,
    XaxismoveKey,
    YaxismoveKey,
    ZaxismoveKey,
    SelectExtruderKey,
    Heater0LoadEnterKey,
    FilamentLoadKey,
    Heater1LoadEnterKey,
    SelectLanguageKey,
    FilamentCheckKey,
    PowerContinuePrintKey,
    PrintSelectModeKey,
    XhotendOffsetKey,
    YhotendOffsetKey,
    ZhotendOffsetKey,
    StoreMemoryKey,
    PrintFileKey,
    SelectFileKey,
    ChangePageKey,
    SetPreNozzleTemp,
    SetPreBedTemp,
    HardwareTest,
    Err_Control
  };

  const unsigned long Addrbuf[] = 
  {
    0x1002, //MainPageKey
    0x1004, //AdjustmentKey
    0x1006, //PrintSpeedKey
    0x1008, //StopPrintKey
    0x100A, //PausePrintKey
    0x100C, //ResumePrintKey
    0x1026, //ZOffsetKey
    0x1030, //TempScreenKey
    0x1032, //CoolScreenKey
    0x1034, //Heater0TempEnterKey
    0x1038, //Heater1TempEnterKey
    0x103A, //HotBedTempEnterKey
    0x103E, //SettingScreenKey
    0x1040, //SettingBackKey
    0x1044, //BedLevelFunKey
    0x1046, //AxisPageSelectKey
    0x1048, //XaxismoveKey
    0x104A, //YaxismoveKey
    0x104C, //ZaxismoveKey
    0x104E, //SelectExtruderKey
    0x1054, //Heater0LoadEnterKey
    0x1056, //FilamentLoadKey
    0x1058, //Heater1LoadEnterKey
    0x105C, //SelectLanguageKey
    0x105E, //FilamentCheckKey
    0x105F, //PowerContinuePrintKey
    0x1090, //PrintSelectModeKey
    0x1092, //XhotendOffsetKey
    0x1094, //YhotendOffsetKey
    0x1096, //ZhotendOffsetKey
    0x1098, //StoreMemoryKey
    0x2198, //PrintFileKey
    0x2199, //SelectFileKey
    0x110E, //ChangePageKey
    0x2200, //SetPreNozzleTemp
    0x2201, //SetPreBedTemp
    0x2202, //HardwareTest
    0X2203, //Err_Control
    0
  };

  extern void RTSUpdate();
  extern void RTSInit();

  extern void RTS_reset_settings(void); 

  // extern uint8_t active_extruder_font;
  // extern uint8_t dualXPrintingModeStatus;
  // extern int Update_Time_Value;
  extern bool PoweroffContinue;
  // extern bool sdcard_pause_check;
  // extern bool sd_printing_autopause;

  // extern char save_dual_x_carriage_mode;
  // extern float current_position_x0_axis;
  // extern float current_position_x1_axis;

  void RTS_PauseMoveAxisPage();
  void RTS_AutoBedLevelPage();
  void RTS_MoveAxisHoming();
  void EachMomentUpdate();

  #define MACVERSION              STRING_CONFIG_H_AUTHOR
  #define SOFTVERSION             "1.1.5.1"
  #define CORP_WEBSITE            "www.elegoo.com"

  #if ENABLED(SDSUPPORT)
    #include "../../../../sd/SdFile.h"
    #include "../../../../sd/cardreader.h"
  #endif

  class MediaFileReader {
    private:
      #if ENABLED(SDSUPPORT)
        SdFile root, file;
      #endif

    public:
      bool open(const char *filename);
      int16_t read(void *buff, size_t bytes);
      uint32_t size();
      void rewind();
      void close();
      void seekset(const uint32_t pos);
      static int16_t read(void *obj, void *buff, size_t bytes);
  };

#endif
