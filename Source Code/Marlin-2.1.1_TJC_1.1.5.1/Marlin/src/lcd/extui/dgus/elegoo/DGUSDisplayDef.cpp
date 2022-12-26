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

#include "../../../../inc/MarlinConfigPre.h"

#if ENABLED(RTS_AVAILABLE)
  #include "../../../../MarlinCore.h"
  #include "../../../../HAL/HAL.h"
  #include "../../../../core/macros.h"
  #include "../../../../inc/MarlinConfigPre.h"
  #include "../../../../module/temperature.h"
  #if ENABLED(POWER_LOSS_RECOVERY)
    #include "../../../../feature/powerloss.h"
  #endif
  #if ENABLED(EEPROM_SETTINGS)
    #include "../../../../module/settings.h"
  #endif
  #include "../../../../module/temperature.h"
  #include "../../../../module/motion.h"
  #include "../../../../module/planner.h"
  #include "../../../../module/printcounter.h"
  #include "../../../../module/stepper.h"
  #include "../../../../module/endstops.h"
  #include "../../../../feature/babystep.h"
  #include "../../../../gcode/gcode.h"
  #include "../../../../../src/feature/bedlevel/bedlevel.h"
  #include "../../../../../src/feature/bedlevel/abl/bbl.h"
  
  #include "../../../../lcd/extui/dgus/elegoo/DGUSDisplayDef.h"
  #include "../../../../lcd/extui/ui_api.h"

  #include "../../../../lcd/extui/dgus/DGUSDisplay.h"
  #include "../../../../lcd/extui/dgus/DGUSScreenHandler.h"
  #include "../../../../lcd/extui/dgus/DGUSScreenHandler.h"
  #include "../../../../lcd/extui/dgus/DGUSScreenHandlerBase.h"
  #include "../../../../lcd/extui/dgus/DGUSVPVariable.h"
  

  //Stepper stepper; // Singleton

  #define CHECKFILEMENT true

  float zprobe_zoffset;
  float last_zoffset = 0.0;

  //float manual_feedrate_mm_m[] = {50 * 60, 50 * 60, 4 * 60, 60};
  float manual_feedrate_mm_m[] = {50 * 60, 50 * 60, 4 * 60, 120};

  //bed_mesh_t z_values;
  uint8_t showcount = 0;

  int startprogress = 0;
  CRec CardRecbuf;
  float pause_z = 0;
  float pause_e = 0;
  bool sdcard_pause_check = true;
  bool print_preheat_check = false;

  float ChangeFilament0Temp = 200;
  float ChangeFilament1Temp = 200;

  float current_position_x0_axis = X_MIN_POS;
  #if ENABLED(DUAL_X_CARRIAGE)
    float current_position_x1_axis = X2_MAX_POS;
    float current_position_x1_axis = X_MIN_POS;
  #endif
  
  int heatway = 0;
  millis_t next_rts_update_ms = 0;
  int last_target_temperature[4] = {0};
  int last_target_temperature_bed;
  char waitway = 0;
  int recnum = 0;
  unsigned char Percentrecord = 0;

  bool pause_action_flag = false;
  int power_off_type_yes = 0;
  // represents to update file list
  bool CardUpdate = false;

  extern CardReader card;
  // represents SD-card status, true means SD is available, false means opposite.
  bool lcd_sd_status;

  char Checkfilenum = 0;
  int FilenamesCount = 0;
  char cmdbuf[20] = {0};
  float Filament0LOAD = 50;
  float Filament1LOAD = 50;
  float XoffsetValue = 0;

  // 0 for 10mm, 1 for 1mm, 2 for 0.1mm
  unsigned char AxisUnitMode;
  float axis_unit = 1;
  unsigned char AutoHomeIconNum;
  RTSSHOW rtscheck;
  int Update_Time_Value = 0;

  bool PoweroffContinue = false;
  char commandbuf[30];
  bool active_extruder_flag = false;

  static int change_page_number = 1; 

  char save_dual_x_carriage_mode = 0;

  uint16_t remain_time = 0;

  uint8_t active_extruder_font;
  uint8_t dualXPrintingModeStatus;
  //int Update_Time_Value;
  //bool PoweroffContinue;
  //bool sdcard_pause_check;
  bool sd_printing_autopause;

  float zoffset_unit = 0.1;
  uint8_t unit = 1;  //调整单位
  uint8_t temp_ctrl  = 1;
  uint8_t speed_ctrl = 1;

  bool status_led1 = false;
  bool status_led2 = false;

  uint8_t temp_set_flag = 0x00;
  
  celsius_t pla_extrusion_temp;
  celsius_t pla_bed_temp;
    
  celsius_t petg_extrusion_temp;
  celsius_t petg_bed_temp;

  celsius_t abs_extrusion_temp;
  celsius_t abs_bed_temp;

  celsius_t tpu_extrusion_temp;
  celsius_t tpu_bed_temp;

  celsius_t probe_extrusion_temp;
  celsius_t probe_bed_temp;

  //bool enable_filment_check = false; 
  bool enable_filment_check = true;

  char temp[128];

  uint8_t flag_led1_run_ctrl = 1;

  static unsigned char last_cardpercentValue = 100;

  #if ENABLED(DUAL_X_CARRIAGE)
    char save_dual_x_carriage_mode;
    float current_position_x0_axis;
    float current_position_x1_axis;
  #endif

  uint8_t advaned_set = 0;

  extern ExtUI::FileList filelist;

  float opos = 0;
  float npos = 0;

  char tmpfilename[32];

  bool abortSD_flag   = false;
  bool RTS_M600_Flag  = false;
  bool Home_stop_flag = false;

  void RTS_reset_settings(void) 
  {
    pla_extrusion_temp = PREHEAT_1_TEMP_HOTEND;
    pla_bed_temp = PREHEAT_1_TEMP_BED;
      
    petg_extrusion_temp = PREHEAT_2_TEMP_HOTEND;
    petg_bed_temp = PREHEAT_2_TEMP_BED;

    abs_extrusion_temp = PREHEAT_3_TEMP_HOTEND;
    abs_bed_temp = PREHEAT_3_TEMP_BED;

    tpu_extrusion_temp = PREHEAT_4_TEMP_HOTEND;
    tpu_bed_temp = PREHEAT_4_TEMP_BED;

    probe_extrusion_temp = LEVELING_NOZZLE_TEMP;
    probe_bed_temp = LEVELING_BED_TEMP;  
  }

  inline void RTS_line_to_current(AxisEnum axis)
  {
    if (!planner.is_full())
    {
      planner.buffer_line(current_position, MMM_TO_MMS(manual_feedrate_mm_m[(int8_t)axis]), active_extruder);
    }
  }

  RTSSHOW::RTSSHOW()
  {
    recdat.head[0] = snddat.head[0] = FHONE;
    recdat.head[1] = snddat.head[1] = FHTWO;
    memset(databuf, 0, sizeof(databuf));
  }

  #if ENABLED(SDSUPPORT)
    bool MediaFileReader::open(const char *filename) {
      root = CardReader::getroot();
      return file.open(&root, filename, O_READ);
    }

    int16_t MediaFileReader::read(void *buff, size_t bytes) {
      return file.read(buff, bytes);
    }

    void MediaFileReader::close() {
      file.close();
    }

    uint32_t MediaFileReader::size() {
      return file.fileSize();
    }

    void MediaFileReader::rewind() {
      file.rewind();
    }

    void MediaFileReader::seekset(const uint32_t pos) {
      file.seekSet(pos);
    }

    int16_t MediaFileReader::read(void *obj, void *buff, size_t bytes) {
      return reinterpret_cast<MediaFileReader*>(obj)->read(buff, bytes);
    }
  #else
    bool MediaFileReader::open(const char*)               {return -1;}
    int16_t MediaFileReader::read(void *, size_t)         {return 0;}
    void MediaFileReader::close()                         {}
    uint32_t MediaFileReader::size()                      {return 0;}
    void MediaFileReader::rewind()                        {}
    int16_t MediaFileReader::read(void *, void *, size_t) {return 0;}
  #endif

  void RTSSHOW::RTS_SndData(void)
  {
    if((snddat.head[0] == FHONE) && (snddat.head[1] == FHTWO) && (snddat.len >= 3))
    {
      databuf[0] = snddat.head[0];
      databuf[1] = snddat.head[1];
      databuf[2] = snddat.len;
      databuf[3] = snddat.command;

      // to write data to the register
      if(snddat.command == 0x80)
      {
        databuf[4] = snddat.addr;
        for(int i = 0;i <(snddat.len - 2);i ++)
        {
          databuf[5 + i] = snddat.data[i];
        }
      }
      else if((snddat.len == 3) && (snddat.command == 0x81))
      {
        // to read data from the register
        databuf[4] = snddat.addr;
        databuf[5] = snddat.bytelen;
      }
      else if(snddat.command == 0x82)
      {
        // to write data to the variate
        databuf[4] = snddat.addr >> 8;
        databuf[5] = snddat.addr & 0xFF;
        for(int i =0;i <(snddat.len - 3);i += 2)
        {
          databuf[6 + i] = snddat.data[i/2] >> 8;
          databuf[7 + i] = snddat.data[i/2] & 0xFF;
        }
      }
      else if((snddat.len == 4) && (snddat.command == 0x83))
      {
        // to read data from the variate
        databuf[4] = snddat.addr >> 8;
        databuf[5] = snddat.addr & 0xFF;
        databuf[6] = snddat.bytelen;
      }
      
      #if ENABLED(TJC_AVAILABLE)

      #else
        LCD_SERIAL_2.write(databuf,snddat.len + 3);
        LCD_SERIAL_2.flush();
      #endif
    }

    memset(&snddat, 0, sizeof(snddat));
    memset(databuf, 0, sizeof(databuf));
    snddat.head[0] = FHONE;
    snddat.head[1] = FHTWO;
  }

  void RTSSHOW::RTS_SndData(const String &s, unsigned long addr, unsigned char cmd /*= VarAddr_W*/)
  {
    if(s.length() < 1)
    {
      return;
    }
    RTS_SndData(s.c_str(), addr, cmd);
  }

  void RTSSHOW::RTS_SndData(const char *str, unsigned long addr, unsigned char cmd/*= VarAddr_W*/)
  {
    int len = strlen(str);
    if(len > 0)
    {
      databuf[0] = FHONE;
      databuf[1] = FHTWO;
      databuf[2] = 3 + len;
      databuf[3] = cmd;
      databuf[4] = addr >> 8;
      databuf[5] = addr & 0x00FF;
      for(int i = 0;i < len;i ++)
      {
        databuf[6 + i] = str[i];
      }

      for(int i = 0;i < (len + 6);i ++)
      {
        #if ENABLED(TJC_AVAILABLE)

        #else
          LCD_SERIAL_2.write(databuf,snddat.len + 3);
          LCD_SERIAL_2.flush();
        #endif

        delayMicroseconds(1);
      }

      memset(databuf, 0, sizeof(databuf));
    }
  }

  void RTSSHOW::RTS_SndData(char c, unsigned long addr, unsigned char cmd /*= VarAddr_W*/)
  {
    snddat.command = cmd;
    snddat.addr = addr;
    snddat.data[0] = (unsigned long)c;
    snddat.data[0] = snddat.data[0] << 8;
    snddat.len = 5;
    RTS_SndData();
  }

  void RTSSHOW::RTS_SndData(unsigned char *str, unsigned long addr, unsigned char cmd) 
  { 
    RTS_SndData((char *)str, addr, cmd); 
  }

  void RTSSHOW::RTS_SndData(int n, unsigned long addr, unsigned char cmd /*= VarAddr_W*/)
  {
    if (cmd == VarAddr_W)
    {
      if (n > 0xFFFF)
      {
        snddat.data[0] = n >> 16;
        snddat.data[1] = n & 0xFFFF;
        snddat.len = 7;
      }
      else
      {
        snddat.data[0] = n;
        snddat.len = 5;
      }
    }
    else if (cmd == RegAddr_W)
    {
      snddat.data[0] = n;
      snddat.len = 3;
    }
    else if (cmd == VarAddr_R)
    {
      snddat.bytelen = n;
      snddat.len = 4;
    }
    snddat.command = cmd;
    snddat.addr = addr;
    RTS_SndData();
  }

  void RTSSHOW::RTS_SndData(unsigned int n, unsigned long addr, unsigned char cmd) 
  { 
    RTS_SndData((int)n, addr, cmd); 
  }

  void RTSSHOW::RTS_SndData(float n, unsigned long addr, unsigned char cmd) 
  { 
    RTS_SndData((int)n, addr, cmd); 
  }

  void RTSSHOW::RTS_SndData(long n, unsigned long addr, unsigned char cmd) 
  { 
    RTS_SndData((unsigned long)n, addr, cmd); 
  }

  void RTSSHOW::RTS_SndData(unsigned long n, unsigned long addr, unsigned char cmd /*= VarAddr_W*/)
  {
    if (cmd == VarAddr_W)
    {
      if (n > 0xFFFF)
      {
        snddat.data[0] = n >> 16;
        snddat.data[1] = n & 0xFFFF;
        snddat.len = 7;
      }
      else
      {
        snddat.data[0] = n;
        snddat.len = 5;
      }
    }
    else if (cmd == VarAddr_R)
    {
      snddat.bytelen = n;
      snddat.len = 4;
    }
    snddat.command = cmd;
    snddat.addr = addr;
    RTS_SndData();
  }

  void RTSSHOW::RTS_SDCardInit(void)
  {
    if(RTS_SD_Detected())
    {
      card.mount();
    }

    // char tmpfilename[32]={0};
    // for(unsigned char filecount=0;filecount<25;filecount++)
    // {
    //   uint8_t page_num = ((filecount / 5) + 1);

    //   if (filelist.seek(filecount)) 
    //   {                   
    //     sprintf(tmpfilename, "file%d.t%d.txt=\"%s\"", page_num, filecount , filelist.filename());
    //     LCD_SERIAL_2.printf(tmpfilename);
    //     LCD_SERIAL_2.printf("\xff\xff\xff");
    //   }
    //   else
    //   {
    //     sprintf(tmpfilename, "file%d.t%d.txt=\"\"", page_num, filecount);
    //     LCD_SERIAL_2.printf(tmpfilename);
    //     LCD_SERIAL_2.printf("\xff\xff\xff");                    
    //   }
    // }

    if(CardReader::flag.mounted)
    {
      uint16_t fileCnt = card.get_num_Files();

      card.getWorkDirName();
      if(card.filename[0] != '/')
      {
        card.cdup();
      }

      int addrnum = 0;
      int num = 0;

      for(uint16_t i = 0;(i < fileCnt) && (i < (MaxFileNumber + addrnum));i++)
      {
        card.selectFileByIndex(fileCnt - 1 - i);  
        char *pointFilename = card.longFilename;
        int filenamelen = strlen(card.longFilename);
        int j = 1;
        while((strncmp(&pointFilename[j], ".gcode", 6) && strncmp(&pointFilename[j], ".GCODE", 6)) && ((j ++) < filenamelen));
        //while((strncmp(&pointFilename[j], ".gcode", 6) && strncmp(&pointFilename[j], ".GCODE", 6)) && ((j ++) < 64));
        if(j >= filenamelen)
        //if(j >= 64)
        {
          addrnum++;
          continue;
        }
        if (j >= TEXTBYTELEN)
        {
          strncpy(&card.longFilename[TEXTBYTELEN - 3], "..", 2);
          card.longFilename[TEXTBYTELEN - 1] = '\0';
          j = TEXTBYTELEN - 1;
        }
        memset(CardRecbuf.Cardshowfilename[num],0,sizeof(CardRecbuf.Cardshowfilename[num]));
        strncpy(CardRecbuf.Cardshowfilename[num], card.longFilename, j);
        strcpy(CardRecbuf.Cardfilename[num], card.filename);
        CardRecbuf.addr[num] = FILE1_TEXT_VP + (num * 20);
        RTS_SndData(CardRecbuf.Cardshowfilename[num], CardRecbuf.addr[num]);

        //清文件名
        for(int j = CardRecbuf.Filesum;j < MaxFileNumber;j ++)
        {
          CardRecbuf.addr[j] = FILE1_TEXT_VP + (j * 20);
          RTS_SndData(0, CardRecbuf.addr[j]);

          #if ENABLED(TJC_AVAILABLE)
            char tmpfilename[32]={0};
            uint8_t page_num = ((j / 5) + 1);
            sprintf(tmpfilename, "file%d.t%d.txt=\"\"", page_num, j); 
            LCD_SERIAL_2.printf(tmpfilename);
            LCD_SERIAL_2.printf("\xff\xff\xff"); 
          #endif
        }

        //显示文件名
        #if ENABLED(TJC_AVAILABLE)
          uint8_t page_num = ((num / 5) + 1);
          char filename[128]={0}; 
          sprintf(filename, "file%d.t%d.txt=\"%s\"", page_num, num, CardRecbuf.Cardshowfilename[num]); 
          LCD_SERIAL_2.printf(filename);
          LCD_SERIAL_2.printf("\xff\xff\xff");         
        #endif

        CardRecbuf.Filesum = (++num);
      }

      // clean print file
      for(int j = 0;j < 25;j ++)
      {
        RTS_SndData(0, PRINT_FILE_TEXT_VP + j);       
      }

      lcd_sd_status = IS_SD_INSERTED();
    }
    else
    {
      if(sd_printing_autopause == true)
      {
        RTS_SndData(CardRecbuf.Cardshowfilename[CardRecbuf.recordcount], PRINT_FILE_TEXT_VP);
        card.mount();
      }
      else
      {
        // clean filename Icon
        for(int j = 0;j < MaxFileNumber;j ++)
        {
          // clean filename Icon
          for(int i = 0;i < TEXTBYTELEN;i ++)
          {
            RTS_SndData(0, CardRecbuf.addr[j] + i);
          }

          //清除文件 
          #if ENABLED(TJC_AVAILABLE)
            memset(tmpfilename,0,sizeof(tmpfilename));
            uint8_t page_num = ((j / 5) + 1);
            char filename[128]; 
            memset(filename,0,sizeof(filename));
            //sprintf(tmpfilename, "file%d.t%d.txt=\"\"", page_num, j); 
            //LCD_SERIAL_2.printf(tmpfilename);
            //LCD_SERIAL_2.printf("\xff\xff\xff"); 
            sprintf(filename, "file%d.t%d.txt=\"%s\"", page_num, j, tmpfilename); 
            LCD_SERIAL_2.printf(filename);
            LCD_SERIAL_2.printf("\xff\xff\xff");  
          #endif
        }
        memset(&CardRecbuf, 0, sizeof(CardRecbuf));
      }
    }
  }

  void RTSSHOW::RTS_SDcard_Stop()
  {
    waitway = 7;
    // planner.synchronize();
    // #if ENABLED(DUAL_X_CARRIAGE)
    //   extruder_duplication_enabled = false;
    //   dual_x_carriage_mode = DEFAULT_DUAL_X_CARRIAGE_MODE;
    //   active_extruder = 0;
    // #endif
    // card.endFilePrint();
    ExtUI::stopPrint();
    card.flag.abort_sd_printing = true;
    queue.clear();
    quickstop_stepper();
    print_job_timer.stop();
    #if DISABLED(SD_ABORT_NO_COOLDOWN)
      thermalManager.disable_all_heaters();
    #endif
    print_job_timer.reset();
    #if HAS_HOTEND
        thermalManager.setTargetHotend(0, 0);
        RTS_SndData(0, HEAD0_SET_TEMP_VP);
    #endif
    #if HAS_MULTI_HOTEND
        thermalManager.setTargetHotend(0, 1);
        RTS_SndData(0, HEAD1_SET_TEMP_VP);
    #endif
    thermalManager.setTargetBed(0);
    RTS_SndData(0, BED_SET_TEMP_VP);
    thermalManager.zero_fan_speeds();
    wait_for_heatup = wait_for_user = false;
    PoweroffContinue = false;
    sd_printing_autopause = false;
    if(CardReader::flag.mounted)
    {
      #if ENABLED(SDSUPPORT) && ENABLED(POWER_LOSS_RECOVERY)
        card.removeJobRecoveryFile();
      #endif
    }

    // shut down the stepper motor.
    // queue.enqueue_now_P(PSTR("M84"));
    RTS_SndData(0, MOTOR_FREE_ICON_VP);
    RTS_SndData(0, PRINT_PROCESS_ICON_VP);
    RTS_SndData(0, PRINT_PROCESS_VP);
    delay(2);
    for(int j = 0;j < 20;j ++)
    {
      RTS_SndData(0, PRINT_FILE_TEXT_VP + j); // clean screen.
      RTS_SndData(0, SELECT_FILE_TEXT_VP + j); // clean filename
    }
    #if ENABLED(TJC_AVAILABLE)
      LCD_SERIAL_2.printf("printpause.cp0.close()");
      LCD_SERIAL_2.printf("\xff\xff\xff");

      LCD_SERIAL_2.printf("printpause.cp0.aph=0");
      LCD_SERIAL_2.printf("\xff\xff\xff");          

      LCD_SERIAL_2.printf("printpause.va0.txt=\"\"");
      LCD_SERIAL_2.printf("\xff\xff\xff");

      LCD_SERIAL_2.printf("printpause.va1.txt=\"\"");
      LCD_SERIAL_2.printf("\xff\xff\xff");
    #endif
  }

  bool RTSSHOW::RTS_SD_Detected(void)
  {
    static bool last;
    static bool state;
    static bool flag_stable;
    static uint32_t stable_point_time;

    bool tmp = IS_SD_INSERTED();

    if(tmp != last)
    {
      flag_stable = false;
    }
    else
    {
      if(!flag_stable)
      {
        flag_stable = true;
        stable_point_time = millis();
      }
    }

    if(flag_stable)
    {
      if(millis() - stable_point_time > 30)
      {
        state = tmp;
      }
    }

    last = tmp;

    return state;
  }

  void RTSSHOW::RTS_SDCardUpate(void)
  {
    const bool sd_status = RTS_SD_Detected();
    if (sd_status != lcd_sd_status)
    {
      if (sd_status)
      {
        // SD card power on
        card.mount();
        RTS_SDCardInit();
      }
      else
      {
        card.release();
        
        if(sd_printing_autopause == true)
        {
          RTS_SndData(CardRecbuf.Cardshowfilename[CardRecbuf.recordcount], PRINT_FILE_TEXT_VP);
        }
        else
        {
          for(int i = 0;i < CardRecbuf.Filesum;i ++)
          {
            for(int j = 0;j < 20;j ++)
            {
              RTS_SndData(0, CardRecbuf.addr[i] + j);
            }
            RTS_SndData((unsigned long)0xA514, FilenameNature + (i + 1) * 16);
          }

          for(int j = 0; j < MaxFileNumber; j++)
          {
            //清除文件
            #if ENABLED(TJC_AVAILABLE)
              char tmpfilename[128];
              memset(tmpfilename,0,sizeof(tmpfilename));
              uint8_t page_num = ((j / 5) + 1);
              sprintf(tmpfilename, "file%d.t%d.txt=\"\"", page_num, j); 
              LCD_SERIAL_2.printf(tmpfilename);
              LCD_SERIAL_2.printf("\xff\xff\xff");
            #endif  
          }

          for(int j = 0;j < 20;j ++)
          {
            // clean screen.
            RTS_SndData(0, PRINT_FILE_TEXT_VP + j);
            RTS_SndData(0, SELECT_FILE_TEXT_VP + j);
          }

          memset(&CardRecbuf, 0, sizeof(CardRecbuf));
        }
      }
      lcd_sd_status = sd_status;
    }

    // represents to update file list
    if(CardUpdate && lcd_sd_status && RTS_SD_Detected())
    {
      for(uint16_t i = 0;i < CardRecbuf.Filesum;i ++)
      {
        delay(1);
        RTS_SndData(CardRecbuf.Cardshowfilename[i], CardRecbuf.addr[i]);
        RTS_SndData((unsigned long)0xA514, FilenameNature + (i + 1) * 16);
      }
      CardUpdate = false;
    }
  }

  void RTSSHOW::RTS_Init()
  {
    AxisUnitMode = 1;

    pinMode(CHECKFILEMENT0_PIN, INPUT); //初始化

    //修改
    #if ENABLED(DUAL_X_CARRIAGE)
      active_extruder = active_extruder_font;
      save_dual_x_carriage_mode = dualXPrintingModeStatus;
      if(save_dual_x_carriage_mode == 1)
      {
        RTS_SndData(1, PRINT_MODE_ICON_VP);
        RTS_SndData(1, SELECT_MODE_ICON_VP);
      }
      else if(save_dual_x_carriage_mode == 2)
      {
        RTS_SndData(2, PRINT_MODE_ICON_VP);
        RTS_SndData(2, SELECT_MODE_ICON_VP);
      }
      else if(save_dual_x_carriage_mode == 3)
      {
        RTS_SndData(3, PRINT_MODE_ICON_VP);
        RTS_SndData(3, SELECT_MODE_ICON_VP);
      }
      else
      {
        RTS_SndData(4, PRINT_MODE_ICON_VP);
        RTS_SndData(4, SELECT_MODE_ICON_VP);
      }
    #else
     #if ENABLED(RTS_AVAILABLE)
      RTS_SndData(4, PRINT_MODE_ICON_VP);
      RTS_SndData(4, SELECT_MODE_ICON_VP);
     #endif
    #endif

    #if ENABLED(AUTO_BED_LEVELING_BILINEAR)
      //bool zig = true;
      //bool zig = false;

      bool zig = (GRID_MAX_POINTS_Y & 1);

      int8_t inStart, inStop, inInc, showcount;
      showcount = 0;
      #if ENABLED(EEPROM_SETTINGS)
        settings.load();
      #endif
      for (int y = 0; y < GRID_MAX_POINTS_Y; y++)
      {
        // away from origin
        if (zig)
        {
          inStart = 0;
          inStop = GRID_MAX_POINTS_X;
          inInc = 1;
        }
        else
        {
          // towards origin
          inStart = GRID_MAX_POINTS_X - 1;
          inStop = -1;
          inInc  = -1;
        }
        zig ^= true;
        for (int x = inStart; x != inStop; x += inInc)
        {
          #if ENABLED(EEPROM_SETTINGS)
            #if ENABLED(RTS_AVAILABLE)
              RTS_SndData(bedlevel.z_values[x][y] * 1000, AUTO_BED_LEVEL_1POINT_VP + showcount * 2);
            #endif

            #if ENABLED(TJC_AVAILABLE)
              #if ENABLED(NEPTUNE_3_PLUS)
                char temp[32] = {0};
                //sprintf(temp, "leveldata_49.x%d.val=%d",showcount,(int)(bedlevel.z_values[x][y]*100)); //显示数据
                sprintf(temp, "aux49_data.x%d.val=%d",showcount,(int)(bedlevel.z_values[x][y]*100)); //显示数据
                LCD_SERIAL_2.printf(temp);
                LCD_SERIAL_2.printf("\xff\xff\xff");
              #elif ENABLED(NEPTUNE_3_MAX)
                char temp[32] = {0};
                //sprintf(temp, "leveldata_64.x%d.val=%d",showcount,(int)(bedlevel.z_values[x][y]*100)); //显示数据
                //sprintf(temp, "aux64_data.x%d.val=%d",showcount,(int)(bedlevel.z_values[x][y]*100)); //显示数据
                sprintf(temp, "aux63_data.x%d.val=%d",showcount,(int)(bedlevel.z_values[x][y]*100)); //显示数据
                LCD_SERIAL_2.printf(temp);
                LCD_SERIAL_2.printf("\xff\xff\xff");              
              #elif ENABLED(NEPTUNE_3_PRO)
                char temp[32] = {0};
                sprintf(temp, "leveldata_36.x%d.val=%d",(int)showcount,(int)(bedlevel.z_values[x][y]*100)); //显示数据
                LCD_SERIAL_2.printf(temp);
                LCD_SERIAL_2.printf("\xff\xff\xff");
              #endif
            #endif

            showcount++;
          #endif
        }
      }
      queue.enqueue_now_P(PSTR("M420 S1"));
    #endif
    last_zoffset = zprobe_zoffset = probe.offset.z;
    #if ENABLED(RTS_AVAILABLE)
      RTS_SndData(zprobe_zoffset * 100, AUTO_BED_LEVEL_ZOFFSET_VP);
      #if ENABLED(TJC_AVAILABLE) 
        memset(temp,0,sizeof(temp));
        sprintf(temp, "leveldata.z_offset.val=%d", (int)(zprobe_zoffset * 100));
        LCD_SERIAL_2.printf(temp);
        LCD_SERIAL_2.printf("\xff\xff\xff");           
      #endif
    #endif
    #if ENABLED(TJC_AVAILABLE)
    
    #endif
    #if ENABLED(DUAL_X_CARRIAGE)
      RTS_SndData((hotend_offset[1].x - X2_MAX_POS) * 10, TWO_EXTRUDER_HOTEND_XOFFSET_VP);
      RTS_SndData(hotend_offset[1].y * 10, TWO_EXTRUDER_HOTEND_YOFFSET_VP);
      RTS_SndData(hotend_offset[1].z * 10, TWO_EXTRUDER_HOTEND_ZOFFSET_VP);
    #endif

    last_target_temperature[0] = thermalManager.temp_hotend[0].target;
    last_target_temperature_bed = thermalManager.temp_bed.target;

    #if ENABLED(DUAL_X_CARRIAGE)
      last_target_temperature[1] = thermalManager.temp_hotend[1].target;
    #endif

    feedrate_percentage = 100;
    #if ENABLED(RTS_AVAILABLE)
      RTS_SndData(feedrate_percentage, PRINT_SPEED_RATE_VP);
    #endif

    
    #if ENABLED(RTS_AVAILABLE)
      /***************turn off motor*****************/
      RTS_SndData(1, MOTOR_FREE_ICON_VP);

      /***************transmit temperature to screen*****************/
      RTS_SndData(0, HEAD0_SET_TEMP_VP);
      #if ENABLED(DUAL_X_CARRIAGE)
      RTS_SndData(0, HEAD1_SET_TEMP_VP);
      #endif
      RTS_SndData(0, BED_SET_TEMP_VP);
      RTS_SndData(thermalManager.temp_hotend[0].celsius, HEAD0_CURRENT_TEMP_VP);
      RTS_SndData(thermalManager.temp_bed.celsius, BED_CURRENT_TEMP_VP);
      #if ENABLED(DUAL_X_CARRIAGE)
      RTS_SndData(thermalManager.temp_hotend[1].celsius, HEAD1_CURRENT_TEMP_VP);
      #endif
      
      /***************transmit Fan speed to screen*****************/
      // turn off fans
      thermalManager.set_fan_speed(0, 0);
      thermalManager.set_fan_speed(1, 0);
      RTS_SndData(1, HEAD0_FAN_ICON_VP);
      RTS_SndData(1, HEAD1_FAN_ICON_VP);
      delay(5);

      /*********transmit SD card filename to screen***************/
      RTS_SDCardInit();

      /***************transmit Printer information to screen*****************/
      char sizebuf[20] = {0};
      sprintf(sizebuf, "%d X %d X %d", X_MAX_POS - 2, Y_MAX_POS - 2, Z_MAX_POS);
      RTS_SndData(MACVERSION, PRINTER_MACHINE_TEXT_VP);
      RTS_SndData(SOFTVERSION, PRINTER_VERSION_TEXT_VP);
      RTS_SndData(sizebuf, PRINTER_PRINTSIZE_TEXT_VP);
      RTS_SndData(CORP_WEBSITE, PRINTER_WEBSITE_TEXT_VP);

      #if ENABLED(TJC_AVAILABLE)

        //主板软件版本
        memset(temp,0,sizeof(temp));
        sprintf(temp, "information.sversion.txt=\"%s\"",SOFTVERSION); 
        LCD_SERIAL_2.printf(temp);
        LCD_SERIAL_2.printf("\xff\xff\xff");

        //对应屏幕软件版本
        // memset(temp,0,sizeof(temp));
        // sprintf(temp, "information.lversion.txt=\"%s\"",LCDVERSION); 
        // LCD_SERIAL_2.printf(temp);
        // LCD_SERIAL_2.printf("\xff\xff\xff");

      #endif

      /**************************some info init*******************************/
      RTS_SndData(0, PRINT_PROCESS_ICON_VP);

      //机型确认
      #if ENABLED(NEPTUNE_3_PLUS)
        LCD_SERIAL_2.printf("main.va0.val=2");  
        LCD_SERIAL_2.printf("\xff\xff\xff");
      #elif ENABLED(NEPTUNE_3_PRO)
        LCD_SERIAL_2.printf("main.va0.val=1");  
        LCD_SERIAL_2.printf("\xff\xff\xff");                  
      #elif ENABLED(NEPTUNE_3_MAX)
        LCD_SERIAL_2.printf("main.va0.val=3");  
        LCD_SERIAL_2.printf("\xff\xff\xff");
      #endif  

      //EachMomentUpdate();
    #endif

  }

  int RTSSHOW::RTS_RecData()
  {
    while((MYSERIAL1.available() > 0) && (recnum < SizeofDatabuf))
    {
      delay(1);
      databuf[recnum] = MYSERIAL1.read();
      if (databuf[0] == FHONE)
      {
        recnum++;
      }
      else if (databuf[0] == FHTWO)
      {
        databuf[0] = FHONE;
        databuf[1] = FHTWO;
        recnum += 2;
      }
      else if (databuf[0] == FHLENG)
      {
        databuf[0] = FHONE;
        databuf[1] = FHTWO;
        databuf[2] = FHLENG;
        recnum += 3;
      }
      else if (databuf[0] == VarAddr_R)
      {
        databuf[0] = FHONE;
        databuf[1] = FHTWO;
        databuf[2] = FHLENG;
        databuf[3] = VarAddr_R;
        recnum += 4;
      }
      else
      {
        recnum = 0;
      }
    }

    // receive nothing
    if (recnum < 1)
    {
      return -1;
    }
    else if ((recdat.head[0] == databuf[0]) && (recdat.head[1] == databuf[1]) && (recnum > 2))
    {
      recdat.len = databuf[2];
      recdat.command = databuf[3];
      // response for writing byte
      if ((recdat.len == 0x03) && ((recdat.command == 0x82) || (recdat.command == 0x80)) && (databuf[4] == 0x4F) && (databuf[5] == 0x4B))
      {
        memset(databuf, 0, sizeof(databuf));
        recnum = 0;
        return -1;
      }
      else if (recdat.command == 0x83)
      {
        // response for reading the data from the variate
        recdat.addr = databuf[4];
        recdat.addr = (recdat.addr << 8) | databuf[5];
        recdat.bytelen = databuf[6];
        for (unsigned int i = 0; i < recdat.bytelen; i += 2)
        {
          recdat.data[i / 2] = databuf[7 + i];
          recdat.data[i / 2] = (recdat.data[i / 2] << 8) | databuf[8 + i];
        }
      }
      else if (recdat.command == 0x81)
      {
        // response for reading the page from the register
        recdat.addr = databuf[4];
        recdat.bytelen = databuf[5];
        for (unsigned int i = 0; i < recdat.bytelen; i ++)
        {
          recdat.data[i] = databuf[6 + i];
          // recdat.data[i] = (recdat.data[i] << 8 )| databuf[7 + i];
        }
      }
    }
    else
    {
      memset(databuf, 0, sizeof(databuf));
      recnum = 0;
      // receive the wrong data
      return -1;
    }
    memset(databuf, 0, sizeof(databuf));
    recnum = 0;
    return 2;
  }

  bool flag_power_on = true;

  void EachMomentUpdate()
  {
    millis_t ms = millis();
   
    if(ms > next_rts_update_ms)
    {
      #if ENABLED(POWER_LOSS_RECOVERY)
        if(flag_power_on)
        {
          flag_power_on = false;

          //print the file before the power is off.
          if((power_off_type_yes == 0) && lcd_sd_status && (recovery.info.valid()))
          {
            uint8_t count_startprogress = 0;
            power_off_type_yes = 1;

            #if ENABLED(RTS_AVAILABLE)  
              rtscheck.RTS_SndData(ExchangePageBase, ExchangepageAddr);
              for(count_startprogress=0;count_startprogress<=100;count_startprogress++)
              {
                rtscheck.RTS_SndData(count_startprogress, START1_PROCESS_ICON_VP);
                delay(30);
              }
              rtscheck.RTS_SndData(StartSoundSet, SoundAddr);

              #if ENABLED(TJC_AVAILABLE)

                LCD_SERIAL_2.printf("page boot");
                LCD_SERIAL_2.printf("\xff\xff\xff");

                LCD_SERIAL_2.printf("com_star");
                LCD_SERIAL_2.printf("\xff\xff\xff");

                #if ENABLED(NEPTUNE_3_PLUS)
                  LCD_SERIAL_2.printf("main.va0.val=2");  
                  LCD_SERIAL_2.printf("\xff\xff\xff");
                #elif ENABLED(NEPTUNE_3_PRO)
                  LCD_SERIAL_2.printf("main.va0.val=1");  
                  LCD_SERIAL_2.printf("\xff\xff\xff");                  
                #elif ENABLED(NEPTUNE_3_MAX)
                  LCD_SERIAL_2.printf("main.va0.val=3");  
                  LCD_SERIAL_2.printf("\xff\xff\xff");
                #endif

                for(count_startprogress=0;count_startprogress<=100;count_startprogress++)
                {
                  char send[16] ={0};
                  sprintf(send, "boot.j0.val=%d", count_startprogress);
                  LCD_SERIAL_2.printf(send); 
                  LCD_SERIAL_2.printf("\xff\xff\xff");
                  delay(30);
                  TERN_(USE_WATCHDOG, hal.watchdog_refresh(););
                }          

              #endif   

              for(uint16_t i = 0;i < CardRecbuf.Filesum;i ++) 
              {
                if(!strcmp(CardRecbuf.Cardfilename[i], &recovery.info.sd_filename[1]))
                {
                  rtscheck.RTS_SndData(CardRecbuf.Cardshowfilename[i], PRINT_FILE_TEXT_VP);
                  rtscheck.RTS_SndData(ExchangePageBase + 36, ExchangepageAddr);

                  #if ENABLED(TJC_AVAILABLE)
                    memset(temp,0,sizeof(temp));
                    sprintf(temp, "continueprint.t0.txt=\"%s\"",CardRecbuf.Cardshowfilename[i]); //显示文件名
                    LCD_SERIAL_2.printf(temp);
                    LCD_SERIAL_2.printf("\xff\xff\xff");

                    memset(temp,0,sizeof(temp));
                    sprintf(temp, "printpause.t0.txt=\"%s\"",CardRecbuf.Cardshowfilename[i]); //显示文件名
                    LCD_SERIAL_2.printf(temp);
                    LCD_SERIAL_2.printf("\xff\xff\xff");

                    LCD_SERIAL_2.printf("page continueprint");
                    LCD_SERIAL_2.printf("\xff\xff\xff");                                                       
                  #endif 

                  // 图片预览
                  // #if ENABLED(TJC_AVAILABLE)

                  //   LCD_SERIAL_2.printf("printpause.cp0.close()");
                  //   LCD_SERIAL_2.printf("\xff\xff\xff");

                  //   LCD_SERIAL_2.printf("printpause.cp0.aph=0");
                  //   LCD_SERIAL_2.printf("\xff\xff\xff");          

                  //   LCD_SERIAL_2.printf("printpause.va0.txt=\"\"");
                  //   LCD_SERIAL_2.printf("\xff\xff\xff");

                  //   LCD_SERIAL_2.printf("printpause.va1.txt=\"\"");
                  //   LCD_SERIAL_2.printf("\xff\xff\xff");

                  //   char picname[64];
                  //   uint8_t public_buf[512];
                  //   MediaFileReader file;
                  //   sprintf(picname,"%s.txt",CardRecbuf.Cardshowfilename[i]);
                  //   bool run = file.open(picname);
                  //   if(run)
                  //   {
                  //     while(1)
                  //     {
                  //       TERN_(USE_WATCHDOG, hal.watchdog_refresh());

                  //       LCD_SERIAL_2.printf("printpause.cp0.aph=0");
                  //       LCD_SERIAL_2.printf("\xff\xff\xff");

                  //       LCD_SERIAL_2.printf("printpause.va0.txt=\"\"");
                  //       LCD_SERIAL_2.printf("\xff\xff\xff");

                  //       memset(public_buf,0,sizeof(public_buf));
                  //       int16_t byte = file.read(public_buf, 512);
                        
                  //       LCD_SERIAL_2.printf("printpause.va0.txt=");
                  //       LCD_SERIAL_2.write(0x22);
                  //       LCD_SERIAL_2.write(public_buf,sizeof(public_buf));
                  //       LCD_SERIAL_2.write(0x22);
                  //       LCD_SERIAL_2.printf("\xff\xff\xff");

                  //       LCD_SERIAL_2.printf("printpause.va1.txt+=printpause.va0.txt");
                  //       LCD_SERIAL_2.printf("\xff\xff\xff");
                              
                  //       if(byte<=0) 
                  //       {
                  //         LCD_SERIAL_2.printf("printpause.cp0.aph=127");
                  //         LCD_SERIAL_2.printf("\xff\xff\xff");

                  //         LCD_SERIAL_2.printf("printpause.cp0.write(printpause.va1.txt)");
                  //         LCD_SERIAL_2.printf("\xff\xff\xff");

                  //         file.close();
                  //         break;
                  //       };
                  //     }
                  //   }
                  //   else
                  //   {
                  //     file.close();
                  //     memset(picname,0,sizeof(picname));

                  //     SdFile *diveDir;
                  //     const char * const fname = card.diveToFile(true, diveDir, CardRecbuf.Cardfilename[i]);
                  //     bool run = file.open(fname);
                  //     if(run)
                  //     {
                  //       uint32_t gPicturePreviewStart = 0;
                  //       uint64_t cnt_pre = 0;

                  //       uint8_t public_buf[1024];

                  //       while(1)
                  //       {
                  //         LCD_SERIAL_2.printf("printpause.va0.txt=\"\"");
                  //         LCD_SERIAL_2.printf("\xff\xff\xff");

                  //         memset(public_buf,0,sizeof(public_buf));
                  //         int16_t  byte = file.read(public_buf,sizeof(public_buf));
                  //         cnt_pre = (cnt_pre + byte);
                  //         //uint32_t *p1  = (uint32_t *)strstr((char *)public_buf, ";gimage:");
                  //         uint32_t *p1  = (uint32_t *)strstr((char *)public_buf, ";simage:");
                  //         uint32_t *m1  = (uint32_t *)strstr((char *)public_buf, "M10086");

                  //         if(m1)
                  //         {
                  //           LCD_SERIAL_2.printf("printpause.cp0.aph=0");
                  //           LCD_SERIAL_2.printf("\xff\xff\xff");

                  //           LCD_SERIAL_2.printf("printpause.cp0.close()");
                  //           LCD_SERIAL_2.printf("\xff\xff\xff");
                  //           break;
                  //         }
                          
                  //         if(p1)
                  //         { 
                  //           while(1)
                  //           {
                  //             LCD_SERIAL_2.printf("printpause.va0.txt=\"\"");
                  //             LCD_SERIAL_2.printf("\xff\xff\xff");

                  //             memset(public_buf,0,sizeof(public_buf));
                  //             int16_t  byte = file.read(public_buf,sizeof(public_buf));
                  //             //uint32_t *p2  = (uint32_t *)strstr((char *)public_buf, ";;gimage:");
                  //             uint32_t *p2  = (uint32_t *)strstr((char *)public_buf, ";;simage:");
                  //             uint32_t *p3  = (uint32_t *)strstr((char *)public_buf, ";00000");
                  //             cnt_pre = (cnt_pre + byte);

                  //             //if(p2)
                  //             if(p2||p3)
                  //             {
                  //               file.rewind(); //文件读取指针复位

                  //               while(1) //";gimage:"起始位置
                  //               {
                  //                 while(1)
                  //                 {
                  //                   LCD_SERIAL_2.printf("printpause.va0.txt=\"\"");
                  //                   LCD_SERIAL_2.printf("\xff\xff\xff");

                  //                   TERN_(USE_WATCHDOG, hal.watchdog_refresh());
                  //                   memset(public_buf,0,sizeof(public_buf));
                  //                   int16_t byte = file.read(public_buf,1024);
                  //                   gPicturePreviewStart+=byte;

                  //                   //uint32_t *p1 = (uint32_t *)strstr((char *)public_buf, ";gimage:");
                  //                   //uint32_t *p2 = (uint32_t *)strstr((char *)public_buf, ";;gimage:");
                  //                   uint32_t *p1 = (uint32_t *)strstr((char *)public_buf, ";simage:");
                  //                   uint32_t *p2 = (uint32_t *)strstr((char *)public_buf, ";;simage:");
                  //                   uint32_t *p3 = (uint32_t *)strstr((char *)public_buf, ";00000");
                  //                   uint32_t *p4 = (uint32_t *)strstr((char *)public_buf, ";;gimage:");

                  //                   //if(p2)

                  //                   if( (p1==0)&& (p2==0) && (p4==0) && p3)
                  //                   {
                  //                     LCD_SERIAL_2.printf("printpause.cp0.aph=127");
                  //                     LCD_SERIAL_2.printf("\xff\xff\xff");

                  //                     LCD_SERIAL_2.printf("printpause.cp0.write(printpause.va1.txt)");
                  //                     LCD_SERIAL_2.printf("\xff\xff\xff");
                  //                     break;
                  //                   }

                  //                   if(p2)
                  //                   {
                  //                     LCD_SERIAL_2.printf("printpause.va0.txt=");
                  //                     LCD_SERIAL_2.write(0x22);
                  //                     LCD_SERIAL_2.write(&public_buf[9],1023-9);
                  //                     LCD_SERIAL_2.write(0x22);
                  //                     LCD_SERIAL_2.printf("\xff\xff\xff");
                  //                     LCD_SERIAL_2.printf("printpause.va1.txt+=printpause.va0.txt");
                  //                     LCD_SERIAL_2.printf("\xff\xff\xff");                                     

                  //                     LCD_SERIAL_2.printf("printpause.cp0.aph=127");
                  //                     LCD_SERIAL_2.printf("\xff\xff\xff");

                  //                     TERN_(USE_WATCHDOG, hal.watchdog_refresh());
                  //                     delay(200);
                  //                     TERN_(USE_WATCHDOG, hal.watchdog_refresh());

                  //                     LCD_SERIAL_2.printf("printpause.cp0.write(printpause.va1.txt)");
                  //                     LCD_SERIAL_2.printf("\xff\xff\xff");
                  //                     break;
                  //                   }

                  //                   if(p1)
                  //                   {
                  //                     LCD_SERIAL_2.printf("printpause.va0.txt=");
                  //                     LCD_SERIAL_2.write(0x22);
                  //                     LCD_SERIAL_2.write(&public_buf[8],1023-8);
                  //                     LCD_SERIAL_2.write(0x22);
                  //                     LCD_SERIAL_2.printf("\xff\xff\xff");
                  //                     LCD_SERIAL_2.printf("printpause.va1.txt+=printpause.va0.txt");
                  //                     LCD_SERIAL_2.printf("\xff\xff\xff");

                  //                     TERN_(USE_WATCHDOG, hal.watchdog_refresh());
                  //                     delay(200);
                  //                     TERN_(USE_WATCHDOG, hal.watchdog_refresh());
                  //                   }
                  //                 }

                  //                 break;
                  //               }
                  //             }

                  //             if(cnt_pre>=102400)  break;
                  //           }

                  //         }

                  //         if(cnt_pre>=102400)  break;

                  //       }


                  //       file.close();
                  //       delay(20);
                  //     }

                  //     file.close();
                  //   }
                  // #endif                  

                  break;
                }
              }
            #endif
          }
          else if((power_off_type_yes == 0) && (!recovery.info.valid()))
          {
            uint8_t count_startprogress = 0;
            power_off_type_yes = 1;

            #if ENABLED(RTS_AVAILABLE)
              rtscheck.RTS_SndData(ExchangePageBase, ExchangepageAddr);
              for(count_startprogress=0;count_startprogress<=100;count_startprogress++)
              {
                rtscheck.RTS_SndData(count_startprogress, START1_PROCESS_ICON_VP);
                delay(30);
              }
              rtscheck.RTS_SndData(StartSoundSet, SoundAddr);
              rtscheck.RTS_SndData(ExchangePageBase + 1, ExchangepageAddr);
            #endif

            #if ENABLED(TJC_AVAILABLE)

              LCD_SERIAL_2.printf("page boot");
              LCD_SERIAL_2.printf("\xff\xff\xff");

              #if ENABLED(NEPTUNE_3_PLUS)
                LCD_SERIAL_2.printf("main.va0.val=2");  
                LCD_SERIAL_2.printf("\xff\xff\xff");
              #elif ENABLED(NEPTUNE_3_PRO)
                LCD_SERIAL_2.printf("main.va0.val=1");  
                LCD_SERIAL_2.printf("\xff\xff\xff");                  
              #elif ENABLED(NEPTUNE_3_MAX)
                LCD_SERIAL_2.printf("main.va0.val=3");  
                LCD_SERIAL_2.printf("\xff\xff\xff");
              #endif

              // 进度条
              for(count_startprogress=0;count_startprogress<=100;count_startprogress++)
              {
                char send[16]={0};
                sprintf(send, "boot.j0.val=%d", count_startprogress);
                LCD_SERIAL_2.printf(send); 
                LCD_SERIAL_2.printf("\xff\xff\xff");
                delay(30);
                TERN_(USE_WATCHDOG, hal.watchdog_refresh());
              }

              LCD_SERIAL_2.printf("page main");
              LCD_SERIAL_2.printf("\xff\xff\xff");

            #endif

            Update_Time_Value = RTS_UPDATE_VALUE;  
          }

          #if ENABLED(TJC_AVAILABLE)
            #if ENABLED(POWER_LOSS_RECOVERY)
              if(recovery.enabled==0)
              { 
                LCD_SERIAL_2.printf("multiset.plrbutton.val=0");
                LCD_SERIAL_2.printf("\xff\xff\xff"); 
              }
              else if(recovery.enabled==1)
              {
                LCD_SERIAL_2.printf("multiset.plrbutton.val=1");
                LCD_SERIAL_2.printf("\xff\xff\xff"); 
              }
            #endif              
          #endif               
        }
      #else
        if(flag_power_on)
        {
          flag_power_on = false;
          uint8_t count_startprogress = 0;
          rtscheck.RTS_SndData(ExchangePageBase, ExchangepageAddr);
          for(count_startprogress=0;count_startprogress<=100;count_startprogress++)
          {
            rtscheck.RTS_SndData(count_startprogress, START1_PROCESS_ICON_VP);
            delay(30);
          }
          rtscheck.RTS_SndData(StartSoundSet, SoundAddr);
          power_off_type_yes = 1;
          Update_Time_Value = RTS_UPDATE_VALUE;
          rtscheck.RTS_SndData(ExchangePageBase + 1, ExchangepageAddr);

          #if ENABLED(TJC_AVAILABLE)
            LCD_SERIAL_2.printf("page boot");
            LCD_SERIAL_2.printf("\xff\xff\xff");

            #if ENABLED(NEPTUNE_3_PLUS)
              LCD_SERIAL_2.printf("main.va0.val=2");  
              LCD_SERIAL_2.printf("\xff\xff\xff");
            #elif ENABLED(NEPTUNE_3_PRO)
              LCD_SERIAL_2.printf("main.va0.val=1");  
              LCD_SERIAL_2.printf("\xff\xff\xff");                  
            #elif ENABLED(NEPTUNE_3_MAX)
              LCD_SERIAL_2.printf("main.va0.val=3");  
              LCD_SERIAL_2.printf("\xff\xff\xff");
            #endif

            // 进度条
            for(count_startprogress=0;count_startprogress<=100;count_startprogress++)
            {
              char send[16]={0};
              sprintf(send, "boot.j0.val=%d", count_startprogress);
              LCD_SERIAL_2.printf(send); 
              LCD_SERIAL_2.printf("\xff\xff\xff");
              delay(30);
              TERN_(USE_WATCHDOG, hal.watchdog_refresh());
            }

            LCD_SERIAL_2.printf("page main");
            LCD_SERIAL_2.printf("\xff\xff\xff");

          #endif

          #if ENABLED(TJC_AVAILABLE)
            #if ENABLED(POWER_LOSS_RECOVERY)
              if(recovery.enabled==0)
              { 
                LCD_SERIAL_2.printf("multiset.plrbutton.val=0");
                LCD_SERIAL_2.printf("\xff\xff\xff"); 
              }
              else if(recovery.enabled==1)
              {
                LCD_SERIAL_2.printf("multiset.plrbutton.val=1");
                LCD_SERIAL_2.printf("\xff\xff\xff"); 
              }
            #endif              
          #endif            
        }
      #endif

      #if PIN_EXISTS(LED2)
        if(flag_led1_run_ctrl)
        {
          if(current_position.z >= 10)
          {
            OUT_WRITE(LED2_PIN, LOW);
            status_led1 = false;
            #if ENABLED(TJC_AVAILABLE) 
              LCD_SERIAL_2.printf("status_led1=0");
              LCD_SERIAL_2.printf("\xff\xff\xff");
            #endif
          }
          else if(printJobOngoing())
          {
            OUT_WRITE(LED2_PIN, HIGH);
            status_led1 = true;
            #if ENABLED(TJC_AVAILABLE) 
              LCD_SERIAL_2.printf("status_led1=1");
              LCD_SERIAL_2.printf("\xff\xff\xff");
            #endif
          }
          else
          {
            OUT_WRITE(LED2_PIN, LOW);
            status_led1 = false;
            #if ENABLED(TJC_AVAILABLE) 
              LCD_SERIAL_2.printf("status_led1=0");
              LCD_SERIAL_2.printf("\xff\xff\xff");
            #endif
          }
        }
      #endif

      #if ENABLED(TJC_AVAILABLE)

        if(enable_filment_check)
        {
          LCD_SERIAL_2.printf("set.va1.val=1");
          LCD_SERIAL_2.printf("\xff\xff\xff");            
        }
        else
        {
          LCD_SERIAL_2.printf("set.va1.val=0");
          LCD_SERIAL_2.printf("\xff\xff\xff");                 
        }

        if (thermalManager.fan_speed[0])
        {
          LCD_SERIAL_2.printf("set.va0.val=1");
          LCD_SERIAL_2.printf("\xff\xff\xff");                
        }
        else
        {
          LCD_SERIAL_2.printf("set.va0.val=0");
          LCD_SERIAL_2.printf("\xff\xff\xff");               
        }

      #endif

      #if ENABLED(TJC_AVAILABLE)
        #if ENABLED(POWER_LOSS_RECOVERY)
          if(recovery.enabled==0)
          { 
            LCD_SERIAL_2.printf("multiset.plrbutton.val=0");
            LCD_SERIAL_2.printf("\xff\xff\xff"); 
          }
          else if(recovery.enabled==1)
          {
            LCD_SERIAL_2.printf("multiset.plrbutton.val=1");
            LCD_SERIAL_2.printf("\xff\xff\xff"); 
          }
        #endif              
      #endif

      if(!flag_power_on)
      {
        // need to optimize
        #if ENABLED(POWER_LOSS_RECOVERY)
        //if(recovery.info.print_job_elapsed != 0)
        #endif
        {
          duration_t elapsed = print_job_timer.duration();

          #if ENABLED(RTS_AVAILABLE) 
            rtscheck.RTS_SndData(elapsed.value / 3600, PRINT_TIME_HOUR_VP);
            rtscheck.RTS_SndData((elapsed.value % 3600) / 60, PRINT_TIME_MIN_VP);
          #endif

          if(card.isPrinting() && (last_cardpercentValue != card.percentDone()))
          {
            if((unsigned char) card.percentDone() >= 0)
            {
              Percentrecord = card.percentDone();
              if(Percentrecord <= 100)
              {
                #if ENABLED(RTS_AVAILABLE) 
                  rtscheck.RTS_SndData((unsigned char)Percentrecord, PRINT_PROCESS_ICON_VP);
                #endif
              }

              // 剩余时间
              // Estimate remaining time every 20 seconds
              static millis_t next_remain_time_update = 0;
              if(ELAPSED(ms, next_remain_time_update))
              {
                if((0 == save_dual_x_carriage_mode) && (thermalManager.temp_hotend[0].celsius >= (thermalManager.temp_hotend[0].target - 5)))
                {
                  remain_time = elapsed.value / (Percentrecord * 0.01f) - elapsed.value;
                  next_remain_time_update += 20 * 1000UL;

                  #if ENABLED(RTS_AVAILABLE) 
                    rtscheck.RTS_SndData(remain_time / 3600, PRINT_SURPLUS_TIME_HOUR_VP);
                    rtscheck.RTS_SndData((remain_time % 3600) / 60, PRINT_SURPLUS_TIME_MIN_VP);
                  #endif                
                }
                #if ENABLED(DUAL_X_CARRIAGE)
                else if((0 != save_dual_x_carriage_mode) && (thermalManager.temp_hotend[0].celsius >= (thermalManager.temp_hotend[0].target - 5)) && (thermalManager.temp_hotend[1].celsius >= (thermalManager.temp_hotend[1].target - 5)))
                {
                  remain_time = elapsed.value / (Percentrecord * 0.01f) - elapsed.value;
                  next_remain_time_update += 20 * 1000UL;

                  #if ENABLED(RTS_AVAILABLE) 
                    rtscheck.RTS_SndData(remain_time / 3600, PRINT_SURPLUS_TIME_HOUR_VP);
                    rtscheck.RTS_SndData((remain_time % 3600) / 60, PRINT_SURPLUS_TIME_MIN_VP);
                  #endif      
                }
                #endif
              }
            }
            else
            {
              #if ENABLED(RTS_AVAILABLE) 
                rtscheck.RTS_SndData(0, PRINT_PROCESS_ICON_VP);
                rtscheck.RTS_SndData(0, PRINT_SURPLUS_TIME_HOUR_VP);
                rtscheck.RTS_SndData(0, PRINT_SURPLUS_TIME_MIN_VP);
              #endif
            }

            last_cardpercentValue = card.percentDone();

            #if ENABLED(RTS_AVAILABLE) 
              rtscheck.RTS_SndData((unsigned char)card.percentDone(), PRINT_PROCESS_VP);
              rtscheck.RTS_SndData(10 * current_position[Z_AXIS], AXIS_Z_COORD_VP);
            #endif 
          }
        }

        if(pause_action_flag && (false == sdcard_pause_check) && printingIsPaused() && !planner.has_blocks_queued())
        {
          pause_action_flag = false;
          if((1 == active_extruder) && (1 == save_dual_x_carriage_mode))
          {
            queue.enqueue_now_P(PSTR("G0 F3000 X350 Y0"));
          }
          else
          {
            queue.enqueue_now_P(PSTR("G0 F3000 X-50 Y0"));
          }
        }

        #if ENABLED(RTS_AVAILABLE) 
          rtscheck.RTS_SndData(thermalManager.temp_hotend[0].celsius, HEAD0_CURRENT_TEMP_VP);
          #if ENABLED(DUAL_X_CARRIAGE)
            rtscheck.RTS_SndData(thermalManager.temp_hotend[1].celsius, HEAD1_CURRENT_TEMP_VP);
          #endif
          rtscheck.RTS_SndData(thermalManager.temp_bed.celsius, BED_CURRENT_TEMP_VP);

          #if ENABLED(TJC_AVAILABLE) 
            //挤出头温度信息
            memset(temp,0,sizeof(temp));
            sprintf(temp, "main.nozzletemp.txt=\"%d / %d\"", thermalManager.wholeDegHotend(0) , thermalManager.degTargetHotend(0));
            LCD_SERIAL_2.printf(temp);
            LCD_SERIAL_2.printf("\xff\xff\xff");

            //热床温度信息
            memset(temp,0,sizeof(temp));
            sprintf(temp, "main.bedtemp.txt=\"%d / %d\"", thermalManager.wholeDegBed() , thermalManager.degTargetBed());
            LCD_SERIAL_2.printf(temp);
            LCD_SERIAL_2.printf("\xff\xff\xff");
          #endif

        #endif

        #if ENABLED(SDSUPPORT)
          if((sd_printing_autopause == true) && (PoweroffContinue == true))
          {
            if(true == sdcard_pause_check)
            {
              if(!CardReader::flag.mounted)
              {
                pause_z = current_position[Z_AXIS];
                pause_e = current_position[E_AXIS] - 3;
                card.pauseSDPrint();
                print_job_timer.pause();
                planner.synchronize();
                Update_Time_Value = 0;
                if((1 == active_extruder) && (1 == save_dual_x_carriage_mode))
                {
                  queue.enqueue_now_P(PSTR("G1 F3000 X350 Y0"));
                }
                else
                {
                  queue.enqueue_now_P(PSTR("G1 F3000 X-50 Y0"));
                }
                sdcard_pause_check = false;

                #if ENABLED(RTS_AVAILABLE) 
                  rtscheck.RTS_SndData(ExchangePageBase + 46, ExchangepageAddr);
                #endif 

                #if ENABLED(TJC_AVAILABLE)   

                #endif 
              }
            }
          }

          if((false == sdcard_pause_check) && (false == card.isPrinting()) && !planner.has_blocks_queued())
          {
            #if ENABLED(RTS_AVAILABLE) 
              if(CardReader::flag.mounted)
              {
                rtscheck.RTS_SndData(1, CHANGE_SDCARD_ICON_VP);
              }
              else
              {
                rtscheck.RTS_SndData(0, CHANGE_SDCARD_ICON_VP);
              }
            #endif

            #if ENABLED(TJC_AVAILABLE)   

            #endif             
          }
        #endif
        #if ENABLED(DUAL_X_CARRIAGE)
        if( (last_target_temperature[0] != thermalManager.temp_hotend[0].target) || (last_target_temperature[1] != thermalManager.temp_hotend[1].target) || (last_target_temperature_bed != thermalManager.temp_bed.target))
        #else
        if( (last_target_temperature[0] != thermalManager.temp_hotend[0].target) || (last_target_temperature_bed != thermalManager.temp_bed.target))
        #endif
        {

          thermalManager.setTargetHotend(thermalManager.temp_hotend[0].target, 0);
          thermalManager.setTargetBed(thermalManager.temp_bed.target);
          last_target_temperature[0] = thermalManager.temp_hotend[0].target;

          #if ENABLED(RTS_AVAILABLE)
            rtscheck.RTS_SndData(thermalManager.temp_hotend[0].target, HEAD0_SET_TEMP_VP);
            rtscheck.RTS_SndData(thermalManager.temp_bed.target, BED_SET_TEMP_VP);
          #endif
          
          #if ENABLED(TJC_AVAILABLE)   

          #endif   

          #if ENABLED(DUAL_X_CARRIAGE)
            thermalManager.setTargetHotend(thermalManager.temp_hotend[1].target, 1);
            last_target_temperature[1] = thermalManager.temp_hotend[1].target;
            rtscheck.RTS_SndData(thermalManager.temp_hotend[1].target, HEAD1_SET_TEMP_VP);
          #endif
          
          last_target_temperature_bed = thermalManager.temp_bed.target;
        }

        if((thermalManager.temp_hotend[0].celsius >= (thermalManager.temp_hotend[0].target-5)) && (heatway == 1))
        {
          #if ENABLED(RTS_AVAILABLE)
            if(printingIsPaused())
            {
              rtscheck.RTS_SndData(ExchangePageBase + 16, ExchangepageAddr); //新UI处理
              #if ENABLED(TJC_AVAILABLE) 
                LCD_SERIAL_2.printf("page adjusttemp");
                LCD_SERIAL_2.printf("\xff\xff\xff");               
              #endif
            }
            else
            {
              rtscheck.RTS_SndData(ExchangePageBase + 31, ExchangepageAddr); //新UI处理
              #if ENABLED(TJC_AVAILABLE) 
                LCD_SERIAL_2.printf("page prefilament");
                LCD_SERIAL_2.printf("\xff\xff\xff");               
              #endif
            }
            rtscheck.RTS_SndData(10 * Filament0LOAD, HEAD0_FILAMENT_LOAD_DATA_VP);
          #endif

          heatway = 0;
        }
      #if ENABLED(DUAL_X_CARRIAGE)
        else if((thermalManager.temp_hotend[1].celsius >= thermalManager.temp_hotend[1].target) && (heatway == 2))
        {
          #if ENABLED(RTS_AVAILABLE)
            if(printingIsPaused())
            {
              rtscheck.RTS_SndData(ExchangePageBase + 16, ExchangepageAddr); //新UI处理
              #if ENABLED(TJC_AVAILABLE) 
                LCD_SERIAL_2.printf("page adjusttemp");
                LCD_SERIAL_2.printf("\xff\xff\xff");               
              #endif
            }
            else
            {
              rtscheck.RTS_SndData(ExchangePageBase + 31, ExchangepageAddr); //新UI处理
              #if ENABLED(TJC_AVAILABLE) 
                LCD_SERIAL_2.printf("page prefilament");
                LCD_SERIAL_2.printf("\xff\xff\xff");               
              #endif              
            }
            rtscheck.RTS_SndData(10 * Filament1LOAD, HEAD1_FILAMENT_LOAD_DATA_VP);
          #endif

          heatway = 0;
        }
      #endif
        
        if(enable_filment_check)
        {
          #if ENABLED(CHECKFILEMENT)

            #if ENABLED(DUAL_X_CARRIAGE)
              if((0 == save_dual_x_carriage_mode) && (0 == READ(CHECKFILEMENT0_PIN)) && (active_extruder == 0))
              {
                rtscheck.RTS_SndData(0, CHANGE_FILAMENT_ICON_VP);
              }
              else if((0 == save_dual_x_carriage_mode) && (1 == READ(CHECKFILEMENT0_PIN)) && (active_extruder == 0))
              {
                rtscheck.RTS_SndData(1, CHANGE_FILAMENT_ICON_VP);
              }
              else if((0 == save_dual_x_carriage_mode) && (0 == READ(CHECKFILEMENT1_PIN)) && (active_extruder == 1))
              {
                rtscheck.RTS_SndData(0, CHANGE_FILAMENT_ICON_VP);
              }
              else if((0 == save_dual_x_carriage_mode) && (1 == READ(CHECKFILEMENT1_PIN)) && (active_extruder == 1))
              {
                rtscheck.RTS_SndData(1, CHANGE_FILAMENT_ICON_VP);
              }
              else if((0 != save_dual_x_carriage_mode) && ((0 == READ(CHECKFILEMENT0_PIN)) || (0 == READ(CHECKFILEMENT1_PIN))))
              {
                rtscheck.RTS_SndData(0, CHANGE_FILAMENT_ICON_VP);
              }
              else if((0 != save_dual_x_carriage_mode) && (1 == READ(CHECKFILEMENT0_PIN)) && (1 == READ(CHECKFILEMENT1_PIN)))
              {
                rtscheck.RTS_SndData(1, CHANGE_FILAMENT_ICON_VP);
              }
              
            #else

              #if ENABLED(RTS_AVAILABLE)
                if(0 == READ(CHECKFILEMENT0_PIN))
                {
                  rtscheck.RTS_SndData(0, CHANGE_FILAMENT_ICON_VP);
                }
                else if(1 == READ(CHECKFILEMENT0_PIN))
                {
                  rtscheck.RTS_SndData(1, CHANGE_FILAMENT_ICON_VP);
                }
              #endif

              #if ENABLED(TJC_AVAILABLE) 

              #endif

            #endif

          #endif
        }
        #if ENABLED(RTS_AVAILABLE)
          rtscheck.RTS_SndData(AutoHomeIconNum ++, AUTO_HOME_DISPLAY_ICON_VP);
        #endif

        #if ENABLED(TJC_AVAILABLE) 

        #endif

        if (AutoHomeIconNum > 8)
        {
          AutoHomeIconNum = 0;
        }
      }
      next_rts_update_ms = ms + RTS_UPDATE_INTERVAL + Update_Time_Value;
    }
  }


  void RTSUpdate()
  {
    static bool start_update;
    static bool first_check = true;
    static uint32_t update_time;
    
    rtscheck.RTS_SDCardUpate(); // Check the status of card

    if(enable_filment_check  && IS_SD_PRINTING())
    {
      #if ENABLED(CHECKFILEMENT)
         
        // checking filement status during printing
        opos = npos;
        npos = planner.get_axis_position_mm(E_AXIS);
        
        if((true == card.isPrinting()) && (true == PoweroffContinue) && (opos!=npos))
        {
          #if ENABLED(DUAL_X_CARRIAGE)
            if((0 == save_dual_x_carriage_mode) && (0 == READ(CHECKFILEMENT0_PIN)) && (active_extruder == 0))
            {
              Checkfilenum ++;
              delay(5);
            }
            else if((0 == save_dual_x_carriage_mode) && (0 == READ(CHECKFILEMENT1_PIN)) && (active_extruder == 1))
            {
              Checkfilenum ++;
              delay(5);
            }
            else if((0 != save_dual_x_carriage_mode) && ((0 == READ(CHECKFILEMENT0_PIN)) || (0 == READ(CHECKFILEMENT1_PIN))))
            {
              Checkfilenum ++;
              delay(5);
            }
            else
            {
              delay(5);
              if((0 == save_dual_x_carriage_mode) && (0 == READ(CHECKFILEMENT0_PIN)) && (active_extruder == 0))
              {
                Checkfilenum ++;
              }
              else if((0 == save_dual_x_carriage_mode) && (0 == READ(CHECKFILEMENT1_PIN)) && (active_extruder == 1))
              {
                Checkfilenum ++;
              }
              else if((0 != save_dual_x_carriage_mode) && ((0 == READ(CHECKFILEMENT0_PIN)) || (0 == READ(CHECKFILEMENT1_PIN))))
              {
                Checkfilenum ++;
              }
              else
              {
                Checkfilenum = 0;
              }
            }
          #else
            {
              //if(0 == READ(CHECKFILEMENT0_PIN))
              if( (0 == READ(CHECKFILEMENT0_PIN)) ||  RTS_M600_Flag)
              {
                Checkfilenum++;
                delay(5);
              }
              else
              {
                Checkfilenum = 0;
              }
            }
          #endif

          if(Checkfilenum > 10)
          {
            pause_z = current_position[Z_AXIS];
            pause_e = current_position[E_AXIS] - 3;

            #if ENABLED(DUAL_X_CARRIAGE)
              if((0 == save_dual_x_carriage_mode) && (thermalManager.temp_hotend[0].celsius <= (thermalManager.temp_hotend[0].target - 5)))
              {
                rtscheck.RTS_SndData(ExchangePageBase + 39, ExchangepageAddr);
                card.pauseSDPrint();
                print_job_timer.pause();

                pause_action_flag = true;
                Checkfilenum = 0;
                Update_Time_Value = 0;
                sdcard_pause_check = false;
                print_preheat_check = true;
              }
              else if((0 != save_dual_x_carriage_mode) && ((thermalManager.temp_hotend[0].celsius <= (thermalManager.temp_hotend[0].target - 5)) || (thermalManager.temp_hotend[1].celsius <= (thermalManager.temp_hotend[1].target - 5))))
              {
                rtscheck.RTS_SndData(ExchangePageBase + 39, ExchangepageAddr);
                card.pauseSDPrint();
                print_job_timer.pause();

                pause_action_flag = true;
                Checkfilenum = 0;
                Update_Time_Value = 0;
                sdcard_pause_check = false;
                print_preheat_check = true;
              }
              else if(thermalManager.temp_bed.celsius <= thermalManager.temp_bed.target)
              {
                rtscheck.RTS_SndData(ExchangePageBase + 39, ExchangepageAddr);
                card.pauseSDPrint();
                print_job_timer.pause();

                pause_action_flag = true;
                Checkfilenum = 0;
                Update_Time_Value = 0;
                sdcard_pause_check = false;
                print_preheat_check = true;
              }
              else if((!TEST(axis_known_position, X_AXIS)) || (!TEST(axis_known_position, Y_AXIS)) || (!TEST(axis_known_position, Z_AXIS)))
              {
                rtscheck.RTS_SndData(ExchangePageBase + 39, ExchangepageAddr);
                card.pauseSDPrint();
                print_job_timer.pause();

                pause_action_flag = true;
                Checkfilenum = 0;
                Update_Time_Value = 0;
                sdcard_pause_check = false;
                print_preheat_check = true;
              }
              else
              {
                rtscheck.RTS_SndData(ExchangePageBase + 40, ExchangepageAddr);
                #if ENABLED(TJC_AVAILABLE) 
                  LCD_SERIAL_2.printf("page wait");
                  LCD_SERIAL_2.printf("\xff\xff\xff");               
                #endif
                waitway = 5;

                #if ENABLED(POWER_LOSS_RECOVERY)
                  if (recovery.enabled)
                  {
                    recovery.save(true, false);
                  }
                #endif
                card.pauseSDPrint();
                print_job_timer.pause();

                pause_action_flag = true;
                Checkfilenum = 0;
                Update_Time_Value = 0;
                planner.synchronize();
                sdcard_pause_check = false;
              }
            #else
              {
                waitway = 5;

                #if ENABLED(RTS_AVAILABLE)
                  rtscheck.RTS_SndData(ExchangePageBase + 40, ExchangepageAddr);
                  #if ENABLED(TJC_AVAILABLE) 
                  LCD_SERIAL_2.printf("page wait");
                  LCD_SERIAL_2.printf("\xff\xff\xff");               
                  #endif
                #endif

                // #if ENABLED(POWER_LOSS_RECOVERY)
                //   if (recovery.enabled)
                //   {
                //     recovery.save(true, false);
                //   }
                // #endif

                // card.pauseSDPrint();
                // print_job_timer.pause();

                ExtUI::pausePrint();

                pause_action_flag  = true;
                Checkfilenum       = 0;
                Update_Time_Value  = 0;
                sdcard_pause_check = false;

                //planner.synchronize();              
              }
            #endif
          }
        }
      
      #endif
    }

    if(first_check)
    {
      first_check = false;
      update_time = millis();
    }

    if(millis() - update_time > 60)
    {
      start_update = true;
    }

    if(start_update)
    {
      EachMomentUpdate();
    }

  }

  float pause_opos = 0;
  float pause_npos = 0;
  uint8_t pause_count_pos = 0;

  void RTS_PauseMoveAxisPage()
  {
    #if ENABLED(RTS_AVAILABLE)
      if(waitway == 1)
      {
        rtscheck.RTS_SndData(ExchangePageBase + 12, ExchangepageAddr);
        #if ENABLED(TJC_AVAILABLE)
          LCD_SERIAL_2.printf("page printpause");
          LCD_SERIAL_2.printf("\xff\xff\xff");
        #endif
        waitway = 0;
      }
      else if(waitway == 5)
      {
        rtscheck.RTS_SndData(ExchangePageBase + 39, ExchangepageAddr);
        #if ENABLED(TJC_AVAILABLE)
          LCD_SERIAL_2.printf("page noFilamentPush");
          LCD_SERIAL_2.printf("\xff\xff\xff");
        #endif
        waitway = 0;
      }
      else if(waitway == 7)
      {
        // Click Print finish
        waitway = 0;

        #if ENABLED(RTS_AVAILABLE)
          rtscheck.RTS_SndData(ExchangePageBase + 1, ExchangepageAddr);
          #if ENABLED(TJC_AVAILABLE) 
              LCD_SERIAL_2.printf("page main");
              LCD_SERIAL_2.printf("\xff\xff\xff");            
          #endif
        #endif

        queue.enqueue_now_P(PSTR("M84"));
      }
      #endif

    #if ENABLED(TJC_AVAILABLE)
      if(card.isPrinting())
      {
        pause_opos = pause_npos;
        pause_npos = planner.get_axis_position_mm(X_AXIS);

        if((pause_opos!=pause_npos) && (pause_count_pos<1))
        {
          pause_count_pos++;
          #if ENABLED(TJC_AVAILABLE)
            LCD_SERIAL_2.printf("restFlag1=0");
            LCD_SERIAL_2.printf("\xff\xff\xff");
            LCD_SERIAL_2.printf("restFlag2=1");
            LCD_SERIAL_2.printf("\xff\xff\xff");
          #endif
          abortSD_flag = true;  
        }
      }
    #endif    
  }

  void RTS_AutoBedLevelPage()
  {
    if(waitway == 3)
    {
      waitway = 0;

      #if ENABLED(RTS_AVAILABLE)
        rtscheck.RTS_SndData(ExchangePageBase + 22, ExchangepageAddr);
      #endif

      #if ENABLED(TJC_AVAILABLE)

        #if ENABLED(NEPTUNE_3_PLUS)
          //LCD_SERIAL_2.printf("page leveldata_49");
          LCD_SERIAL_2.printf("page aux49_data");
          LCD_SERIAL_2.printf("\xff\xff\xff");
          LCD_SERIAL_2.printf("leveling_49.tm0.en=0");
          LCD_SERIAL_2.printf("\xff\xff\xff");
        #elif ENABLED(NEPTUNE_3_PRO)
          LCD_SERIAL_2.printf("page leveldata_36");
          LCD_SERIAL_2.printf("\xff\xff\xff");
          LCD_SERIAL_2.printf("leveling_36.tm0.en=0");
          LCD_SERIAL_2.printf("\xff\xff\xff");        
        #elif ENABLED(NEPTUNE_3_MAX)
          //LCD_SERIAL_2.printf("page leveldata_64");
          //LCD_SERIAL_2.printf("page aux64_data");
          LCD_SERIAL_2.printf("page aux63_data");
          LCD_SERIAL_2.printf("\xff\xff\xff");
          //LCD_SERIAL_2.printf("leveling_64.tm0.en=0");
          LCD_SERIAL_2.printf("leveling_63.tm0.en=0");
          LCD_SERIAL_2.printf("\xff\xff\xff"); 
        #endif

        LCD_SERIAL_2.printf("leveling.tm0.en=0");
        LCD_SERIAL_2.printf("\xff\xff\xff");

        LCD_SERIAL_2.printf("page warn_zoffset");
        LCD_SERIAL_2.printf("\xff\xff\xff");               
      #endif
    }
  }

  void RTS_MoveAxisHoming()
  {
    if(waitway == 4)
    {
      waitway = 0;
      #if ENABLED(RTS_AVAILABLE)
        rtscheck.RTS_SndData(ExchangePageBase + 29 + (AxisUnitMode - 1), ExchangepageAddr);
        #if ENABLED(TJC_AVAILABLE) 
          LCD_SERIAL_2.printf("page premove");
          LCD_SERIAL_2.printf("\xff\xff\xff");               
        #endif        
      #endif
    }
    else if(waitway == 6)
    {
      waitway = 0;

      #if ENABLED(RTS_AVAILABLE)
        #if ENABLED(HAS_LEVELING)
          rtscheck.RTS_SndData(ExchangePageBase + 22, ExchangepageAddr);
        #else
          rtscheck.RTS_SndData(ExchangePageBase + 28, ExchangepageAddr);
        #endif
      #endif
      
      #if ENABLED(TJC_AVAILABLE)

        #if ENABLED(NEPTUNE_3_PLUS)
          //LCD_SERIAL_2.printf("page leveldata_49");
          LCD_SERIAL_2.printf("page aux49_data");
          LCD_SERIAL_2.printf("\xff\xff\xff");
          LCD_SERIAL_2.printf("leveling_49.tm0.en=0");
          LCD_SERIAL_2.printf("\xff\xff\xff");  
        #elif ENABLED(NEPTUNE_3_PRO)
          LCD_SERIAL_2.printf("page leveldata_36");
          LCD_SERIAL_2.printf("\xff\xff\xff");
          LCD_SERIAL_2.printf("leveling_36.tm0.en=0");
          LCD_SERIAL_2.printf("\xff\xff\xff");  
        #elif ENABLED(NEPTUNE_3_MAX)
          //LCD_SERIAL_2.printf("page leveldata_64");
          //LCD_SERIAL_2.printf("page aux64_data");
          LCD_SERIAL_2.printf("page aux63_data");
          LCD_SERIAL_2.printf("\xff\xff\xff");
          //LCD_SERIAL_2.printf("leveling_64.tm0.en=0");
          LCD_SERIAL_2.printf("leveling_63.tm0.en=0");
          LCD_SERIAL_2.printf("\xff\xff\xff");  
        #endif

        LCD_SERIAL_2.printf("leveling.tm0.en=0");
        LCD_SERIAL_2.printf("\xff\xff\xff");  
      #endif      
    }
    else if(waitway == 7)
    {
      // Click Print finish
      waitway = 0;

      #if ENABLED(RTS_AVAILABLE)
        rtscheck.RTS_SndData(ExchangePageBase + 1, ExchangepageAddr);
        #if ENABLED(TJC_AVAILABLE) 
            LCD_SERIAL_2.printf("page main");
            LCD_SERIAL_2.printf("\xff\xff\xff");            
        #endif
      #endif

      queue.enqueue_now_P(PSTR("M84"));

      #if ENABLED(TJC_AVAILABLE)

      #endif   
    }

    #if ENABLED(RTS_AVAILABLE)
      if(active_extruder == 0)
      {
        rtscheck.RTS_SndData(0, EXCHANGE_NOZZLE_ICON_VP);
      }
      else
      {
        rtscheck.RTS_SndData(1, EXCHANGE_NOZZLE_ICON_VP);
      }

      rtscheck.RTS_SndData(10*current_position[X_AXIS], AXIS_X_COORD_VP);
      rtscheck.RTS_SndData(10*current_position[Y_AXIS], AXIS_Y_COORD_VP);
      rtscheck.RTS_SndData(10*current_position[Z_AXIS], AXIS_Z_COORD_VP);
    #endif

    #if ENABLED(TJC_AVAILABLE)

    #endif
  }

  void RTSSHOW::RTS_HandleData()
  {
    int Checkkey = -1;
    // for waiting
    if(waitway > 0)
    {
      memset(&recdat, 0, sizeof(recdat));
      recdat.head[0] = FHONE;
      recdat.head[1] = FHTWO;
      return;
    }

    for(int i = 0;Addrbuf[i] != 0;i ++)
    {
      if(recdat.addr == Addrbuf[i])
      {
        if(Addrbuf[i] >= ChangePageKey)
        {
          Checkkey = i;
        }
        break;
      }
    }

    if(Checkkey < 0)
    {
      memset(&recdat, 0, sizeof(recdat));
      recdat.head[0] = FHONE;
      recdat.head[1] = FHTWO;
      return;
    }

    switch(Checkkey)
    {
      case MainPageKey:
      {
        if(recdat.data[0] == 1)
        {
          CardUpdate = true;
          CardRecbuf.recordcount = -1;
          RTS_SDCardUpate();

          if(CardReader::flag.mounted)
          {
            RTS_SndData(ExchangePageBase + 2, ExchangepageAddr);

            #if ENABLED(TJC_AVAILABLE)
              LCD_SERIAL_2.printf("page file1");
              LCD_SERIAL_2.printf("\xff\xff\xff");
            #endif
          }
          else
          {
            RTS_SndData(ExchangePageBase + 47, ExchangepageAddr);

            #if ENABLED(TJC_AVAILABLE)
              LCD_SERIAL_2.printf("page nosdcard");
              LCD_SERIAL_2.printf("\xff\xff\xff");
            #endif
          }
        }
        else if(recdat.data[0] == 2)
        {
          card.flag.abort_sd_printing = true;
          queue.clear();
          quickstop_stepper();
          print_job_timer.stop();
          RTS_SndData(0, MOTOR_FREE_ICON_VP);
          RTS_SndData(0, PRINT_PROCESS_ICON_VP);
          RTS_SndData(0, PRINT_PROCESS_VP);
          delay(2);
          RTS_SndData(0, PRINT_TIME_HOUR_VP);
          RTS_SndData(0, PRINT_TIME_MIN_VP);
          RTS_SndData(0, PRINT_SURPLUS_TIME_HOUR_VP);
          RTS_SndData(0, PRINT_SURPLUS_TIME_MIN_VP);
          print_job_timer.reset();
          sd_printing_autopause = false;
          //RTS_SndData(ExchangePageBase + 1, ExchangepageAddr);
        }
        else if(recdat.data[0] == 3)
        {
          thermalManager.fan_speed[0] ? RTS_SndData(0, HEAD0_FAN_ICON_VP) : RTS_SndData(1, HEAD0_FAN_ICON_VP);
          #if HAS_FAN1
            thermalManager.fan_speed[1] ? RTS_SndData(0, HEAD1_FAN_ICON_VP) : RTS_SndData(1, HEAD1_FAN_ICON_VP);
          #endif
          RTS_SndData(ExchangePageBase + 15, ExchangepageAddr);
        }
        else if(recdat.data[0] == 4)
        {
          RTS_SndData(ExchangePageBase + 21, ExchangepageAddr);

          if(enable_filment_check)
          {
            RTS_SndData(1, ICON_FILMENT_DETACT);
          }
          else
          {
            RTS_SndData(0, ICON_FILMENT_DETACT);
          }

        }
        else if(recdat.data[0] == 5)
        {
          #if ENABLED(DUAL_X_CARRIAGE)
            save_dual_x_carriage_mode = dualXPrintingModeStatus;
            if(save_dual_x_carriage_mode == 1)
            {
              RTS_SndData(1, TWO_COLOR_MODE_ICON_VP);
              RTS_SndData(0, COPY_MODE_ICON_VP);
              RTS_SndData(0, MIRROR_MODE_ICON_VP);
              RTS_SndData(0, SINGLE_MODE_ICON_VP);

              RTS_SndData(1, PRINT_MODE_ICON_VP);
              RTS_SndData(1, SELECT_MODE_ICON_VP);
            }
            else if(save_dual_x_carriage_mode == 2)
            {
              RTS_SndData(0, TWO_COLOR_MODE_ICON_VP);
              RTS_SndData(1, COPY_MODE_ICON_VP);
              RTS_SndData(0, MIRROR_MODE_ICON_VP);
              RTS_SndData(0, SINGLE_MODE_ICON_VP);

              RTS_SndData(2, PRINT_MODE_ICON_VP);
              RTS_SndData(2, SELECT_MODE_ICON_VP);
            }
            else if(save_dual_x_carriage_mode == 3)
            {
              RTS_SndData(0, TWO_COLOR_MODE_ICON_VP);
              RTS_SndData(0, COPY_MODE_ICON_VP);
              RTS_SndData(1, MIRROR_MODE_ICON_VP);
              RTS_SndData(0, SINGLE_MODE_ICON_VP);

              RTS_SndData(3, PRINT_MODE_ICON_VP);
              RTS_SndData(3, SELECT_MODE_ICON_VP);
            }
            else
            {
              RTS_SndData(0, TWO_COLOR_MODE_ICON_VP);
              RTS_SndData(0, COPY_MODE_ICON_VP);
              RTS_SndData(0, MIRROR_MODE_ICON_VP);
              RTS_SndData(1, SINGLE_MODE_ICON_VP);

              RTS_SndData(4, PRINT_MODE_ICON_VP);
              RTS_SndData(4, SELECT_MODE_ICON_VP);
            }

            RTS_SndData(ExchangePageBase + 34, ExchangepageAddr);
          #endif
        }
      }
      break;
 
      case AdjustmentKey: //0x1004
      {
        if(recdat.data[0] == 1)
        {
          unit = 10;

          RTS_SndData(0, ICON_ADJUST_PRINTING_EXTRUDER_OR_BED); //默认为喷头界面
          RTS_SndData(2, ICON_ADJUST_PRINTING_TEMP_UNIT);       //默认单位调整为10
          RTS_SndData(thermalManager.temp_hotend[0].target, SPEED_SET_VP);

          #if ENABLED(TJC_AVAILABLE)
            temp_ctrl = 1;

            memset(temp,0,sizeof(temp));
            sprintf(temp, "adjusttemp.targettemp.val=%d", (int)(thermalManager.temp_hotend[0].target));
            LCD_SERIAL_2.printf(temp);
            LCD_SERIAL_2.printf("\xff\xff\xff");

            LCD_SERIAL_2.printf("adjusttemp.va0.val=1");  //默认为喷头界面
            LCD_SERIAL_2.printf("\xff\xff\xff");

            LCD_SERIAL_2.printf("adjusttemp.va1.val=3");  //默认单位调整为10
            LCD_SERIAL_2.printf("\xff\xff\xff");                       
          #endif 

        }
        else if(recdat.data[0] == 2)
        {
          if(printingIsPaused())
          {
            RTS_SndData(ExchangePageBase + 12, ExchangepageAddr);
            #if ENABLED(TJC_AVAILABLE)
              LCD_SERIAL_2.printf("page printpause");
              LCD_SERIAL_2.printf("\xff\xff\xff");
            #endif
          }
          else if(printJobOngoing())
          {
            RTS_SndData(ExchangePageBase + 11, ExchangepageAddr);
            #if ENABLED(TJC_AVAILABLE)
                LCD_SERIAL_2.printf("page printpause");
                LCD_SERIAL_2.printf("\xff\xff\xff");
            #endif
          }
          else
          {
            RTS_SndData(ExchangePageBase + 10, ExchangepageAddr);
            #if ENABLED(TJC_AVAILABLE)
                LCD_SERIAL_2.printf("page printpause");
                LCD_SERIAL_2.printf("\xff\xff\xff");
            #endif
          }
        }
        else if(recdat.data[0] == 3)
        {
          if (thermalManager.fan_speed[0])
          {
            RTS_SndData(1, HEAD0_FAN_ICON_VP);
            thermalManager.set_fan_speed(0, 0);
          }
          else
          {
            RTS_SndData(0, HEAD0_FAN_ICON_VP);
            thermalManager.set_fan_speed(0, 255);
          }
        }
        else if(recdat.data[0] == 4)
        {
          #if HAS_FAN1
            if (thermalManager.fan_speed[1])
            {
              RTS_SndData(1, HEAD1_FAN_ICON_VP);
              thermalManager.set_fan_speed(1, 0);
              #if PIN_EXISTS(LED3)
                OUT_WRITE(LED3_PIN, LOW);
              #endif
            }
            else
            {
              RTS_SndData(0, HEAD1_FAN_ICON_VP);
              thermalManager.set_fan_speed(1, 255);
              #if PIN_EXISTS(LED3)
                OUT_WRITE(LED3_PIN, HIGH);
              #endif
            }
          #endif
        }
        else if(recdat.data[0] == 5)
        {
          unit = 10;
          RTS_SndData(0, ICON_ADJUST_PRINTING_EXTRUDER_OR_BED); //默认为喷头界面
          RTS_SndData(2, ICON_ADJUST_PRINTING_TEMP_UNIT);       //默认单位调整为10
          RTS_SndData(ExchangePageBase + 16, ExchangepageAddr);
          #if ENABLED(TJC_AVAILABLE) 
            LCD_SERIAL_2.printf("page adjusttemp");
            LCD_SERIAL_2.printf("\xff\xff\xff");           
          #endif
        }
        else if(recdat.data[0] == 6)
        {
          unit = 10;
          speed_ctrl = 1;
          RTS_SndData(feedrate_percentage, HEAD1_SET_TEMP_VP);
          RTS_SndData(0, ICON_ADJUST_PRINTING_SPEED_FLOW); //默认为速度调整界面
          RTS_SndData(2, ICON_ADJUST_PRINTING_S_F_UNIT);   //默认单位调整为10
          RTS_SndData(ExchangePageBase + 17, ExchangepageAddr);

          #if ENABLED(TJC_AVAILABLE) 
            memset(temp,0,sizeof(temp));
            sprintf(temp, "adjustspeed.targetspeed.val=%d", (int)(feedrate_percentage));
            LCD_SERIAL_2.printf(temp);
            LCD_SERIAL_2.printf("\xff\xff\xff");           

            LCD_SERIAL_2.printf("page adjustspeed");
            LCD_SERIAL_2.printf("\xff\xff\xff");            
          #endif
        }
        else if(recdat.data[0] == 7)
        {
          zoffset_unit = 0.1;
          #if ENABLED(TJC_AVAILABLE) 
            LCD_SERIAL_2.printf("adjustzoffset.zoffset_value.val=2");
            LCD_SERIAL_2.printf("\xff\xff\xff");          
          #endif
          
          RTS_SndData(1, ICON_ADJUST_Z_OFFSET_UNIT); //默认单位为0.1mm
          RTS_SndData(1, ICON_LEVEL_SELECT);         //默认单位为0.1mm
          RTS_SndData(ExchangePageBase + 18, ExchangepageAddr);

          #if ENABLED(TJC_AVAILABLE) 
            memset(temp,0,sizeof(temp));
            sprintf(temp, "adjustzoffset.z_offset.val=%d", (int)(probe.offset.z * 100));
            LCD_SERIAL_2.printf(temp);
            LCD_SERIAL_2.printf("\xff\xff\xff");   

            LCD_SERIAL_2.printf("page adjustzoffset");
            LCD_SERIAL_2.printf("\xff\xff\xff");              
          #endif
        }
        else if(recdat.data[0] == 8)
        {
          #if ENABLED(TJC_AVAILABLE) 
            feedrate_percentage = 100;
            memset(temp,0,sizeof(temp));
            sprintf(temp, "adjustspeed.targetspeed.val=%d", (int)(feedrate_percentage));
            LCD_SERIAL_2.printf(temp);
            LCD_SERIAL_2.printf("\xff\xff\xff");  
          #endif
        }
        else if(recdat.data[0] == 9)
        {
          #if ENABLED(TJC_AVAILABLE)
            planner.flow_percentage[0] = 100; 
            memset(temp,0,sizeof(temp));
            sprintf(temp, "adjustspeed.targetspeed.val=%d", (int)(planner.flow_percentage[0]));
            LCD_SERIAL_2.printf(temp);
            LCD_SERIAL_2.printf("\xff\xff\xff");           
          #endif   
        }
        else if(recdat.data[0] == 0x0A)
        {
          thermalManager.fan_speed[0] = 255;
          #if ENABLED(TJC_AVAILABLE) 
            memset(temp,0,sizeof(temp));
            sprintf(temp, "adjustspeed.targetspeed.val=%d", (int)(thermalManager.fan_speed[0]));
            LCD_SERIAL_2.printf(temp);
            LCD_SERIAL_2.printf("\xff\xff\xff");           
          #endif             
        }
                
      }
      break;

      case PrintSpeedKey:
      {
        feedrate_percentage = recdat.data[0];
        RTS_SndData(feedrate_percentage, PRINT_SPEED_RATE_VP);
      }
      break;

      case StopPrintKey:
      {
        if((recdat.data[0] == 1) || (recdat.data[0] == 0xF1))
        {
          if(!Home_stop_flag)
          {
            RTS_SndData(ExchangePageBase + 40, ExchangepageAddr);
            #if ENABLED(TJC_AVAILABLE) 
              LCD_SERIAL_2.printf("page wait");
              LCD_SERIAL_2.printf("\xff\xff\xff");               
            #endif

            RTS_SndData(0, PRINT_TIME_HOUR_VP);
            RTS_SndData(0, PRINT_TIME_MIN_VP);
            RTS_SndData(0, PRINT_SURPLUS_TIME_HOUR_VP);
            RTS_SndData(0, PRINT_SURPLUS_TIME_MIN_VP);
            Update_Time_Value = 0;
            RTS_SDcard_Stop();
          }
        }
        else if(recdat.data[0] == 0xF0)
        {
          if(card.isPrinting)
          {
            RTS_SndData(ExchangePageBase + 11, ExchangepageAddr);
            #if ENABLED(TJC_AVAILABLE)
                LCD_SERIAL_2.printf("page printpause");
                LCD_SERIAL_2.printf("\xff\xff\xff");
            #endif            
          }
          else if(sdcard_pause_check == false)
          {
            RTS_SndData(ExchangePageBase + 12, ExchangepageAddr);
            #if ENABLED(TJC_AVAILABLE)
                LCD_SERIAL_2.printf("page printpause");
                LCD_SERIAL_2.printf("\xff\xff\xff");
            #endif  
          }
          else
          {
            RTS_SndData(ExchangePageBase + 10, ExchangepageAddr);
            #if ENABLED(TJC_AVAILABLE)
                LCD_SERIAL_2.printf("page printpause");
                LCD_SERIAL_2.printf("\xff\xff\xff");
            #endif  
          }
        }
      }
      break;

      case PausePrintKey:
      {
        if(recdat.data[0] == 0xF0)
        {
          break;
        }
        else if(recdat.data[0] == 0xF1)
        {
          RTS_SndData(ExchangePageBase + 40, ExchangepageAddr);
          #if ENABLED(TJC_AVAILABLE) 
            LCD_SERIAL_2.printf("page wait");
            LCD_SERIAL_2.printf("\xff\xff\xff");               
          #endif

          //reject to receive cmd
          waitway = 1;

          pause_z = current_position[Z_AXIS];
          pause_e = current_position[E_AXIS] - 3;

          //card.pauseSDPrint();
          //print_job_timer.pause();

          pause_action_flag = true;
          Update_Time_Value = 0;
          planner.synchronize();
          sdcard_pause_check = false;

          ExtUI::pausePrint();
        }
        else if(recdat.data[0] == 0x01)
        {
          if(IS_SD_PRINTING())
          {
            #if ENABLED(TJC_AVAILABLE)
              LCD_SERIAL_2.printf("page pauseconfirm");
              LCD_SERIAL_2.printf("\xff\xff\xff");  
            #endif
          }
        }
      }
      break;

      case ResumePrintKey:
      {
        if(recdat.data[0] == 1)
        {
          if(enable_filment_check)
          {
            #if ENABLED(CHECKFILEMENT)
              #if ENABLED(DUAL_X_CARRIAGE)
                if((0 == save_dual_x_carriage_mode) && (0 == READ(CHECKFILEMENT0_PIN)) && (active_extruder == 0))
                {
                  RTS_SndData(ExchangePageBase + 39, ExchangepageAddr);
                }
                else if((0 == save_dual_x_carriage_mode) && (0 == READ(CHECKFILEMENT1_PIN)) && (active_extruder == 1))
                {
                  RTS_SndData(ExchangePageBase + 39, ExchangepageAddr);
                }
                else if((0 != save_dual_x_carriage_mode) && ((0 == READ(CHECKFILEMENT0_PIN)) || (0 == READ(CHECKFILEMENT1_PIN))))
                {
                  rtscheck.RTS_SndData(ExchangePageBase + 39, ExchangepageAddr);
                }
              #else
                {
                  if(0 == READ(CHECKFILEMENT0_PIN))
                  {
                    RTS_SndData(ExchangePageBase + 39, ExchangepageAddr);
                  }                  
                }
              #endif
            #endif
          }

          RTS_SndData(ExchangePageBase + 40, ExchangepageAddr);
          #if ENABLED(TJC_AVAILABLE) 
            LCD_SERIAL_2.printf("page wait");
            LCD_SERIAL_2.printf("\xff\xff\xff");               
          #endif

          char pause_str_Z[16];
          char pause_str_E[16];

          memset(pause_str_Z, 0, sizeof(pause_str_Z));
          dtostrf(pause_z, 3, 2, pause_str_Z);

          memset(pause_str_E, 0, sizeof(pause_str_E));
          dtostrf(pause_e, 3, 2, pause_str_E);

          memset(commandbuf, 0, sizeof(commandbuf));
          sprintf_P(commandbuf, PSTR("G0 Z%s"), pause_str_Z);
          queue.enqueue_one_now(commandbuf);

          memset(commandbuf, 0, sizeof(commandbuf));
          sprintf_P(commandbuf, PSTR("G92.9 E%s"), pause_str_E);
          queue.enqueue_one_now(commandbuf);

          //card.startFileprint();
          //card.startOrResumeFilePrinting();
          //print_job_timer.start();

          if (ExtUI::isPrintingFromMediaPaused()) 
          {
            nozzle_park_mks.print_pause_start_flag = 0;
            nozzle_park_mks.blstatus = true;
            ExtUI::resumePrint();
          }

          Update_Time_Value = 0;
          sdcard_pause_check = true;

          RTS_SndData(ExchangePageBase + 11, ExchangepageAddr);
          #if ENABLED(TJC_AVAILABLE)
            LCD_SERIAL_2.printf("page printpause");
            LCD_SERIAL_2.printf("\xff\xff\xff");
          #endif 
        }
        else if(recdat.data[0] == 2)
        {
          if(enable_filment_check)
          {
            #if ENABLED(CHECKFILEMENT)
              #if ENABLED(DUAL_X_CARRIAGE)
                if((0 == save_dual_x_carriage_mode) && (0 == READ(CHECKFILEMENT0_PIN)) && (active_extruder == 0))
                {
                  RTS_SndData(ExchangePageBase + 39, ExchangepageAddr);
                }
                else if((0 == save_dual_x_carriage_mode) && (0 == READ(CHECKFILEMENT1_PIN)) && (active_extruder == 1))
                {
                  RTS_SndData(ExchangePageBase + 39, ExchangepageAddr);
                }
                else if((0 != save_dual_x_carriage_mode) && ((0 == READ(CHECKFILEMENT0_PIN)) || (0 == READ(CHECKFILEMENT1_PIN))))
                {
                  rtscheck.RTS_SndData(ExchangePageBase + 39, ExchangepageAddr);
                }
              #else
                if(0 == READ(CHECKFILEMENT0_PIN))
                {
                  RTS_SndData(ExchangePageBase + 39, ExchangepageAddr);
                }
              #endif
                else
                {
                  if(print_preheat_check == true)
                  {
                    queue.clear();
                    quickstop_stepper();
                    print_job_timer.stop();

                    RTS_SndData(ExchangePageBase + 10, ExchangepageAddr);
                    #if ENABLED(TJC_AVAILABLE)
                      LCD_SERIAL_2.printf("page printpause");
                      LCD_SERIAL_2.printf("\xff\xff\xff");
                    #endif

                    if((0 == save_dual_x_carriage_mode) && (thermalManager.temp_hotend[0].target <= 175))
                    {
                      queue.enqueue_now_P(PSTR("M109 S200"));
                    }
                    #if ENABLED(DUAL_X_CARRIAGE)
                    else if((0 != save_dual_x_carriage_mode) && ((thermalManager.temp_hotend[0].target < 175) && (thermalManager.temp_hotend[1].target < 175)))
                    {
                      queue.enqueue_now_P(PSTR("M109 T0 S200\nM109 T1 S200"));
                    }
                    #endif

                    char cmd[30];
                    char *c;
                    sprintf_P(cmd, PSTR("M23 %s"), CardRecbuf.Cardfilename[FilenamesCount]);
                    for (c = &cmd[4]; *c; c++)
                      *c = tolower(*c);

                    queue.enqueue_one_now(cmd);
                    queue.enqueue_now_P(PSTR("M24"));
                    PoweroffContinue = true;
                    sdcard_pause_check = true;
                    print_preheat_check = false;
                  }
                  else
                  {
                    RTS_SndData(ExchangePageBase + 40, ExchangepageAddr);

                    #if ENABLED(TJC_AVAILABLE) 
                      LCD_SERIAL_2.printf("page wait");
                      LCD_SERIAL_2.printf("\xff\xff\xff");               
                    #endif

                    //char pause_str_Z[16];
                    char pause_str_E[16];

                    // memset(pause_str_Z, 0, sizeof(pause_str_Z));
                    // dtostrf(pause_z, 3, 2, pause_str_Z);
                    // memset(commandbuf, 0, sizeof(commandbuf));
                    // sprintf_P(commandbuf, PSTR("G0 Z%s"), pause_str_Z);
                    // queue.enqueue_one_now(commandbuf);

                    memset(pause_str_E, 0, sizeof(pause_str_E));
                    dtostrf(pause_e, 3, 2, pause_str_E);
                    memset(commandbuf, 0, sizeof(commandbuf));
                    sprintf_P(commandbuf, PSTR("G92.9 E%s"), pause_str_E);
                    queue.enqueue_one_now(commandbuf);

                    //card.startFileprint();
                    //card.startOrResumeFilePrinting();

                    ExtUI::resumePrint();
                    
                    print_job_timer.start();
                    Update_Time_Value = 0;
                    sdcard_pause_check = true;
                    RTS_SndData(ExchangePageBase + 11, ExchangepageAddr);

                    #if ENABLED(TJC_AVAILABLE)
                      LCD_SERIAL_2.printf("page printpause");
                      LCD_SERIAL_2.printf("\xff\xff\xff");
                    #endif 
                  }
                }
            #endif
          }
        }
        else if(recdat.data[0] == 3)
        {
          if(PoweroffContinue == true)
          {
            if(enable_filment_check)
            {
              #if ENABLED(CHECKFILEMENT)
                #if ENABLED(DUAL_X_CARRIAGE)
                {
                  if((0 == save_dual_x_carriage_mode) && (0 == READ(CHECKFILEMENT0_PIN)) && (active_extruder == 0))
                  {
                    RTS_SndData(0, CHANGE_FILAMENT_ICON_VP);
                  }
                  else if((0 == save_dual_x_carriage_mode) && (1 == READ(CHECKFILEMENT0_PIN)) && (active_extruder == 0))
                  {
                    RTS_SndData(1, CHANGE_FILAMENT_ICON_VP);
                  }
                  else if((0 == save_dual_x_carriage_mode) && (0 == READ(CHECKFILEMENT1_PIN)) && (active_extruder == 1))
                  {
                    RTS_SndData(0, CHANGE_FILAMENT_ICON_VP);
                  }
                  else if((0 == save_dual_x_carriage_mode) && (1 == READ(CHECKFILEMENT1_PIN)) && (active_extruder == 1))
                  {
                    RTS_SndData(1, CHANGE_FILAMENT_ICON_VP);
                  }
                  else if((0 != save_dual_x_carriage_mode) && ((0 == READ(CHECKFILEMENT0_PIN)) || (0 == READ(CHECKFILEMENT1_PIN))))
                  {
                    RTS_SndData(0, CHANGE_FILAMENT_ICON_VP);
                  }
                  else if((0 != save_dual_x_carriage_mode) && (1 == READ(CHECKFILEMENT0_PIN)) && (1 == READ(CHECKFILEMENT1_PIN)))
                  {
                    RTS_SndData(1, CHANGE_FILAMENT_ICON_VP);
                  }
                }
                #else
                {
                  if(0 == READ(CHECKFILEMENT0_PIN))
                  {
                    RTS_SndData(0, CHANGE_FILAMENT_ICON_VP);
                  }
                  else if(1 == READ(CHECKFILEMENT0_PIN))
                  {
                    RTS_SndData(1, CHANGE_FILAMENT_ICON_VP);
                  }                  
                }
                #endif
              #endif
            }
            RTS_M600_Flag = false;
            RTS_SndData(ExchangePageBase + 8, ExchangepageAddr);
            #if ENABLED(TJC_AVAILABLE) 
              LCD_SERIAL_2.printf("page filamentresume");
              LCD_SERIAL_2.printf("\xff\xff\xff");               
            #endif
          }
          else if(PoweroffContinue == false)
          {
            if(enable_filment_check)
            {
              #if ENABLED(CHECKFILEMENT)
                #if ENABLED(DUAL_X_CARRIAGE)
                  if((0 == save_dual_x_carriage_mode) && (0 == READ(CHECKFILEMENT0_PIN)) && (active_extruder == 0))
                  {
                    RTS_SndData(ExchangePageBase + 39, ExchangepageAddr);
                    break;
                  }
                  else if((0 == save_dual_x_carriage_mode) && (0 == READ(CHECKFILEMENT1_PIN)) && (active_extruder == 1))
                  {
                    RTS_SndData(ExchangePageBase + 39, ExchangepageAddr);
                    break;
                  }
                  else if((0 != save_dual_x_carriage_mode) && ((0 == READ(CHECKFILEMENT0_PIN)) || (0 == READ(CHECKFILEMENT1_PIN))))
                  {
                    RTS_SndData(ExchangePageBase + 39, ExchangepageAddr);
                    break;
                  }
                #else
                  {
                    if(0 == READ(CHECKFILEMENT0_PIN))
                    {
                      RTS_SndData(ExchangePageBase + 39, ExchangepageAddr);
                      break;
                    }
                  }
                #endif
              #endif
            }

            char cmd[30];
            char *c;
            sprintf_P(cmd, PSTR("M23 %s"), CardRecbuf.Cardfilename[FilenamesCount]);
            for (c = &cmd[4]; *c; c++)
              *c = tolower(*c);

            #if ENABLED(TJC_AVAILABLE)
              LCD_SERIAL_2.printf("page printpause");
              LCD_SERIAL_2.printf("\xff\xff\xff");

              LCD_SERIAL_2.printf("restFlag2=0");
              LCD_SERIAL_2.printf("\xff\xff\xff");
              
              pause_count_pos = 0;
            #endif

            //图片预览
            #if ENABLED(TJC_AVAILABLE)

              LCD_SERIAL_2.printf("printpause.cp0.close()");
              LCD_SERIAL_2.printf("\xff\xff\xff");

              LCD_SERIAL_2.printf("printpause.cp0.aph=0");
              LCD_SERIAL_2.printf("\xff\xff\xff");          

              LCD_SERIAL_2.printf("printpause.va0.txt=\"\"");
              LCD_SERIAL_2.printf("\xff\xff\xff");

              LCD_SERIAL_2.printf("printpause.va1.txt=\"\"");
              LCD_SERIAL_2.printf("\xff\xff\xff");

              char picname[64];
              uint8_t public_buf[512];
              MediaFileReader file;
              sprintf(picname,"%s.txt",CardRecbuf.Cardshowfilename[CardRecbuf.recordcount]);
              bool run = file.open(picname);
              if(run)
              {
                while(1)
                {
                  TERN_(USE_WATCHDOG, hal.watchdog_refresh());

                  LCD_SERIAL_2.printf("printpause.cp0.aph=0");
                  LCD_SERIAL_2.printf("\xff\xff\xff");

                  LCD_SERIAL_2.printf("printpause.va0.txt=\"\"");
                  LCD_SERIAL_2.printf("\xff\xff\xff");

                  memset(public_buf,0,sizeof(public_buf));
                  int16_t byte = file.read(public_buf, 512);
                  
                  LCD_SERIAL_2.printf("printpause.va0.txt=");
                  LCD_SERIAL_2.write(0x22);
                  LCD_SERIAL_2.write(public_buf,sizeof(public_buf));
                  LCD_SERIAL_2.write(0x22);
                  LCD_SERIAL_2.printf("\xff\xff\xff");

                  LCD_SERIAL_2.printf("printpause.va1.txt+=printpause.va0.txt");
                  LCD_SERIAL_2.printf("\xff\xff\xff");
                        
                  if(byte<=0) 
                  {
                    LCD_SERIAL_2.printf("printpause.cp0.aph=127");
                    LCD_SERIAL_2.printf("\xff\xff\xff");

                    LCD_SERIAL_2.printf("printpause.cp0.write(printpause.va1.txt)");
                    LCD_SERIAL_2.printf("\xff\xff\xff");

                    file.close();
                    break;
                  };
                }
              }
              else
              {
                file.close();
                memset(picname,0,sizeof(picname));

                SdFile *diveDir;
                const char * const fname = card.diveToFile(true, diveDir, CardRecbuf.Cardfilename[CardRecbuf.recordcount]);
                bool run = file.open(fname);
                if(run)
                {
                  uint32_t gPicturePreviewStart = 0;
                  uint64_t cnt_pre = 0;

                  uint8_t public_buf[1024];

                  while(1)
                  {
                    LCD_SERIAL_2.printf("printpause.va0.txt=\"\"");
                    LCD_SERIAL_2.printf("\xff\xff\xff");

                    memset(public_buf,0,sizeof(public_buf));
                    int16_t  byte = file.read(public_buf,sizeof(public_buf));
                    if(((unsigned int)byte)<(sizeof(public_buf))) break;
                    cnt_pre = (cnt_pre + byte);
                    //uint32_t *p1  = (uint32_t *)strstr((char *)public_buf, ";gimage:");
                    uint32_t *p1  = (uint32_t *)strstr((char *)public_buf, ";simage:");
                    uint32_t *m1  = (uint32_t *)strstr((char *)public_buf, "M10086");

                    if(m1)
                    {
                      LCD_SERIAL_2.printf("printpause.cp0.aph=0");
                      LCD_SERIAL_2.printf("\xff\xff\xff");

                      LCD_SERIAL_2.printf("printpause.cp0.close()");
                      LCD_SERIAL_2.printf("\xff\xff\xff");
                      break;
                    }
                    
                    if(p1)
                    { 
                      while(1)
                      {
                        LCD_SERIAL_2.printf("printpause.va0.txt=\"\"");
                        LCD_SERIAL_2.printf("\xff\xff\xff");

                        memset(public_buf,0,sizeof(public_buf));
                        int16_t  byte = file.read(public_buf,sizeof(public_buf));
                        //uint32_t *p2  = (uint32_t *)strstr((char *)public_buf, ";;gimage:");
                        uint32_t *p2  = (uint32_t *)strstr((char *)public_buf, ";;simage:");
                        uint32_t *p3  = (uint32_t *)strstr((char *)public_buf, ";00000");
                        cnt_pre = (cnt_pre + byte);

                        //if(p2)
                        if(p2||p3)
                        {
                          file.rewind(); //文件读取指针复位

                          while(1) //";gimage:"起始位置
                          {
                            while(1)
                            {
                              LCD_SERIAL_2.printf("printpause.va0.txt=\"\"");
                              LCD_SERIAL_2.printf("\xff\xff\xff");

                              TERN_(USE_WATCHDOG, hal.watchdog_refresh());
                              memset(public_buf,0,sizeof(public_buf));
                              int16_t byte = file.read(public_buf,1024);
                              gPicturePreviewStart+=byte;

                              //uint32_t *p1 = (uint32_t *)strstr((char *)public_buf, ";gimage:");
                              //uint32_t *p2 = (uint32_t *)strstr((char *)public_buf, ";;gimage:");
                              uint32_t *p1 = (uint32_t *)strstr((char *)public_buf, ";simage:");
                              uint32_t *p2 = (uint32_t *)strstr((char *)public_buf, ";;simage:");
                              uint32_t *p3 = (uint32_t *)strstr((char *)public_buf, ";00000");
                              uint32_t *p4 = (uint32_t *)strstr((char *)public_buf, ";;gimage:");

                              //if(p2)

                              if( (p1==0)&& (p2==0) && (p4==0) && p3)
                              {
                                LCD_SERIAL_2.printf("printpause.cp0.aph=127");
                                LCD_SERIAL_2.printf("\xff\xff\xff");

                                LCD_SERIAL_2.printf("printpause.cp0.write(printpause.va1.txt)");
                                LCD_SERIAL_2.printf("\xff\xff\xff");

                                cnt_pre = 102400;
                                break;
                              }

                              if(p2)
                              {
                                LCD_SERIAL_2.printf("printpause.va0.txt=");
                                LCD_SERIAL_2.write(0x22);
                                LCD_SERIAL_2.write(&public_buf[9],1023-9);
                                LCD_SERIAL_2.write(0x22);
                                LCD_SERIAL_2.printf("\xff\xff\xff");
                                LCD_SERIAL_2.printf("printpause.va1.txt+=printpause.va0.txt");
                                LCD_SERIAL_2.printf("\xff\xff\xff");                                     

                                LCD_SERIAL_2.printf("printpause.cp0.aph=127");
                                LCD_SERIAL_2.printf("\xff\xff\xff");

                                TERN_(USE_WATCHDOG, hal.watchdog_refresh());
                                delay(200);
                                TERN_(USE_WATCHDOG, hal.watchdog_refresh());

                                LCD_SERIAL_2.printf("printpause.cp0.write(printpause.va1.txt)");
                                LCD_SERIAL_2.printf("\xff\xff\xff");
                                break;
                              }

                              if(p1)
                              {
                                LCD_SERIAL_2.printf("printpause.va0.txt=");
                                LCD_SERIAL_2.write(0x22);
                                LCD_SERIAL_2.write(&public_buf[8],1023-8);
                                LCD_SERIAL_2.write(0x22);
                                LCD_SERIAL_2.printf("\xff\xff\xff");
                                LCD_SERIAL_2.printf("printpause.va1.txt+=printpause.va0.txt");
                                LCD_SERIAL_2.printf("\xff\xff\xff");

                                TERN_(USE_WATCHDOG, hal.watchdog_refresh());
                                delay(200);
                                TERN_(USE_WATCHDOG, hal.watchdog_refresh());
                              }
                            }
                            break;
                          }
                        }
                        if(cnt_pre>=102400)  break;
                      }
                    }
                    if(cnt_pre>=102400)  break;
                  }
                  file.close();
                  delay(20);
                }

                file.close();
              }
            #endif

            queue.enqueue_one_now(cmd);
            delay(20);
            queue.enqueue_now_P(PSTR("M24"));

            // clean screen.
            for (int j = 0; j < 20; j ++)
            {
              RTS_SndData(0, PRINT_FILE_TEXT_VP + j);
            }
            RTS_SndData(CardRecbuf.Cardshowfilename[CardRecbuf.recordcount], PRINT_FILE_TEXT_VP);
            delay(2);
            #if ENABLED(BABYSTEPPING)
              RTS_SndData(0, AUTO_BED_LEVEL_ZOFFSET_VP);
              #if ENABLED(TJC_AVAILABLE) 
                memset(temp,0,sizeof(temp));
                sprintf(temp, "leveldata.z_offset.val=%d", 0);
                LCD_SERIAL_2.printf(temp);
                LCD_SERIAL_2.printf("\xff\xff\xff");           
              #endif
            #endif
            feedrate_percentage = 100;
            RTS_SndData(feedrate_percentage, PRINT_SPEED_RATE_VP);
            zprobe_zoffset = last_zoffset;
            RTS_SndData(zprobe_zoffset * 100, AUTO_BED_LEVEL_ZOFFSET_VP);
            #if ENABLED(TJC_AVAILABLE) 
              memset(temp,0,sizeof(temp));
              sprintf(temp, "leveldata.z_offset.val=%d", (int)(zprobe_zoffset * 100));
              LCD_SERIAL_2.printf(temp);
              LCD_SERIAL_2.printf("\xff\xff\xff");           
            #endif
            PoweroffContinue = true;

            RTS_SndData(ExchangePageBase + 10, ExchangepageAddr);
            // #if ENABLED(TJC_AVAILABLE)
            //   LCD_SERIAL_2.printf("page printpause");
            //   LCD_SERIAL_2.printf("\xff\xff\xff");
            // #endif

            sdcard_pause_check = true;
          }
        }
        else if(recdat.data[0] == 4)
        {
          if(!CardReader::flag.mounted)
          {
            CardUpdate = true;
            RTS_SDCardUpate();
            // card.mount();
            RTS_SndData(ExchangePageBase + 46, ExchangepageAddr);
          }
          else
          {
            RTS_SndData(ExchangePageBase + 40, ExchangepageAddr);
            #if ENABLED(TJC_AVAILABLE) 
             LCD_SERIAL_2.printf("page wait");
             LCD_SERIAL_2.printf("\xff\xff\xff");               
            #endif
            char pause_str_Z[16];
            char pause_str_E[16];
            memset(pause_str_Z, 0, sizeof(pause_str_Z));
            dtostrf(pause_z, 3, 2, pause_str_Z);
            memset(pause_str_E, 0, sizeof(pause_str_E));
            dtostrf(pause_e, 3, 2, pause_str_E);

            memset(commandbuf, 0, sizeof(commandbuf));
            sprintf_P(commandbuf, PSTR("G0 Z%s"), pause_str_Z);
            queue.enqueue_one_now(commandbuf);
            memset(commandbuf, 0, sizeof(commandbuf));
            sprintf_P(commandbuf, PSTR("G92.9 E%s"), pause_str_E);
            queue.enqueue_one_now(commandbuf);

            //card.startFileprint();
            card.startOrResumeFilePrinting();
            print_job_timer.start();
            Update_Time_Value = 0;
            sdcard_pause_check = true;
            sd_printing_autopause = false;
            RTS_SndData(ExchangePageBase + 11, ExchangepageAddr);
            #if ENABLED(TJC_AVAILABLE)
              LCD_SERIAL_2.printf("page printpause");
              LCD_SERIAL_2.printf("\xff\xff\xff");
            #endif             
          }
        }
      }
      break;

      case ZOffsetKey:
      {
        #if ENABLED(BABYSTEPPING)
          last_zoffset = zprobe_zoffset;
          if(recdat.data[0] >= 32768)
          {
            zprobe_zoffset = ((float)recdat.data[0] - 65536) / 100;
          }
          else
          {
            zprobe_zoffset = ((float)recdat.data[0]) / 100;
          }
          if(WITHIN((zprobe_zoffset), Z_PROBE_OFFSET_RANGE_MIN, Z_PROBE_OFFSET_RANGE_MAX))
          {
            babystep.add_mm(Z_AXIS, zprobe_zoffset - last_zoffset);
          }

          #if HAS_BED_PROBE
            probe.offset.z = zprobe_zoffset;
          #endif
        #endif
      }
      break;

      case TempScreenKey: //0x1030
      {
        if (recdat.data[0] == 1)
        {
          temp_ctrl = 1;
          RTS_SndData(0, ICON_ADJUST_PRINTING_EXTRUDER_OR_BED);
          RTS_SndData(thermalManager.temp_hotend[0].target, SPEED_SET_VP);
          RTS_SndData(thermalManager.temp_hotend[0].target, HEAD0_SET_TEMP_VP);

          #if ENABLED(TJC_AVAILABLE) 
            memset(temp,0,sizeof(temp));
            sprintf(temp, "adjusttemp.targettemp.val=%d", (int)(thermalManager.temp_hotend[0].target));
            LCD_SERIAL_2.printf(temp);
            LCD_SERIAL_2.printf("\xff\xff\xff");           
          #endif     
        }
        else if (recdat.data[0] == 2)
        {
        
        }
        else if (recdat.data[0] == 3)
        {          
          temp_ctrl = 0; 
          RTS_SndData(1, ICON_ADJUST_PRINTING_EXTRUDER_OR_BED);
          RTS_SndData(thermalManager.temp_bed.target, SPEED_SET_VP);

          #if ENABLED(TJC_AVAILABLE) 
            memset(temp,0,sizeof(temp));
            sprintf(temp, "adjusttemp.targettemp.val=%d", (int)(thermalManager.temp_bed.target));
            LCD_SERIAL_2.printf(temp);
            LCD_SERIAL_2.printf("\xff\xff\xff");           
          #endif 
        }
        else if (recdat.data[0] == 4)
        {

        }
        else if (recdat.data[0] == 5) //1℃
        {
          unit = 1;
          RTS_SndData(0, ICON_ADJUST_PRINTING_TEMP_UNIT);
          RTS_SndData(0, ICON_ADJUST_PRINTING_S_F_UNIT);

          AxisUnitMode = 1;
          axis_unit    = 0.1;
          RTS_SndData(0, ICON_MOVE_DISTANCE_SELECT);
        }
        else if (recdat.data[0] == 6) //5℃
        {
          unit = 5;
          RTS_SndData(1, ICON_ADJUST_PRINTING_TEMP_UNIT);
          RTS_SndData(1, ICON_ADJUST_PRINTING_S_F_UNIT);

          AxisUnitMode = 2;
          axis_unit    = 1.0;
          RTS_SndData(1, ICON_MOVE_DISTANCE_SELECT);
        }
        else if (recdat.data[0] == 7) //10℃
        {
          unit = 10;
          RTS_SndData(2, ICON_ADJUST_PRINTING_TEMP_UNIT);
          RTS_SndData(2, ICON_ADJUST_PRINTING_S_F_UNIT);

          AxisUnitMode = 3;
          axis_unit    = 10.0;
          RTS_SndData(2, ICON_MOVE_DISTANCE_SELECT);
        } 
        else if (recdat.data[0] == 8) //++温度
        {
          if(temp_ctrl)
          {
            if((thermalManager.temp_hotend[0].target + unit)>260)
            {
              thermalManager.temp_hotend[0].target = 260;
            }
            else
            {
              thermalManager.temp_hotend[0].target = (thermalManager.temp_hotend[0].target + unit);
            }
            thermalManager.setTargetHotend(thermalManager.temp_hotend[0].target, 0);
            RTS_SndData(thermalManager.temp_hotend[0].target, HEAD0_SET_TEMP_VP);
            RTS_SndData(thermalManager.temp_hotend[0].target, SPEED_SET_VP);

            #if ENABLED(TJC_AVAILABLE) 
              memset(temp,0,sizeof(temp));
              sprintf(temp, "adjusttemp.targettemp.val=%d", (int)(thermalManager.temp_hotend[0].target));
              LCD_SERIAL_2.printf(temp);
              LCD_SERIAL_2.printf("\xff\xff\xff");           
            #endif            

          }
          else
          {
            if((thermalManager.temp_bed.target + unit)>110)
            {
              thermalManager.temp_bed.target = 110;
            }
            else
            {
              thermalManager.temp_bed.target = (thermalManager.temp_bed.target + unit);
              thermalManager.setTargetBed(thermalManager.temp_bed.target);
              RTS_SndData(thermalManager.temp_bed.target, BED_SET_TEMP_VP);
              RTS_SndData(thermalManager.temp_bed.target, SPEED_SET_VP);
            }

            #if ENABLED(TJC_AVAILABLE) 
              memset(temp,0,sizeof(temp));
              sprintf(temp, "adjusttemp.targettemp.val=%d", (int)(thermalManager.temp_bed.target));
              LCD_SERIAL_2.printf(temp);
              LCD_SERIAL_2.printf("\xff\xff\xff");           
            #endif 
          }
        }
        else if (recdat.data[0] == 9)  //--温度
        {
          if(temp_ctrl)
          {
            if((thermalManager.temp_hotend[0].target - unit)>=25)
            {
              thermalManager.temp_hotend[0].target = (thermalManager.temp_hotend[0].target - unit);
              thermalManager.setTargetHotend(thermalManager.temp_hotend[0].target, 0);
              RTS_SndData(thermalManager.temp_hotend[0].target, HEAD0_SET_TEMP_VP);
              RTS_SndData(thermalManager.temp_hotend[0].target, SPEED_SET_VP);
            }

            #if ENABLED(TJC_AVAILABLE) 
              memset(temp,0,sizeof(temp));
              sprintf(temp, "adjusttemp.targettemp.val=%d", (int)(thermalManager.temp_hotend[0].target));
              LCD_SERIAL_2.printf(temp);
              LCD_SERIAL_2.printf("\xff\xff\xff");           
            #endif   
          }
          else
          {
            if((thermalManager.temp_bed.target - unit)>=25)
            {
              thermalManager.temp_bed.target = (thermalManager.temp_bed.target - unit);
              thermalManager.setTargetBed(thermalManager.temp_bed.target);
              RTS_SndData(thermalManager.temp_bed.target, BED_SET_TEMP_VP);
              RTS_SndData(thermalManager.temp_bed.target, SPEED_SET_VP);
            }

            #if ENABLED(TJC_AVAILABLE) 
              memset(temp,0,sizeof(temp));
              sprintf(temp, "adjusttemp.targettemp.val=%d", (int)(thermalManager.temp_bed.target));
              LCD_SERIAL_2.printf(temp);
              LCD_SERIAL_2.printf("\xff\xff\xff");           
            #endif 
          }
        }
        else if(recdat.data[0] == 0x0A)
        {
          RTS_SndData(0, ICON_ADJUST_PRINTING_SPEED_FLOW);
          RTS_SndData(feedrate_percentage, SPEED_SET_VP);
          speed_ctrl = 1;

          #if ENABLED(TJC_AVAILABLE) 
            memset(temp,0,sizeof(temp));
            sprintf(temp, "adjustspeed.targetspeed.val=%d", (int)(feedrate_percentage));
            LCD_SERIAL_2.printf(temp);
            LCD_SERIAL_2.printf("\xff\xff\xff");           
          #endif 
        } 
        else if(recdat.data[0] == 0x0B)
        {
          RTS_SndData(1, ICON_ADJUST_PRINTING_SPEED_FLOW);
          RTS_SndData(planner.flow_percentage[0], SPEED_SET_VP);
          speed_ctrl = 2;

          #if ENABLED(TJC_AVAILABLE) 
            memset(temp,0,sizeof(temp));
            sprintf(temp, "adjustspeed.targetspeed.val=%d", (int)(planner.flow_percentage[0]));
            LCD_SERIAL_2.printf(temp);
            LCD_SERIAL_2.printf("\xff\xff\xff");           
          #endif 
        }
        else if(recdat.data[0] == 0x0C)
        {
          RTS_SndData(2, ICON_ADJUST_PRINTING_SPEED_FLOW);
          RTS_SndData(thermalManager.fan_speed[0], SPEED_SET_VP);
          speed_ctrl = 3;
          #if ENABLED(TJC_AVAILABLE) 
            memset(temp,0,sizeof(temp));
            sprintf(temp, "adjustspeed.targetspeed.val=%d", (int)(thermalManager.fan_speed[0]));
            LCD_SERIAL_2.printf(temp);
            LCD_SERIAL_2.printf("\xff\xff\xff");           
          #endif 
        }
        else if(recdat.data[0] == 0x0D)
        {
          if(speed_ctrl==1)
          {
            if((feedrate_percentage + unit)>300)
            {
              feedrate_percentage = 300;
            }
            else
            {
              feedrate_percentage = (feedrate_percentage + unit);
            }
            RTS_SndData(feedrate_percentage, SPEED_SET_VP);

            #if ENABLED(TJC_AVAILABLE) 
              memset(temp,0,sizeof(temp));
              sprintf(temp, "adjustspeed.targetspeed.val=%d", (int)(feedrate_percentage));
              LCD_SERIAL_2.printf(temp);
              LCD_SERIAL_2.printf("\xff\xff\xff");           
            #endif 
          }
          else if(speed_ctrl==2)
          {
            if((planner.flow_percentage[0] + unit)>300)
            {
              planner.flow_percentage[0] = 300;
            }
            else 
            {
              planner.flow_percentage[0] = (planner.flow_percentage[0] + unit);
            }
            planner.refresh_e_factor(0);
            RTS_SndData(planner.flow_percentage[0], SPEED_SET_VP);

            #if ENABLED(TJC_AVAILABLE) 
              memset(temp,0,sizeof(temp));
              sprintf(temp, "adjustspeed.targetspeed.val=%d", (int)(planner.flow_percentage[0]));
              LCD_SERIAL_2.printf(temp);
              LCD_SERIAL_2.printf("\xff\xff\xff");           
            #endif 
          }
          else if(speed_ctrl==3)
          {
            if((thermalManager.fan_speed[0] + unit)>255)
            {
              thermalManager.fan_speed[0] = 255;
            }
            else
            {
              thermalManager.fan_speed[0] = (thermalManager.fan_speed[0] + unit);
            }
            thermalManager.set_fan_speed(0, thermalManager.fan_speed[0]);
            RTS_SndData(thermalManager.fan_speed[0], SPEED_SET_VP);

            #if ENABLED(TJC_AVAILABLE) 
              memset(temp,0,sizeof(temp));
              sprintf(temp, "adjustspeed.targetspeed.val=%d", (int)(thermalManager.fan_speed[0]));
              LCD_SERIAL_2.printf(temp);
              LCD_SERIAL_2.printf("\xff\xff\xff");           
            #endif
          }
        }
        else if(recdat.data[0] == 0x0E)
        {
          if(speed_ctrl==1)
          {
            if((feedrate_percentage - unit)<10)
            {
              feedrate_percentage = 10;
            }
            else
            {
              feedrate_percentage = (feedrate_percentage - unit);
            }
            RTS_SndData(feedrate_percentage, SPEED_SET_VP);
            #if ENABLED(TJC_AVAILABLE) 
              memset(temp,0,sizeof(temp));
              sprintf(temp, "adjustspeed.targetspeed.val=%d", (int)(feedrate_percentage));
              LCD_SERIAL_2.printf(temp);
              LCD_SERIAL_2.printf("\xff\xff\xff");           
            #endif 
          }
          else if(speed_ctrl==2)
          {
            if((planner.flow_percentage[0] - unit)<100)
            {
              planner.flow_percentage[0] = 100;
            }
            else 
            {
              planner.flow_percentage[0] = (planner.flow_percentage[0] - unit);
            }
            planner.refresh_e_factor(0);
            RTS_SndData(planner.flow_percentage[0], SPEED_SET_VP);
            #if ENABLED(TJC_AVAILABLE) 
              memset(temp,0,sizeof(temp));
              sprintf(temp, "adjustspeed.targetspeed.val=%d", (int)(planner.flow_percentage[0]));
              LCD_SERIAL_2.printf(temp);
              LCD_SERIAL_2.printf("\xff\xff\xff");           
            #endif             
          }
          else if(speed_ctrl==3)
          {
            if((thermalManager.fan_speed[0] - unit)<0)
            {
              thermalManager.fan_speed[0] = 0;
            }
            else
            {
              thermalManager.fan_speed[0] = (thermalManager.fan_speed[0] - unit);
            }
            thermalManager.set_fan_speed(0, thermalManager.fan_speed[0]);
            RTS_SndData(thermalManager.fan_speed[0], SPEED_SET_VP);
            #if ENABLED(TJC_AVAILABLE) 
              memset(temp,0,sizeof(temp));
              sprintf(temp, "adjustspeed.targetspeed.val=%d", (int)(thermalManager.fan_speed[0]));
              LCD_SERIAL_2.printf(temp);
              LCD_SERIAL_2.printf("\xff\xff\xff");           
            #endif
          }
        }
        else if(recdat.data[0] == 0x0F) //最大加速度
        {
          advaned_set = 1;
          unit = 10;
        }
        else if(recdat.data[0] == 0x10) //最大速度
        {
          advaned_set = 2;
          unit = 10;    
        }
        else if(recdat.data[0] == 0x11)
        {
          if(advaned_set==1)
          {
            planner.settings.max_feedrate_mm_s[X_AXIS] = (planner.settings.max_feedrate_mm_s[X_AXIS] - unit);
            if(planner.settings.max_feedrate_mm_s[X_AXIS]<100)
            {
              planner.settings.max_feedrate_mm_s[X_AXIS] = 100;
            }
            planner.set_max_feedrate(X_AXIS, planner.settings.max_feedrate_mm_s[X_AXIS]);
          }
          else if(advaned_set==2)
          {
            planner.settings.max_acceleration_mm_per_s2[X_AXIS] = (planner.settings.max_acceleration_mm_per_s2[X_AXIS] - (unit*10));
            if(planner.settings.max_acceleration_mm_per_s2[X_AXIS]<100)
            {
              planner.settings.max_acceleration_mm_per_s2[X_AXIS] = 100;
            }
            planner.set_max_acceleration(X_AXIS, planner.settings.max_acceleration_mm_per_s2[X_AXIS]);            
          }
        }
        else if(recdat.data[0] == 0x12)
        {
          if(advaned_set==1)
          {
            planner.settings.max_feedrate_mm_s[Y_AXIS] = (planner.settings.max_feedrate_mm_s[Y_AXIS] - unit);
            if(planner.settings.max_feedrate_mm_s[Y_AXIS]<100)
            {
              planner.settings.max_feedrate_mm_s[Y_AXIS] = 100;
            }
            planner.set_max_feedrate(Y_AXIS, planner.settings.max_feedrate_mm_s[Y_AXIS]);
          }
          else if(advaned_set==2)
          {
            planner.settings.max_acceleration_mm_per_s2[Y_AXIS] = (planner.settings.max_acceleration_mm_per_s2[Y_AXIS] - (unit*10));
            if(planner.settings.max_acceleration_mm_per_s2[Y_AXIS]<100)
            {
              planner.settings.max_acceleration_mm_per_s2[Y_AXIS] = 100;
            }             
          }
          planner.set_max_acceleration(Y_AXIS, planner.settings.max_acceleration_mm_per_s2[Y_AXIS]);
        }
        else if(recdat.data[0] == 0x13)
        {
          if(advaned_set==1)
          {
            planner.settings.max_feedrate_mm_s[Z_AXIS] = (planner.settings.max_feedrate_mm_s[Z_AXIS] - unit);
            if(planner.settings.max_feedrate_mm_s[Z_AXIS]<5)
            {
              planner.settings.max_feedrate_mm_s[Z_AXIS] = 5;
            }
            planner.set_max_feedrate(Z_AXIS, planner.settings.max_feedrate_mm_s[Z_AXIS]);
          }
          else if(advaned_set==2)
          {
            if(planner.settings.max_acceleration_mm_per_s2[Z_AXIS]==50)
            {
              planner.settings.max_acceleration_mm_per_s2[Z_AXIS] = 50;
            }
            else
            {
              planner.settings.max_acceleration_mm_per_s2[Z_AXIS] = (planner.settings.max_acceleration_mm_per_s2[Z_AXIS] - (unit*10));
              if(planner.settings.max_acceleration_mm_per_s2[Z_AXIS]<50)
              {
                planner.settings.max_acceleration_mm_per_s2[Z_AXIS] = 50;
              }
            }
            planner.set_max_acceleration(Z_AXIS, planner.settings.max_acceleration_mm_per_s2[Z_AXIS]);            
          }
        }
        else if(recdat.data[0] == 0x14)
        {
          if(advaned_set==1)
          {
            planner.settings.max_feedrate_mm_s[E_AXIS_N(0)] = (planner.settings.max_feedrate_mm_s[E_AXIS_N(0)] - unit);
            if(planner.settings.max_feedrate_mm_s[E_AXIS_N(0)]<10)
            {
              planner.settings.max_feedrate_mm_s[E_AXIS_N(0)] = 10;
            }
            planner.set_max_feedrate(E_AXIS_N(0), planner.settings.max_feedrate_mm_s[E_AXIS_N(0)]);
          }
          else if(advaned_set==2)
          {
            planner.settings.max_acceleration_mm_per_s2[E_AXIS_N(0)] = (planner.settings.max_acceleration_mm_per_s2[E_AXIS_N(0)] - (unit*10));
            if(planner.settings.max_acceleration_mm_per_s2[E_AXIS_N(0)]<100)
            {
              planner.settings.max_acceleration_mm_per_s2[E_AXIS_N(0)] = 100;
            }
            planner.set_max_acceleration(E_AXIS_N(0), planner.settings.max_acceleration_mm_per_s2[E_AXIS_N(0)]);              
          }
        }
        else if(recdat.data[0] == 0x15) //X轴
        {
          if(advaned_set==1) //最大速度
          {
            planner.settings.max_feedrate_mm_s[X_AXIS] = (planner.settings.max_feedrate_mm_s[X_AXIS] + unit);
            if(planner.settings.max_feedrate_mm_s[X_AXIS]>300)
            {
              planner.settings.max_feedrate_mm_s[X_AXIS] = 300;
            }
            planner.set_max_feedrate(X_AXIS, planner.settings.max_feedrate_mm_s[X_AXIS]);
          }
          else if(advaned_set==2) //最大加速度
          {
            planner.settings.max_acceleration_mm_per_s2[X_AXIS] = (planner.settings.max_acceleration_mm_per_s2[X_AXIS] + (unit*10));
            if(planner.settings.max_acceleration_mm_per_s2[X_AXIS]>3000)
            {
              planner.settings.max_acceleration_mm_per_s2[X_AXIS] = 3000;
            }
            planner.set_max_acceleration(X_AXIS, planner.settings.max_acceleration_mm_per_s2[X_AXIS]);         
          }
        }
        else if(recdat.data[0] == 0x16) //Y轴
        {
          if(advaned_set==1)  //最大速度
          {
            planner.settings.max_feedrate_mm_s[Y_AXIS] = (planner.settings.max_feedrate_mm_s[Y_AXIS] + unit);
            if(planner.settings.max_feedrate_mm_s[Y_AXIS]>300)
            {
              planner.settings.max_feedrate_mm_s[Y_AXIS] = 300;
            }
            planner.set_max_feedrate(Y_AXIS, planner.settings.max_feedrate_mm_s[Y_AXIS]);
          }
          else if(advaned_set==2) //最大加速度
          {
            planner.settings.max_acceleration_mm_per_s2[Y_AXIS] = (planner.settings.max_acceleration_mm_per_s2[Y_AXIS] + (unit*10));
            if(planner.settings.max_acceleration_mm_per_s2[Y_AXIS]>3000)
            {
              planner.settings.max_acceleration_mm_per_s2[Y_AXIS] = 3000;
            }
            planner.set_max_acceleration(Y_AXIS, planner.settings.max_acceleration_mm_per_s2[Y_AXIS]);            
          }
        }
        else if(recdat.data[0] == 0x17) //Z轴
        {
          if(advaned_set==1) //最大速度
          {
            planner.settings.max_feedrate_mm_s[Z_AXIS] = (planner.settings.max_feedrate_mm_s[Z_AXIS] + unit);
            if(planner.settings.max_feedrate_mm_s[Z_AXIS]>15)
            {
              planner.settings.max_feedrate_mm_s[Z_AXIS] = 15;
            }
            planner.set_max_feedrate(Z_AXIS, planner.settings.max_feedrate_mm_s[Z_AXIS]);
          }
          else if(advaned_set==2) //最大加速度
          {
            planner.settings.max_acceleration_mm_per_s2[Z_AXIS] = (planner.settings.max_acceleration_mm_per_s2[Z_AXIS] + (unit*10));
            if(planner.settings.max_acceleration_mm_per_s2[Z_AXIS]>150)
            {
              planner.settings.max_acceleration_mm_per_s2[Z_AXIS] = 150;
            }
            planner.set_max_acceleration(Z_AXIS, planner.settings.max_acceleration_mm_per_s2[Z_AXIS]);             
          }
        }
        else if(recdat.data[0] == 0x18) //E轴
        {
          if(advaned_set==1) //最大速度
          {
            planner.settings.max_feedrate_mm_s[E_AXIS_N(0)] = (planner.settings.max_feedrate_mm_s[E_AXIS_N(0)] + unit);
            if(planner.settings.max_feedrate_mm_s[E_AXIS_N(0)]>25)
            {
              planner.settings.max_feedrate_mm_s[E_AXIS_N(0)] = 25;
            }
            planner.set_max_feedrate(E_AXIS_N(0), planner.settings.max_feedrate_mm_s[E_AXIS_N(0)]);
          }
          else if(advaned_set==2) //最大加速度
          {
            planner.settings.max_acceleration_mm_per_s2[E_AXIS_N(0)] = (planner.settings.max_acceleration_mm_per_s2[E_AXIS_N(0)] + (unit*10));
            if(planner.settings.max_acceleration_mm_per_s2[E_AXIS_N(0)]>2000)
            {
              planner.settings.max_acceleration_mm_per_s2[E_AXIS_N(0)] = 2000;
            }
            planner.set_max_acceleration(E_AXIS_N(0), planner.settings.max_acceleration_mm_per_s2[E_AXIS_N(0)]);              
          }
        }                        
        else if (recdat.data[0] == 0xF1)
        {
          #if FAN_COUNT > 0
            for (uint8_t i = 0; i < FAN_COUNT; i++)
            {
              thermalManager.fan_speed[i] = 255;
            }
          #endif

          thermalManager.setTargetHotend(0, 0);
          RTS_SndData(0, HEAD0_SET_TEMP_VP);
          delay(1);
          thermalManager.setTargetHotend(0, 1);
          RTS_SndData(0, HEAD1_SET_TEMP_VP);
          delay(1);
          thermalManager.setTargetBed(0);
          RTS_SndData(0, BED_SET_TEMP_VP);
          delay(1);

          RTS_SndData(ExchangePageBase + 15, ExchangepageAddr);
        }
        else if (recdat.data[0] == 0xF0)
        {
          RTS_SndData(ExchangePageBase + 15, ExchangepageAddr);
        }

        #if ENABLED(TJC_AVAILABLE) 
          if(advaned_set==1)
          {
            memset(temp,0,sizeof(temp));
            sprintf(temp, "speedsetvalue.xaxis.val=%d", (int)(planner.settings.max_feedrate_mm_s[X_AXIS]));
            LCD_SERIAL_2.printf(temp);
            LCD_SERIAL_2.printf("\xff\xff\xff");

            memset(temp,0,sizeof(temp));
            sprintf(temp, "speedsetvalue.yaxis.val=%d", (int)(planner.settings.max_feedrate_mm_s[Y_AXIS]));
            LCD_SERIAL_2.printf(temp);
            LCD_SERIAL_2.printf("\xff\xff\xff");

            memset(temp,0,sizeof(temp));
            sprintf(temp, "speedsetvalue.zaxis.val=%d", (int)(planner.settings.max_feedrate_mm_s[Z_AXIS]));
            LCD_SERIAL_2.printf(temp);
            LCD_SERIAL_2.printf("\xff\xff\xff");

            memset(temp,0,sizeof(temp));
            sprintf(temp, "speedsetvalue.eaxis.val=%d", (int)(planner.settings.max_feedrate_mm_s[E_AXIS_N(0)]));
            LCD_SERIAL_2.printf(temp);
            LCD_SERIAL_2.printf("\xff\xff\xff");
          }
          else if(advaned_set==2)
          {
            memset(temp,0,sizeof(temp));
            sprintf(temp, "speedsetvalue.xaxis.val=%d", (int)(planner.settings.max_acceleration_mm_per_s2[X_AXIS]));
            LCD_SERIAL_2.printf(temp);
            LCD_SERIAL_2.printf("\xff\xff\xff");

            memset(temp,0,sizeof(temp));
            sprintf(temp, "speedsetvalue.yaxis.val=%d", (int)(planner.settings.max_acceleration_mm_per_s2[Y_AXIS]));
            LCD_SERIAL_2.printf(temp);
            LCD_SERIAL_2.printf("\xff\xff\xff");

            memset(temp,0,sizeof(temp));
            sprintf(temp, "speedsetvalue.zaxis.val=%d", (int)(planner.settings.max_acceleration_mm_per_s2[Z_AXIS]));
            LCD_SERIAL_2.printf(temp);
            LCD_SERIAL_2.printf("\xff\xff\xff");

            memset(temp,0,sizeof(temp));
            sprintf(temp, "speedsetvalue.eaxis.val=%d", (int)(planner.settings.max_acceleration_mm_per_s2[E_AXIS_N(0)]));
            LCD_SERIAL_2.printf(temp);
            LCD_SERIAL_2.printf("\xff\xff\xff");    
          }                                    
        #endif 
      }
      break;

      case CoolScreenKey:
      {
        uint8_t e_temp = 0,bed_temp = 0;
        
        if (recdat.data[0] == 1)
        {
          thermalManager.setTargetHotend(0, 0);
          //thermalManager.fan_speed[0] = 255;
          RTS_SndData(0, HEAD0_SET_TEMP_VP);
          RTS_SndData(0, HEAD0_FAN_ICON_VP);

          #if ENABLED(TJC_AVAILABLE) 
            //pretemp
            memset(temp,0,sizeof(temp));
            sprintf(temp, "pretemp.nozzletemp.txt=\"%d / %d\"", thermalManager.wholeDegHotend(0) , thermalManager.degTargetHotend(0));
            LCD_SERIAL_2.printf(temp);
            LCD_SERIAL_2.printf("\xff\xff\xff");

            memset(temp,0,sizeof(temp));
            sprintf(temp, "pretemp.bedtemp.txt=\"%d / %d\"", thermalManager.wholeDegBed() , thermalManager.degTargetBed());
            LCD_SERIAL_2.printf(temp);
            LCD_SERIAL_2.printf("\xff\xff\xff");
          #endif
        }
        else if (recdat.data[0] == 2)
        {
          thermalManager.setTargetBed(0);
          RTS_SndData(0, BED_SET_TEMP_VP);

          #if ENABLED(TJC_AVAILABLE) 
            //pretemp
            memset(temp,0,sizeof(temp));
            sprintf(temp, "pretemp.nozzletemp.txt=\"%d / %d\"", thermalManager.wholeDegHotend(0) , thermalManager.degTargetHotend(0));
            LCD_SERIAL_2.printf(temp);
            LCD_SERIAL_2.printf("\xff\xff\xff");

            memset(temp,0,sizeof(temp));
            sprintf(temp, "pretemp.bedtemp.txt=\"%d / %d\"", thermalManager.wholeDegBed() , thermalManager.degTargetBed());
            LCD_SERIAL_2.printf(temp);
            LCD_SERIAL_2.printf("\xff\xff\xff");
          #endif
        }
        else if (recdat.data[0] == 3)
        {
          thermalManager.setTargetHotend(0, 1);
          RTS_SndData(0, HEAD1_SET_TEMP_VP);
          #if HAS_FAN1
            thermalManager.fan_speed[1] = 255;
          #endif
          RTS_SndData(0, HEAD1_FAN_ICON_VP);
        }
        else if (recdat.data[0] == 4)
        {
          RTS_SndData(ExchangePageBase + 15, ExchangepageAddr);
        }
        else if (recdat.data[0] == 5)
        {
          #if HAS_HOTEND
            thermalManager.temp_hotend[0].target = PREHEAT_1_TEMP_HOTEND;
            thermalManager.setTargetHotend(thermalManager.temp_hotend[0].target, 0);
            RTS_SndData(thermalManager.temp_hotend[0].target, HEAD0_SET_TEMP_VP);
            thermalManager.temp_bed.target = PREHEAT_1_TEMP_BED;
            thermalManager.setTargetBed(thermalManager.temp_bed.target);
            RTS_SndData(thermalManager.temp_bed.target, BED_SET_TEMP_VP);
          #endif
        }
        else if (recdat.data[0] == 6)
        {
          #if HAS_HOTEND
            thermalManager.temp_hotend[0].target = PREHEAT_2_TEMP_HOTEND;
            thermalManager.setTargetHotend(thermalManager.temp_hotend[0].target, 0);
            RTS_SndData(thermalManager.temp_hotend[0].target, HEAD0_SET_TEMP_VP);
            thermalManager.temp_bed.target = PREHEAT_2_TEMP_BED;
            thermalManager.setTargetBed(thermalManager.temp_bed.target);
            RTS_SndData(thermalManager.temp_bed.target, BED_SET_TEMP_VP);
          #endif
        }
        else if (recdat.data[0] == 7)
        {
          #if HAS_MULTI_HOTEND
            thermalManager.temp_hotend[1].target = PREHEAT_1_TEMP_HOTEND;
            thermalManager.setTargetHotend(thermalManager.temp_hotend[1].target, 1);
            RTS_SndData(thermalManager.temp_hotend[1].target, HEAD1_SET_TEMP_VP);
            thermalManager.temp_bed.target = PREHEAT_1_TEMP_BED;
            thermalManager.setTargetBed(thermalManager.temp_bed.target);
            RTS_SndData(thermalManager.temp_bed.target, BED_SET_TEMP_VP);
          #endif
        }
        else if (recdat.data[0] == 8)
        {
          #if HAS_MULTI_HOTEND
            thermalManager.temp_hotend[1].target = PREHEAT_2_TEMP_HOTEND;
            thermalManager.setTargetHotend(thermalManager.temp_hotend[1].target, 1);
            RTS_SndData(thermalManager.temp_hotend[1].target, HEAD1_SET_TEMP_VP);
            thermalManager.temp_bed.target = PREHEAT_2_TEMP_BED;
            thermalManager.setTargetBed(thermalManager.temp_bed.target);
            RTS_SndData(thermalManager.temp_bed.target, BED_SET_TEMP_VP);
          #endif
        }
        else if(recdat.data[0] == 9) //预热PLA
        {
          e_temp   = pla_extrusion_temp;
          bed_temp = pla_bed_temp;
          RTS_SndData(e_temp, HEAD0_SET_TEMP_VP);
          RTS_SndData(bed_temp, BED_SET_TEMP_VP);
          thermalManager.setTargetHotend(e_temp, ExtUI::extruder_t::E0);
          thermalManager.setTargetBed(bed_temp);

          #if ENABLED(TJC_AVAILABLE) 
            //pretemp
            memset(temp,0,sizeof(temp));
            sprintf(temp, "pretemp.nozzletemp.txt=\"%d / %d\"", thermalManager.wholeDegHotend(0) , thermalManager.degTargetHotend(0));
            LCD_SERIAL_2.printf(temp);
            LCD_SERIAL_2.printf("\xff\xff\xff");

            memset(temp,0,sizeof(temp));
            sprintf(temp, "pretemp.bedtemp.txt=\"%d / %d\"", thermalManager.wholeDegBed() , thermalManager.degTargetBed());
            LCD_SERIAL_2.printf(temp);
            LCD_SERIAL_2.printf("\xff\xff\xff");

            memset(temp,0,sizeof(temp));
            sprintf(temp, "pretemp.nozzle.txt=\"%d\"",thermalManager.degTargetHotend(0));
            LCD_SERIAL_2.printf(temp);
            LCD_SERIAL_2.printf("\xff\xff\xff");

            memset(temp,0,sizeof(temp));
            sprintf(temp, "pretemp.bed.txt=\"%d\"", thermalManager.degTargetBed());
            LCD_SERIAL_2.printf(temp);
            LCD_SERIAL_2.printf("\xff\xff\xff");

          #endif
        }
        else if(recdat.data[0] == 10) //预热ABS
        {
          e_temp   = petg_extrusion_temp;
          bed_temp = petg_bed_temp;
          RTS_SndData(e_temp, HEAD0_SET_TEMP_VP);
          RTS_SndData(bed_temp, BED_SET_TEMP_VP);
          thermalManager.setTargetHotend(e_temp, ExtUI::extruder_t::E0);
          thermalManager.setTargetBed(bed_temp);

          #if ENABLED(TJC_AVAILABLE) 
            //pretemp
            memset(temp,0,sizeof(temp));
            sprintf(temp, "pretemp.nozzletemp.txt=\"%d / %d\"", thermalManager.wholeDegHotend(0) , thermalManager.degTargetHotend(0));
            LCD_SERIAL_2.printf(temp);
            LCD_SERIAL_2.printf("\xff\xff\xff");

            memset(temp,0,sizeof(temp));
            sprintf(temp, "pretemp.bedtemp.txt=\"%d / %d\"", thermalManager.wholeDegBed() , thermalManager.degTargetBed());
            LCD_SERIAL_2.printf(temp);
            LCD_SERIAL_2.printf("\xff\xff\xff");

            memset(temp,0,sizeof(temp));
            sprintf(temp, "pretemp.nozzle.txt=\"%d\"",thermalManager.degTargetHotend(0));
            LCD_SERIAL_2.printf(temp);
            LCD_SERIAL_2.printf("\xff\xff\xff");

            memset(temp,0,sizeof(temp));
            sprintf(temp, "pretemp.bed.txt=\"%d\"", thermalManager.degTargetBed());
            LCD_SERIAL_2.printf(temp);
            LCD_SERIAL_2.printf("\xff\xff\xff");
          #endif
        }
        else if(recdat.data[0] == 11) //预热PETG
        {
          e_temp   = abs_extrusion_temp;
          bed_temp = abs_bed_temp;
          RTS_SndData(e_temp, HEAD0_SET_TEMP_VP);
          RTS_SndData(bed_temp, BED_SET_TEMP_VP);
          thermalManager.setTargetHotend(e_temp, ExtUI::extruder_t::E0);
          thermalManager.setTargetBed(bed_temp);

          #if ENABLED(TJC_AVAILABLE) 
            //pretemp
            memset(temp,0,sizeof(temp));
            sprintf(temp, "pretemp.nozzletemp.txt=\"%d / %d\"", thermalManager.wholeDegHotend(0) , thermalManager.degTargetHotend(0));
            LCD_SERIAL_2.printf(temp);
            LCD_SERIAL_2.printf("\xff\xff\xff");

            memset(temp,0,sizeof(temp));
            sprintf(temp, "pretemp.bedtemp.txt=\"%d / %d\"", thermalManager.wholeDegBed() , thermalManager.degTargetBed());
            LCD_SERIAL_2.printf(temp);
            LCD_SERIAL_2.printf("\xff\xff\xff");

            memset(temp,0,sizeof(temp));
            sprintf(temp, "pretemp.nozzle.txt=\"%d\"",thermalManager.degTargetHotend(0));
            LCD_SERIAL_2.printf(temp);
            LCD_SERIAL_2.printf("\xff\xff\xff");

            memset(temp,0,sizeof(temp));
            sprintf(temp, "pretemp.bed.txt=\"%d\"", thermalManager.degTargetBed());
            LCD_SERIAL_2.printf(temp);
            LCD_SERIAL_2.printf("\xff\xff\xff");
          #endif
        }
        else if(recdat.data[0] == 12) //预热TPU
        {
          e_temp   = tpu_extrusion_temp;
          bed_temp = tpu_bed_temp;
          RTS_SndData(e_temp, HEAD0_SET_TEMP_VP);
          RTS_SndData(bed_temp, BED_SET_TEMP_VP);
          thermalManager.setTargetHotend(e_temp, ExtUI::extruder_t::E0);
          thermalManager.setTargetBed(bed_temp);

          #if ENABLED(TJC_AVAILABLE) 
            //pretemp
            memset(temp,0,sizeof(temp));
            sprintf(temp, "pretemp.nozzletemp.txt=\"%d / %d\"", thermalManager.wholeDegHotend(0) , thermalManager.degTargetHotend(0));
            LCD_SERIAL_2.printf(temp);
            LCD_SERIAL_2.printf("\xff\xff\xff");

            memset(temp,0,sizeof(temp));
            sprintf(temp, "pretemp.bedtemp.txt=\"%d / %d\"", thermalManager.wholeDegBed() , thermalManager.degTargetBed());
            LCD_SERIAL_2.printf(temp);
            LCD_SERIAL_2.printf("\xff\xff\xff");

            memset(temp,0,sizeof(temp));
            sprintf(temp, "pretemp.nozzle.txt=\"%d\"",thermalManager.degTargetHotend(0));
            LCD_SERIAL_2.printf(temp);
            LCD_SERIAL_2.printf("\xff\xff\xff");

            memset(temp,0,sizeof(temp));
            sprintf(temp, "pretemp.bed.txt=\"%d\"", thermalManager.degTargetBed());
            LCD_SERIAL_2.printf(temp);
            LCD_SERIAL_2.printf("\xff\xff\xff");
          #endif
        }
        else if(recdat.data[0] == 13) //显示默认PLA温度
        {
          temp_set_flag = 0x01;
          RTS_SndData(pla_extrusion_temp, PRHEAT_NOZZLE_TEMP_VP);
          RTS_SndData(pla_bed_temp, PRHEAT_BED_TEMP_VP);
          RTS_SndData(ExchangePageBase + 35, ExchangepageAddr);

          #if ENABLED(TJC_AVAILABLE)
            unit = 10;

            memset(temp,0,sizeof(temp));
            sprintf(temp, "tempsetvalue.nozzletemp.val=%d", (int)pla_extrusion_temp);
            LCD_SERIAL_2.printf(temp);
            LCD_SERIAL_2.printf("\xff\xff\xff");

            memset(temp,0,sizeof(temp));
            sprintf(temp, "tempsetvalue.bedtemp.val=%d", pla_bed_temp);
            LCD_SERIAL_2.printf(temp);
            LCD_SERIAL_2.printf("\xff\xff\xff"); 

            LCD_SERIAL_2.printf("page tempsetvalue");
            LCD_SERIAL_2.printf("\xff\xff\xff");                                     
          #endif 
        }
        else if(recdat.data[0] == 14) //显示默认PETG温度
        {
          temp_set_flag = 0x02;
          RTS_SndData(petg_extrusion_temp, PRHEAT_NOZZLE_TEMP_VP);
          RTS_SndData(petg_bed_temp, PRHEAT_BED_TEMP_VP);
          RTS_SndData(ExchangePageBase + 35, ExchangepageAddr);

          #if ENABLED(TJC_AVAILABLE)
            unit = 10;

            memset(temp,0,sizeof(temp));
            sprintf(temp, "tempsetvalue.nozzletemp.val=%d", (int)petg_extrusion_temp);
            LCD_SERIAL_2.printf(temp);
            LCD_SERIAL_2.printf("\xff\xff\xff");

            memset(temp,0,sizeof(temp));
            sprintf(temp, "tempsetvalue.bedtemp.val=%d", petg_bed_temp);
            LCD_SERIAL_2.printf(temp);
            LCD_SERIAL_2.printf("\xff\xff\xff"); 

            LCD_SERIAL_2.printf("page tempsetvalue");
            LCD_SERIAL_2.printf("\xff\xff\xff");                                     
          #endif 
        }
        else if(recdat.data[0] == 15) //显示默认ABS温度
        {
          temp_set_flag = 0x03;
          RTS_SndData(abs_extrusion_temp, PRHEAT_NOZZLE_TEMP_VP);
          RTS_SndData(abs_bed_temp, PRHEAT_BED_TEMP_VP);
          RTS_SndData(ExchangePageBase + 35, ExchangepageAddr);

          #if ENABLED(TJC_AVAILABLE)
            unit = 10;

            memset(temp,0,sizeof(temp));
            sprintf(temp, "tempsetvalue.nozzletemp.val=%d", (int)abs_extrusion_temp);
            LCD_SERIAL_2.printf(temp);
            LCD_SERIAL_2.printf("\xff\xff\xff");

            memset(temp,0,sizeof(temp));
            sprintf(temp, "tempsetvalue.bedtemp.val=%d", abs_bed_temp);
            LCD_SERIAL_2.printf(temp);
            LCD_SERIAL_2.printf("\xff\xff\xff"); 

            LCD_SERIAL_2.printf("page tempsetvalue");
            LCD_SERIAL_2.printf("\xff\xff\xff");                                     
          #endif 
        }
        else if(recdat.data[0] == 16) //显示默认TPU温度
        {
          temp_set_flag = 0x04;
          RTS_SndData(tpu_extrusion_temp, PRHEAT_NOZZLE_TEMP_VP);
          RTS_SndData(tpu_bed_temp, PRHEAT_BED_TEMP_VP);
          RTS_SndData(ExchangePageBase + 35, ExchangepageAddr);

          #if ENABLED(TJC_AVAILABLE)
            unit = 10;

            memset(temp,0,sizeof(temp));
            sprintf(temp, "tempsetvalue.nozzletemp.val=%d", (int)tpu_extrusion_temp);
            LCD_SERIAL_2.printf(temp);
            LCD_SERIAL_2.printf("\xff\xff\xff");

            memset(temp,0,sizeof(temp));
            sprintf(temp, "tempsetvalue.bedtemp.val=%d", tpu_bed_temp);
            LCD_SERIAL_2.printf(temp);
            LCD_SERIAL_2.printf("\xff\xff\xff"); 

            LCD_SERIAL_2.printf("page tempsetvalue");
            LCD_SERIAL_2.printf("\xff\xff\xff");                                     
          #endif 
        }
        else if(recdat.data[0] == 17) //默认调平温度
        {
          temp_set_flag = 0x05;
          RTS_SndData(probe_extrusion_temp, PRHEAT_NOZZLE_TEMP_VP);
          RTS_SndData(probe_bed_temp, PRHEAT_BED_TEMP_VP);          
          RTS_SndData(ExchangePageBase + 35, ExchangepageAddr);

          #if ENABLED(TJC_AVAILABLE)
            unit = 10;

            memset(temp,0,sizeof(temp));
            sprintf(temp, "tempsetvalue.nozzletemp.val=%d", (int)probe_extrusion_temp);
            LCD_SERIAL_2.printf(temp);
            LCD_SERIAL_2.printf("\xff\xff\xff");

            memset(temp,0,sizeof(temp));
            sprintf(temp, "tempsetvalue.bedtemp.val=%d", probe_bed_temp);
            LCD_SERIAL_2.printf(temp);
            LCD_SERIAL_2.printf("\xff\xff\xff"); 

            LCD_SERIAL_2.printf("page tempsetvalue");
            LCD_SERIAL_2.printf("\xff\xff\xff");                                     
          #endif
        }

      }              
      break;

      case Heater0TempEnterKey:
      {
        #if ENABLED(TJC_AVAILABLE)
          thermalManager.temp_hotend[0].target = ( ((recdat.data[0] & 0xFF00) >> 8) | ((recdat.data[0] & 0x00FF)<<8));
          thermalManager.setTargetHotend(thermalManager.temp_hotend[0].target, 0);

          memset(temp,0,sizeof(temp));
          sprintf(temp, "pretemp.nozzletemp.txt=\"%d / %d\"", thermalManager.wholeDegHotend(0) , thermalManager.degTargetHotend(0));
          LCD_SERIAL_2.printf(temp);
          LCD_SERIAL_2.printf("\xff\xff\xff");

        #else
          thermalManager.temp_hotend[0].target = recdat.data[0];
          thermalManager.setTargetHotend(thermalManager.temp_hotend[0].target, 0);
          RTS_SndData(thermalManager.temp_hotend[0].target, HEAD0_SET_TEMP_VP);
        #endif
      }
      break;

      case Heater1TempEnterKey:
      {
       #if HAS_MULTI_HOTEND
          thermalManager.temp_hotend[1].target = recdat.data[0];
          thermalManager.setTargetHotend(thermalManager.temp_hotend[1].target, 1);
          RTS_SndData(thermalManager.temp_hotend[1].target, HEAD1_SET_TEMP_VP);
       #endif
      }
      break;

      case HotBedTempEnterKey:
      {
        #if ENABLED(TJC_AVAILABLE)
          thermalManager.temp_bed.target = ( ((recdat.data[0] & 0xFF00) >> 8) | ((recdat.data[0] & 0x00FF)<<8));
          thermalManager.setTargetBed(thermalManager.temp_bed.target);

          memset(temp,0,sizeof(temp));
          sprintf(temp, "pretemp.bedtemp.txt=\"%d / %d\"", thermalManager.wholeDegBed() , thermalManager.degTargetBed());
          LCD_SERIAL_2.printf(temp);
          LCD_SERIAL_2.printf("\xff\xff\xff");
        #else
          thermalManager.temp_bed.target = recdat.data[0];
          thermalManager.setTargetBed(thermalManager.temp_bed.target);
          RTS_SndData(thermalManager.temp_bed.target, BED_SET_TEMP_VP);
        #endif
      }
      break;

      case SetPreNozzleTemp:
      {
        if(temp_set_flag == 0x01)
        {
          #if ENABLED(TJC_AVAILABLE)
            if(recdat.data[0]==1)
            {
              pla_extrusion_temp = (pla_extrusion_temp + unit);
              if(pla_extrusion_temp>280)
              {
                pla_extrusion_temp = 280;
              }
            }
            else if(recdat.data[0]==2)
            {
              pla_extrusion_temp = (pla_extrusion_temp - unit);
              if(pla_extrusion_temp<160)
              {
                pla_extrusion_temp = 160;
              }        
            }
            memset(temp,0,sizeof(temp));
            sprintf(temp, "tempsetvalue.nozzletemp.val=%d", (int)pla_extrusion_temp);
            LCD_SERIAL_2.printf(temp);
            LCD_SERIAL_2.printf("\xff\xff\xff");
          #else
            pla_extrusion_temp = recdat.data[0];
            RTS_SndData(pla_extrusion_temp, PRHEAT_NOZZLE_TEMP_VP);
          #endif
        }
        else if(temp_set_flag == 0x02)
        {
          #if ENABLED(TJC_AVAILABLE)
            if(recdat.data[0]==1)
            {
              petg_extrusion_temp = (petg_extrusion_temp + unit);
              if(petg_extrusion_temp>280)
              {
                petg_extrusion_temp = 280;
              }
            }
            else if(recdat.data[0]==2)
            {
              petg_extrusion_temp = (petg_extrusion_temp - unit);
              if(pla_extrusion_temp<160)
              {
                petg_extrusion_temp = 160;
              }        
            }
            memset(temp,0,sizeof(temp));
            sprintf(temp, "tempsetvalue.nozzletemp.val=%d", (int)petg_extrusion_temp);
            LCD_SERIAL_2.printf(temp);
            LCD_SERIAL_2.printf("\xff\xff\xff");
          #else
            petg_extrusion_temp = recdat.data[0];
            RTS_SndData(petg_extrusion_temp, PRHEAT_NOZZLE_TEMP_VP);
          #endif        
        }
        else if(temp_set_flag == 0x03)
        {
          #if ENABLED(TJC_AVAILABLE)
            if(recdat.data[0]==1)
            {
              abs_extrusion_temp = (abs_extrusion_temp + unit);
              if(abs_extrusion_temp>280)
              {
                abs_extrusion_temp = 280;
              }
            }
            else if(recdat.data[0]==2)
            {
              abs_extrusion_temp = (abs_extrusion_temp - unit);
              if(abs_extrusion_temp<160)
              {
                abs_extrusion_temp = 160;
              }        
            }
            memset(temp,0,sizeof(temp));
            sprintf(temp, "tempsetvalue.nozzletemp.val=%d", (int)abs_extrusion_temp);
            LCD_SERIAL_2.printf(temp);
            LCD_SERIAL_2.printf("\xff\xff\xff");
          #else
            abs_extrusion_temp = recdat.data[0];
            RTS_SndData(abs_extrusion_temp, PRHEAT_NOZZLE_TEMP_VP);
          #endif      
        }
        else if(temp_set_flag == 0x04)
        {
          #if ENABLED(TJC_AVAILABLE)
            if(recdat.data[0]==1)
            {
              tpu_extrusion_temp = (tpu_extrusion_temp + unit);
              if(tpu_extrusion_temp>280)
              {
                tpu_extrusion_temp = 280;
              }
            }
            else if(recdat.data[0]==2)
            {
              tpu_extrusion_temp = (tpu_extrusion_temp - unit);
              if(tpu_extrusion_temp<160)
              {
                tpu_extrusion_temp = 160;
              }        
            }
            memset(temp,0,sizeof(temp));
            sprintf(temp, "tempsetvalue.nozzletemp.val=%d", (int)tpu_extrusion_temp);
            LCD_SERIAL_2.printf(temp);
            LCD_SERIAL_2.printf("\xff\xff\xff");
          #else
            tpu_extrusion_temp = recdat.data[0];
            RTS_SndData(tpu_extrusion_temp, PRHEAT_NOZZLE_TEMP_VP); 
          #endif    
        }
        else if(temp_set_flag == 0x05)
        {
          #if ENABLED(TJC_AVAILABLE)
            if(recdat.data[0]==1)
            {
              probe_extrusion_temp = (probe_extrusion_temp + unit);
              if(probe_extrusion_temp>280)
              {
                probe_extrusion_temp = 280;
              }
            }
            else if(recdat.data[0]==2)
            {
              probe_extrusion_temp = (probe_extrusion_temp - unit);
              if(probe_extrusion_temp<140)
              {
                probe_extrusion_temp = 140;
              }        
            }
            memset(temp,0,sizeof(temp));
            sprintf(temp, "tempsetvalue.nozzletemp.val=%d", (int)probe_extrusion_temp);
            LCD_SERIAL_2.printf(temp);
            LCD_SERIAL_2.printf("\xff\xff\xff");
          #else
            probe_extrusion_temp = recdat.data[0];
            RTS_SndData(probe_extrusion_temp, PRHEAT_NOZZLE_TEMP_VP); 
          #endif    
        }    
      }
      break;

      case SetPreBedTemp:
      {
        if(temp_set_flag == 0x01)
        {
          #if ENABLED(TJC_AVAILABLE)
            if(recdat.data[0]==1)
            {
              pla_bed_temp = (pla_bed_temp + unit);
              if(pla_bed_temp>110)
              {
                pla_bed_temp = 110;
              }
            }
            else if(recdat.data[0]==2)
            {
              pla_bed_temp = (pla_bed_temp - unit);
              if(pla_bed_temp<50)
              {
                pla_bed_temp = 50;
              }        
            }
            memset(temp,0,sizeof(temp));
            sprintf(temp, "tempsetvalue.bedtemp.val=%d", (int)pla_bed_temp);
            LCD_SERIAL_2.printf(temp);
            LCD_SERIAL_2.printf("\xff\xff\xff");
          #else
            pla_bed_temp = recdat.data[0];
            RTS_SndData(pla_bed_temp, PRHEAT_BED_TEMP_VP);
          #endif  
        }
        else if(temp_set_flag == 0x02)
        {
          #if ENABLED(TJC_AVAILABLE)
            if(recdat.data[0]==1)
            {
              petg_bed_temp = (petg_bed_temp + unit);
              if(petg_bed_temp>110)
              {
                petg_bed_temp = 110;
              }
            }
            else if(recdat.data[0]==2)
            {
              petg_bed_temp = (petg_bed_temp - unit);
              if(petg_bed_temp<50)
              {
                petg_bed_temp = 50;
              }        
            }
            memset(temp,0,sizeof(temp));
            sprintf(temp, "tempsetvalue.bedtemp.val=%d", (int)petg_bed_temp);
            LCD_SERIAL_2.printf(temp);
            LCD_SERIAL_2.printf("\xff\xff\xff");
          #else
            petg_bed_temp = recdat.data[0];
            RTS_SndData(petg_bed_temp, PRHEAT_BED_TEMP_VP);
          #endif           
        }
        else if(temp_set_flag == 0x03)
        {
          #if ENABLED(TJC_AVAILABLE)
            if(recdat.data[0]==1)
            {
              abs_bed_temp = (abs_bed_temp + unit);
              if(abs_bed_temp>110)
              {
                abs_bed_temp = 110;
              }
            }
            else if(recdat.data[0]==2)
            {
              abs_bed_temp = (abs_bed_temp - unit);
              if(abs_bed_temp<50)
              {
                abs_bed_temp = 50;
              }        
            }
            memset(temp,0,sizeof(temp));
            sprintf(temp, "tempsetvalue.bedtemp.val=%d", (int)abs_bed_temp);
            LCD_SERIAL_2.printf(temp);
            LCD_SERIAL_2.printf("\xff\xff\xff");
          #else
            abs_bed_temp = recdat.data[0];
            RTS_SndData(abs_bed_temp, PRHEAT_BED_TEMP_VP);
          #endif       
        }
        else if(temp_set_flag == 0x04)
        {
          #if ENABLED(TJC_AVAILABLE)
            if(recdat.data[0]==1)
            {
              tpu_bed_temp = (tpu_bed_temp + unit);
              if(tpu_bed_temp>110)
              {
                tpu_bed_temp = 110;
              }
            }
            else if(recdat.data[0]==2)
            {
              tpu_bed_temp = (tpu_bed_temp - unit);
              if(tpu_bed_temp<50)
              {
                tpu_bed_temp = 50;
              }        
            }
            memset(temp,0,sizeof(temp));
            sprintf(temp, "tempsetvalue.bedtemp.val=%d", (int)tpu_bed_temp);
            LCD_SERIAL_2.printf(temp);
            LCD_SERIAL_2.printf("\xff\xff\xff"); 
          #else
            tpu_bed_temp = recdat.data[0];
            RTS_SndData(tpu_bed_temp, PRHEAT_BED_TEMP_VP); 
          #endif     
        }
        else if(temp_set_flag == 0x05)
        {
          #if ENABLED(TJC_AVAILABLE)
            if(recdat.data[0]==1)
            {
              probe_bed_temp = (probe_bed_temp + unit);
              if(probe_bed_temp>110)
              {
                probe_bed_temp = 110;
              }
            }
            else if(recdat.data[0]==2)
            {
              probe_bed_temp = (probe_bed_temp - unit);
              if(probe_bed_temp<50)
              {
                probe_bed_temp = 50;
              }        
            }
            memset(temp,0,sizeof(temp));
            sprintf(temp, "tempsetvalue.bedtemp.val=%d", (int)probe_bed_temp);
            LCD_SERIAL_2.printf(temp);
            LCD_SERIAL_2.printf("\xff\xff\xff");
          #else
            probe_bed_temp = recdat.data[0];
            RTS_SndData(probe_bed_temp, PRHEAT_BED_TEMP_VP); 
          #endif   
        }   
      }
      break;

      case Heater0LoadEnterKey:
      {
        #if ENABLED(TJC_AVAILABLE)
          Filament0LOAD = ( ((recdat.data[0] & 0xFF00) >> 8) | ((recdat.data[0] & 0x00FF)<<8));
          memset(temp,0,sizeof(temp));
          sprintf(temp, "prefilament.filamentlength.txt=\"%d\"", (int)Filament0LOAD);
          LCD_SERIAL_2.printf(temp);
          LCD_SERIAL_2.printf("\xff\xff\xff");
        #else
          Filament0LOAD = ((float)recdat.data[0]) / 10;
        #endif
      }
      break;

      case Heater1LoadEnterKey:
      {
        #if HAS_MULTI_HOTEND
          Filament1LOAD = ((float)recdat.data[0]) / 10;
        #endif

        #if ENABLED(TJC_AVAILABLE)
          manual_feedrate_mm_m[E_AXIS] = ( ((recdat.data[0] & 0xFF00) >> 8) | ((recdat.data[0] & 0x00FF)<<8));
          memset(temp,0,sizeof(temp));
          sprintf(temp, "prefilament.filamentspeed.txt=\"%d\"", (int)manual_feedrate_mm_m[E_AXIS]);
          LCD_SERIAL_2.printf(temp);
          LCD_SERIAL_2.printf("\xff\xff\xff");         
        #else
          manual_feedrate_mm_m[E_AXIS] = ((float)recdat.data[0] / 10);
        #endif
      }
      break;

      case SelectLanguageKey:
      {
        if(recdat.data[0] == 1)
        {
          RTS_SndData(1, VP_LANGUGE_SELECT_1);
          RTS_SndData(0, VP_LANGUGE_SELECT_2);
          RTS_SndData(0, VP_LANGUGE_SELECT_3);
          RTS_SndData(0, VP_LANGUGE_SELECT_4);
          RTS_SndData(0, VP_LANGUGE_SELECT_5);
          RTS_SndData(0, VP_LANGUGE_SELECT_6);
          RTS_SndData(0, VP_LANGUGE_SELECT_7);
          RTS_SndData(0, VP_LANGUGE_SELECT_8);
        }
        else if(recdat.data[0] == 2)
        {
          RTS_SndData(0, VP_LANGUGE_SELECT_1);
          RTS_SndData(1, VP_LANGUGE_SELECT_2);
          RTS_SndData(0, VP_LANGUGE_SELECT_3);
          RTS_SndData(0, VP_LANGUGE_SELECT_4);
          RTS_SndData(0, VP_LANGUGE_SELECT_5);
          RTS_SndData(0, VP_LANGUGE_SELECT_6);
          RTS_SndData(0, VP_LANGUGE_SELECT_7);
          RTS_SndData(0, VP_LANGUGE_SELECT_8);
        }
        else if(recdat.data[0] == 3)
        {
          RTS_SndData(0, VP_LANGUGE_SELECT_1);
          RTS_SndData(0, VP_LANGUGE_SELECT_2);
          RTS_SndData(1, VP_LANGUGE_SELECT_3);
          RTS_SndData(0, VP_LANGUGE_SELECT_4);
          RTS_SndData(0, VP_LANGUGE_SELECT_5);
          RTS_SndData(0, VP_LANGUGE_SELECT_6);
          RTS_SndData(0, VP_LANGUGE_SELECT_7);
          RTS_SndData(0, VP_LANGUGE_SELECT_8);
        }
        else if(recdat.data[0] == 4)
        {
          RTS_SndData(0, VP_LANGUGE_SELECT_1);
          RTS_SndData(0, VP_LANGUGE_SELECT_2);
          RTS_SndData(0, VP_LANGUGE_SELECT_3);
          RTS_SndData(1, VP_LANGUGE_SELECT_4);
          RTS_SndData(0, VP_LANGUGE_SELECT_5);
          RTS_SndData(0, VP_LANGUGE_SELECT_6);
          RTS_SndData(0, VP_LANGUGE_SELECT_7);
          RTS_SndData(0, VP_LANGUGE_SELECT_8);
        }
        else if(recdat.data[0] == 5)
        {
          RTS_SndData(0, VP_LANGUGE_SELECT_1);
          RTS_SndData(0, VP_LANGUGE_SELECT_2);
          RTS_SndData(0, VP_LANGUGE_SELECT_3);
          RTS_SndData(0, VP_LANGUGE_SELECT_4);
          RTS_SndData(1, VP_LANGUGE_SELECT_5);
          RTS_SndData(0, VP_LANGUGE_SELECT_6);
          RTS_SndData(0, VP_LANGUGE_SELECT_7);
          RTS_SndData(0, VP_LANGUGE_SELECT_8);
        }
        else if(recdat.data[0] == 6)
        {
          RTS_SndData(0, VP_LANGUGE_SELECT_1);
          RTS_SndData(0, VP_LANGUGE_SELECT_2);
          RTS_SndData(0, VP_LANGUGE_SELECT_3);
          RTS_SndData(0, VP_LANGUGE_SELECT_4);
          RTS_SndData(0, VP_LANGUGE_SELECT_5);
          RTS_SndData(1, VP_LANGUGE_SELECT_6);
          RTS_SndData(0, VP_LANGUGE_SELECT_7);
          RTS_SndData(0, VP_LANGUGE_SELECT_8);
        }
        else if(recdat.data[0] == 7)
        {
          RTS_SndData(0, VP_LANGUGE_SELECT_1);
          RTS_SndData(0, VP_LANGUGE_SELECT_2);
          RTS_SndData(0, VP_LANGUGE_SELECT_3);
          RTS_SndData(0, VP_LANGUGE_SELECT_4);
          RTS_SndData(0, VP_LANGUGE_SELECT_5);
          RTS_SndData(0, VP_LANGUGE_SELECT_6);
          RTS_SndData(1, VP_LANGUGE_SELECT_7);
          RTS_SndData(0, VP_LANGUGE_SELECT_8);
        }
        else if(recdat.data[0] == 8)
        {
          RTS_SndData(0, VP_LANGUGE_SELECT_1);
          RTS_SndData(0, VP_LANGUGE_SELECT_2);
          RTS_SndData(0, VP_LANGUGE_SELECT_3);
          RTS_SndData(0, VP_LANGUGE_SELECT_4);
          RTS_SndData(0, VP_LANGUGE_SELECT_5);
          RTS_SndData(0, VP_LANGUGE_SELECT_6);
          RTS_SndData(0, VP_LANGUGE_SELECT_7);
          RTS_SndData(1, VP_LANGUGE_SELECT_8);
        }
      }
      break;

      case AxisPageSelectKey:
      {
        if(recdat.data[0] == 1)
        {
          AxisUnitMode = 1;
          axis_unit = 0.1;
        }
        else if(recdat.data[0] == 2)
        {
          AxisUnitMode = 2;
          axis_unit = 1.0;
        }
        else if(recdat.data[0] == 3)
        {
          AxisUnitMode = 3;
          axis_unit = 10.0;
        }
        else if(recdat.data[0] == 4)
        {
          waitway = 4;
          AutoHomeIconNum = 0;
          queue.enqueue_now_P(PSTR("G28"));
          Update_Time_Value = 0;
          RTS_SndData(ExchangePageBase + 32, ExchangepageAddr);
          RTS_SndData(0, MOTOR_FREE_ICON_VP);

          #if ENABLED(TJC_AVAILABLE) 
            LCD_SERIAL_2.printf("page autohome");
            LCD_SERIAL_2.printf("\xff\xff\xff");               
          #endif
        }
        else if(recdat.data[0] == 5)
        {
          queue.enqueue_now_P(PSTR("G28 X"));
        }
        else if(recdat.data[0] == 6)
        {
          queue.enqueue_now_P(PSTR("G28 Y"));
        }
        else if(recdat.data[0] == 7)
        {
          queue.enqueue_now_P(PSTR("G28 Z"));
        }
      }           
      break;

      case SettingScreenKey:
      {
        if(recdat.data[0] == 1)
        {
          // Motor Icon
          RTS_SndData(0, MOTOR_FREE_ICON_VP);
          // only for prohibiting to receive massage
          waitway = 6;
          AutoHomeIconNum = 0;
          //active_extruder = 0;
          active_extruder_flag = false;
          active_extruder_font = active_extruder;
          Update_Time_Value = 0;
          queue.enqueue_now_P(PSTR("G28"));
          queue.enqueue_now_P(PSTR("G1 F200 Z0.0"));
          RTS_SndData(ExchangePageBase + 32, ExchangepageAddr);

          #if ENABLED(TJC_AVAILABLE) 
          
            LCD_SERIAL_2.printf("page autohome");
            LCD_SERIAL_2.printf("\xff\xff\xff");

            #if ENABLED(NEPTUNE_3_PLUS)
              LCD_SERIAL_2.printf("leveling.va1.val=2");
              LCD_SERIAL_2.printf("\xff\xff\xff");  
            #elif ENABLED(NEPTUNE_3_PRO)
              LCD_SERIAL_2.printf("leveling.va1.val=1");
              LCD_SERIAL_2.printf("\xff\xff\xff");  
            #elif ENABLED(NEPTUNE_3_MAX)
              LCD_SERIAL_2.printf("leveling.va1.val=3");
              LCD_SERIAL_2.printf("\xff\xff\xff");  
            #endif

          #endif

          if (active_extruder == 0)
          {
            RTS_SndData(0, EXCHANGE_NOZZLE_ICON_VP);
          }
          else
          {
            RTS_SndData(1, EXCHANGE_NOZZLE_ICON_VP);
          }
        }
        else if(recdat.data[0] == 2)
        {
          Filament0LOAD = 10;
          Filament1LOAD = 10;

          #if HAS_HOTEND
            RTS_SndData(thermalManager.temp_hotend[0].celsius, HEAD0_CURRENT_TEMP_VP);
            thermalManager.setTargetHotend(thermalManager.temp_hotend[0].target, 0);
            RTS_SndData(thermalManager.temp_hotend[0].target, HEAD0_SET_TEMP_VP);
            RTS_SndData(10 * Filament0LOAD, HEAD0_FILAMENT_LOAD_DATA_VP);
          #endif

          #if HAS_MULTI_HOTEND
            RTS_SndData(thermalManager.temp_hotend[1].celsius, HEAD1_CURRENT_TEMP_VP);
            thermalManager.setTargetHotend(thermalManager.temp_hotend[1].target, 1);
            RTS_SndData(thermalManager.temp_hotend[1].target, HEAD1_SET_TEMP_VP);
            RTS_SndData(10 * Filament1LOAD, HEAD1_FILAMENT_LOAD_DATA_VP);
          #endif

          delay(2);
          //RTS_SndData(ExchangePageBase + 23, ExchangepageAddr);
          RTS_SndData(ExchangePageBase + 32, ExchangepageAddr);

          #if ENABLED(TJC_AVAILABLE) 
            LCD_SERIAL_2.printf("page autohome");
            LCD_SERIAL_2.printf("\xff\xff\xff");               
          #endif
        }
        else if (recdat.data[0] == 3)
        {
          #if ENABLED(TJC_AVAILABLE) 
            LCD_SERIAL_2.printf("page premove");
            LCD_SERIAL_2.printf("\xff\xff\xff"); 

            LCD_SERIAL_2.printf("premove.unit_move.val=2"); //默认移动单位1mm
            LCD_SERIAL_2.printf("\xff\xff\xff"); 
          #endif
        
          if(active_extruder == 0)
          {
            RTS_SndData(0, EXCHANGE_NOZZLE_ICON_VP);
            active_extruder_flag = false;
          }
          else if(active_extruder == 1)
          {
            RTS_SndData(1, EXCHANGE_NOZZLE_ICON_VP);
            active_extruder_flag = true;
          }

          AxisUnitMode = 1;

          if(active_extruder == 0)
          {
            #if ENABLED(DUAL_X_CARRIAGE)
              if(TEST(axis_known_position, X_AXIS))
              {
                current_position_x0_axis = current_position[X_AXIS];
              }
              else
              {
                current_position[X_AXIS] = current_position_x0_axis;
              }
              RTS_SndData(10 * current_position_x0_axis, AXIS_X_COORD_VP);
              memset(commandbuf, 0, sizeof(commandbuf));
              sprintf_P(commandbuf, PSTR("G92.9 X%6.3f"), current_position_x0_axis);
              queue.enqueue_one_now(commandbuf);
            #endif
          }
          else if(active_extruder == 1)
          {
            #if ENABLED(DUAL_X_CARRIAGE)
              if(TEST(axis_known_position, X_AXIS))
              {
                current_position_x1_axis = current_position[X_AXIS];
              }
              else
              {
                current_position[X_AXIS] = current_position_x1_axis;
              }
              RTS_SndData(10 * current_position_x1_axis, AXIS_X_COORD_VP);
              memset(commandbuf, 0, sizeof(commandbuf));
              sprintf_P(commandbuf, PSTR("G92.9 X%6.3f"), current_position_x1_axis);
              queue.enqueue_one_now(commandbuf);
            #endif
          }
          RTS_SndData(10 * current_position[Y_AXIS], AXIS_Y_COORD_VP);
          RTS_SndData(10 * current_position[Z_AXIS], AXIS_Z_COORD_VP);
        }
        else if (recdat.data[0] == 4)
        {
          RTS_SndData(ExchangePageBase + 35, ExchangepageAddr);
        }
        else if (recdat.data[0] == 5)
        {
          RTS_SndData(CORP_WEBSITE, PRINTER_WEBSITE_TEXT_VP);
          RTS_SndData(ExchangePageBase + 33, ExchangepageAddr);
        }
        else if (recdat.data[0] == 6)
        {
          queue.enqueue_now_P(PSTR("M84"));
          RTS_SndData(1, MOTOR_FREE_ICON_VP);
        }
        else if (recdat.data[0] == 7)
        { 
          if (thermalManager.fan_speed[0])
          {
            RTS_SndData(1, HEAD0_FAN_ICON_VP);
            thermalManager.set_fan_speed(0, 0);

            #if ENABLED(TJC_AVAILABLE) 
              LCD_SERIAL_2.printf("set.va0.val=0");
              LCD_SERIAL_2.printf("\xff\xff\xff");               
            #endif    
          }
          else
          {
            RTS_SndData(0, HEAD0_FAN_ICON_VP);
            thermalManager.set_fan_speed(0, 255);
            #if ENABLED(TJC_AVAILABLE) 
              LCD_SERIAL_2.printf("set.va0.val=1");
              LCD_SERIAL_2.printf("\xff\xff\xff");               
            #endif   
          }  
        }
        else if(recdat.data[0] == 8)
        {
          if(enable_filment_check)
          {
            enable_filment_check = false;
            #if ENABLED(TJC_AVAILABLE) 
              LCD_SERIAL_2.printf("set.va1.val=0");
              LCD_SERIAL_2.printf("\xff\xff\xff");               
            #endif            
          }
          else
          {
            enable_filment_check = true;
            #if ENABLED(TJC_AVAILABLE) 
              LCD_SERIAL_2.printf("set.va1.val=1");
              LCD_SERIAL_2.printf("\xff\xff\xff");               
            #endif               
          }
        }
        else if(recdat.data[0] == 9)
        {
          RTS_SndData(ExchangePageBase + 30, ExchangepageAddr);

          #if ENABLED(TJC_AVAILABLE)
            LCD_SERIAL_2.printf("page pretemp");
            LCD_SERIAL_2.printf("\xff\xff\xff");
          #endif

          if(thermalManager.wholeDegHotend(0) < 0)
          {
            #if ENABLED(TJC_AVAILABLE)
              LCD_SERIAL_2.printf("page err_nozzleunde");
              LCD_SERIAL_2.printf("\xff\xff\xff");
            #endif
            break;
          }
          else if(thermalManager.wholeDegBed() < 0)
          {
            #if ENABLED(TJC_AVAILABLE)
              LCD_SERIAL_2.printf("page err_bedunder");
              LCD_SERIAL_2.printf("\xff\xff\xff");
            #endif
            break;
          }          

        }
        else if(recdat.data[0] == 0x0A)
        {
          RTS_SndData(ExchangePageBase + 31, ExchangepageAddr);

          #if ENABLED(TJC_AVAILABLE)
            LCD_SERIAL_2.printf("page prefilament");
            LCD_SERIAL_2.printf("\xff\xff\xff");

            memset(temp,0,sizeof(temp));
            sprintf(temp, "prefilament.filamentlength.txt=\"%d\"", (int)Filament0LOAD);
            LCD_SERIAL_2.printf(temp);
            LCD_SERIAL_2.printf("\xff\xff\xff");

            memset(temp,0,sizeof(temp));
            sprintf(temp, "prefilament.filamentspeed.txt=\"%d\"", (int)manual_feedrate_mm_m[E_AXIS]);
            LCD_SERIAL_2.printf(temp);
            LCD_SERIAL_2.printf("\xff\xff\xff");    
          #endif
        }
        else if(recdat.data[0] == 0x0B)
        {
          LCD_SERIAL_2.printf("page set");
          LCD_SERIAL_2.printf("\xff\xff\xff");    
        }
        else if(recdat.data[0] == 0x0C)
        {
          LCD_SERIAL_2.printf("page warn_rdlevel");
          LCD_SERIAL_2.printf("\xff\xff\xff");    
        }
        else if(recdat.data[0] == 0x0D)
        {
          #if ENABLED(TJC_AVAILABLE)
            #if ENABLED(POWER_LOSS_RECOVERY)
              if(recovery.enabled==0)
              { 
                LCD_SERIAL_2.printf("multiset.plrbutton.val=0");
                LCD_SERIAL_2.printf("\xff\xff\xff"); 
              }
              else if(recovery.enabled==1)
              {
                LCD_SERIAL_2.printf("multiset.plrbutton.val=1");
                LCD_SERIAL_2.printf("\xff\xff\xff"); 
              }              
              LCD_SERIAL_2.printf("page multiset");
              LCD_SERIAL_2.printf("\xff\xff\xff");
            #endif 
          #endif            
        }
        
      }
      break;

      case SettingBackKey:
      {
        if (recdat.data[0] == 1)
        {
          Update_Time_Value = RTS_UPDATE_VALUE;
          #if ENABLED(EEPROM_SETTINGS)
            settings.save();
          #endif
          queue.enqueue_now_P(PSTR("G1 F1000 Z15.0"));
        }
        else if (recdat.data[0] == 2)
        {
          if(!planner.has_blocks_queued())
          {
            #if ENABLED(HAS_LEVELING)
              RTS_SndData(ExchangePageBase + 22, ExchangepageAddr);
            #else
              RTS_SndData(ExchangePageBase + 21, ExchangepageAddr);
            #endif
            queue.enqueue_now_P(PSTR("M420 S1"));
          }
        }
        else if (recdat.data[0] == 3)
        {
          #if HAS_MULTI_HOTEND
            if (WITHIN((XoffsetValue), Z_PROBE_OFFSET_RANGE_MIN, Z_PROBE_OFFSET_RANGE_MAX))
            {
              //hotend_offset[1].x = X2_MAX_POS + XoffsetValue;
            }
            delay(5);
            memset(commandbuf, 0, sizeof(commandbuf));
            sprintf_P(commandbuf, PSTR("M218 T1 X%4.1f"), hotend_offset[1].x);
            queue.enqueue_now_P(commandbuf);
            delay(5);
            memset(commandbuf, 0, sizeof(commandbuf));
            sprintf_P(commandbuf, PSTR("M218 T1 Y%4.1f"), hotend_offset[1].y);
            queue.enqueue_now_P(commandbuf);
            delay(5);
            memset(commandbuf, 0, sizeof(commandbuf));
            sprintf_P(commandbuf, PSTR("M218 T1 Z%4.1f"), hotend_offset[1].z);
            queue.enqueue_now_P(commandbuf);

            RTS_SndData(ExchangePageBase + 21, ExchangepageAddr);
            #if ENABLED(EEPROM_SETTINGS)
              settings.save();
            #endif
            delay(1000);
          #endif
        }
        else if(recdat.data[0] == 4)
        {
          RTS_SndData(ExchangePageBase + 22, ExchangepageAddr);
          #if ENABLED(EEPROM_SETTINGS)
            settings.save();
          #endif
        }
        else if(recdat.data[0] == 5)
        {
          #if ENABLED(EEPROM_SETTINGS)
            settings.save();
          #endif          
        }
        else if(recdat.data[0] == 6)
        {
          temp_set_flag = 0x00;
          #if ENABLED(EEPROM_SETTINGS)
            settings.save();
          #endif          
        }
      }
      break;

      case BedLevelFunKey:
      {
        if (recdat.data[0] == 1)
        {
          planner.synchronize();
          waitway = 6;
          #if ENABLED(DUAL_X_CARRIAGE)
          if((active_extruder == 1) || (!TEST(axis_known_position, X_AXIS)) || (!TEST(axis_known_position, Y_AXIS)))
          {
            AutoHomeIconNum = 0;
            active_extruder = 0;
            active_extruder_flag = false;
            active_extruder_font = active_extruder;
            queue.enqueue_now_P(PSTR("G28"));
            RTS_SndData(ExchangePageBase + 32, ExchangepageAddr);

          #if ENABLED(TJC_AVAILABLE) 
            LCD_SERIAL_2.printf("page autohome");
            LCD_SERIAL_2.printf("\xff\xff\xff");               
          #endif
          }
          else
          #endif
          {
            queue.enqueue_now_P(PSTR("G28 Z0"));
          }
          queue.enqueue_now_P(PSTR("G1 F200 Z0.0"));
        }
        else if (recdat.data[0] == 2)
        {
          #if ENABLED(BABYSTEPPING)
            last_zoffset = probe.offset.z;
            //if (WITHIN((zprobe_zoffset + 0.1), Z_PROBE_OFFSET_RANGE_MIN, Z_PROBE_OFFSET_RANGE_MAX))
            
            if (WITHIN((probe.offset.z + zoffset_unit), Z_PROBE_OFFSET_RANGE_MIN, Z_PROBE_OFFSET_RANGE_MAX))
            {
              #if ENABLED(HAS_LEVELING)
                zprobe_zoffset = (probe.offset.z + zoffset_unit);
              #endif
              babystep.add_mm(Z_AXIS, zprobe_zoffset - last_zoffset);
              #if HAS_BED_PROBE
                probe.offset.z = zprobe_zoffset;
              #endif
            }

            RTS_SndData(zprobe_zoffset * 100, AUTO_BED_LEVEL_ZOFFSET_VP);

            #if ENABLED(TJC_AVAILABLE) 
              memset(temp,0,sizeof(temp));
              sprintf(temp, "leveldata.z_offset.val=%d", (int)(probe.offset.z * 100));
              LCD_SERIAL_2.printf(temp);
              LCD_SERIAL_2.printf("\xff\xff\xff");
              
              memset(temp,0,sizeof(temp));
              sprintf(temp, "adjustzoffset.z_offset.val=%d", (int)(probe.offset.z * 100));
              LCD_SERIAL_2.printf(temp);
              LCD_SERIAL_2.printf("\xff\xff\xff");              
            #endif
          #endif
        }
        else if (recdat.data[0] == 3)
        {
          #if ENABLED(BABYSTEPPING)
            last_zoffset = probe.offset.z;
            if (WITHIN((probe.offset.z - zoffset_unit), Z_PROBE_OFFSET_RANGE_MIN, Z_PROBE_OFFSET_RANGE_MAX))
            {
              zprobe_zoffset = (probe.offset.z - zoffset_unit);
              babystep.add_mm(Z_AXIS, zprobe_zoffset - last_zoffset);
              #if HAS_BED_PROBE
                probe.offset.z = zprobe_zoffset;
              #endif
            }
            RTS_SndData(zprobe_zoffset * 100, AUTO_BED_LEVEL_ZOFFSET_VP);
            #if ENABLED(TJC_AVAILABLE) 
              memset(temp,0,sizeof(temp));
              sprintf(temp, "leveldata.z_offset.val=%d", (int)(probe.offset.z * 100));
              LCD_SERIAL_2.printf(temp);
              LCD_SERIAL_2.printf("\xff\xff\xff");   

              memset(temp,0,sizeof(temp));
              sprintf(temp, "adjustzoffset.z_offset.val=%d", (int)(probe.offset.z * 100));
              LCD_SERIAL_2.printf(temp);
              LCD_SERIAL_2.printf("\xff\xff\xff");           
            #endif
          #endif
        }
        else if (recdat.data[0] == 4)
        {
          RTS_SndData(0, ICON_ADJUST_Z_OFFSET_UNIT);
          RTS_SndData(0, ICON_LEVEL_SELECT);
          zoffset_unit = 0.01;
          #if ENABLED(TJC_AVAILABLE) 
            LCD_SERIAL_2.printf("adjustzoffset.zoffset_value.val=1");
            LCD_SERIAL_2.printf("\xff\xff\xff");          
          #endif
        }
        else if (recdat.data[0] == 5)
        {
          RTS_SndData(1, ICON_ADJUST_Z_OFFSET_UNIT);
          RTS_SndData(1, ICON_LEVEL_SELECT);
          zoffset_unit = 0.1;
          #if ENABLED(TJC_AVAILABLE) 
            LCD_SERIAL_2.printf("adjustzoffset.zoffset_value.val=2");
            LCD_SERIAL_2.printf("\xff\xff\xff");          
          #endif
          
        }
        else if (recdat.data[0] == 6)
        {
          RTS_SndData(2, ICON_ADJUST_Z_OFFSET_UNIT);
          RTS_SndData(2, ICON_LEVEL_SELECT);
          zoffset_unit = 1;
          #if ENABLED(TJC_AVAILABLE) 
            LCD_SERIAL_2.printf("adjustzoffset.zoffset_value.val=3");
            LCD_SERIAL_2.printf("\xff\xff\xff");          
          #endif
        }
        else if (recdat.data[0] == 7) //LED2
        {
          if(status_led1)  
          {
            //关LED
            status_led1 = false;
            RTS_SndData(1, ICON_ADJUST_LED2);
            #if ENABLED(TJC_AVAILABLE) 
              LCD_SERIAL_2.printf("status_led1=0");
              LCD_SERIAL_2.printf("\xff\xff\xff");
            #endif
            #if PIN_EXISTS(LED2)
              OUT_WRITE(LED2_PIN, LOW);
            #endif
          }
          else 
          {
            //开LED
            status_led1 = true;
            RTS_SndData(0, ICON_ADJUST_LED2);
            #if ENABLED(TJC_AVAILABLE) 
              LCD_SERIAL_2.printf("status_led1=1");
              LCD_SERIAL_2.printf("\xff\xff\xff");
            #endif
            #if PIN_EXISTS(LED2)
              OUT_WRITE(LED2_PIN, HIGH);
            #endif
          }
          flag_led1_run_ctrl = 0;         
        }
        else if (recdat.data[0] == 8) //LED3
        {
          if(status_led2) 
          {
            status_led2 = false;
            RTS_SndData(1, ICON_ADJUST_LED3);
            #if ENABLED(TJC_AVAILABLE) 
              LCD_SERIAL_2.printf("status_led2=0");
              LCD_SERIAL_2.printf("\xff\xff\xff");
            #endif
            #if PIN_EXISTS(LED3)
              OUT_WRITE(LED3_PIN, LOW);
            #endif
          }
          else 
          {
            status_led2 = true;
            RTS_SndData(0, ICON_ADJUST_LED3);
            #if ENABLED(TJC_AVAILABLE) 
              LCD_SERIAL_2.printf("status_led2=1");
              LCD_SERIAL_2.printf("\xff\xff\xff");
            #endif
            #if PIN_EXISTS(LED3)
              OUT_WRITE(LED3_PIN, HIGH);
            #endif
          }
        }
        else if (recdat.data[0] == 9)
        {
          #if HAS_BED_PROBE
            waitway = 3;
            RTS_SndData(1, AUTO_BED_LEVEL_ICON_VP);
            RTS_SndData(ExchangePageBase + 38, ExchangepageAddr);
            queue.enqueue_now_P(PSTR("G29"));
            planner.synchronize();
          #endif
        }
        else if (recdat.data[0] == 10) //更新PRINTPAUSE上的一些信息 0x0A
        {
          //printspeed
          memset(temp,0,sizeof(temp));
          sprintf(temp, "printpause.printspeed.txt=\"%d\"", feedrate_percentage );
          LCD_SERIAL_2.printf(temp);
          LCD_SERIAL_2.printf("\xff\xff\xff"); 

          //fanspeed         
          memset(temp,0,sizeof(temp));
          sprintf(temp, "printpause.fanspeed.txt=\"%d\"", thermalManager.fan_speed[0] );
          LCD_SERIAL_2.printf(temp);
          LCD_SERIAL_2.printf("\xff\xff\xff");  

          //zvalue   
          memset(temp,0,sizeof(temp));
          sprintf(temp, "printpause.zvalue.val=%d", (int)(10 * current_position[Z_AXIS]));
          LCD_SERIAL_2.printf(temp);
          LCD_SERIAL_2.printf("\xff\xff\xff");

          //printtime
          #if ENABLED(RTS_AVAILABLE) 
            duration_t elapsed = print_job_timer.duration();
            rtscheck.RTS_SndData(elapsed.value / 3600, PRINT_TIME_HOUR_VP);
            rtscheck.RTS_SndData((elapsed.value % 3600) / 60, PRINT_TIME_MIN_VP);
            #if ENABLED(TJC_AVAILABLE)
              memset(temp,0,sizeof(temp));   
              sprintf(temp, "printpause.printtime.txt=\"%d h %d min\"", (int)elapsed.value/3600,(int)(elapsed.value % 3600)/60);
              LCD_SERIAL_2.printf(temp); 
              LCD_SERIAL_2.printf("\xff\xff\xff");            
            #endif  
          #endif

          //printpercent
          if(card.isPrinting() && (last_cardpercentValue != card.percentDone()))
          {
            if((unsigned char) card.percentDone() >= 0)
            {
              Percentrecord = card.percentDone();
              if(Percentrecord <= 100)
              {
                #if ENABLED(RTS_AVAILABLE) 
                  rtscheck.RTS_SndData((unsigned char)Percentrecord, PRINT_PROCESS_ICON_VP);
                  #if ENABLED(TJC_AVAILABLE)   
                    memset(temp,0,sizeof(temp));
                    sprintf(temp, "printpause.printprocess.val=%d", Percentrecord);
                    LCD_SERIAL_2.printf(temp); 
                    LCD_SERIAL_2.printf("\xff\xff\xff");

                    memset(temp,0,sizeof(temp));
                    sprintf(temp, "printpause.printvalue.txt=\"%d\"", Percentrecord);
                    LCD_SERIAL_2.printf(temp); 
                    LCD_SERIAL_2.printf("\xff\xff\xff");                
                  #endif  
                #endif
              }
            }
            else
            {
              #if ENABLED(RTS_AVAILABLE) 
                rtscheck.RTS_SndData(0, PRINT_PROCESS_ICON_VP);
                rtscheck.RTS_SndData(0, PRINT_SURPLUS_TIME_HOUR_VP);
                rtscheck.RTS_SndData(0, PRINT_SURPLUS_TIME_MIN_VP);

                #if ENABLED(TJC_AVAILABLE)
                  LCD_SERIAL_2.printf("printpause.printvalue.txt=\"0\""); 
                  LCD_SERIAL_2.printf("\xff\xff\xff");
                  LCD_SERIAL_2.printf("printpause.printprocess.val=0"); 
                  LCD_SERIAL_2.printf("\xff\xff\xff");    
                #endif  
              #endif
            }

            last_cardpercentValue = card.percentDone();
          }          
        }
        else if(recdat.data[0] == 11)  //0x0B
        {
          #if ENABLED(TJC_AVAILABLE) 
            //挤出头温度信息
            memset(temp,0,sizeof(temp));
            sprintf(temp, "main.nozzletemp.txt=\"%d / %d\"", thermalManager.wholeDegHotend(0) , thermalManager.degTargetHotend(0));
            LCD_SERIAL_2.printf(temp);
            LCD_SERIAL_2.printf("\xff\xff\xff");

            //热床温度信息
            memset(temp,0,sizeof(temp));
            sprintf(temp, "main.bedtemp.txt=\"%d / %d\"", thermalManager.wholeDegBed() , thermalManager.degTargetBed());
            LCD_SERIAL_2.printf(temp);
            LCD_SERIAL_2.printf("\xff\xff\xff");
          #endif
        }
        else if(recdat.data[0] == 12)  //0x0C
        {
          if(!flag_power_on) //已开机
          {
            #if ENABLED(TJC_AVAILABLE)
            
              //boot提示信息 
              LCD_SERIAL_2.printf("tm0.en=0");
              LCD_SERIAL_2.printf("\xff\xff\xff");
              LCD_SERIAL_2.printf("va0.val=0");
              LCD_SERIAL_2.printf("\xff\xff\xff");
              LCD_SERIAL_2.printf("tm1.en=1");
              LCD_SERIAL_2.printf("\xff\xff\xff");

              //机型信息
              #if ENABLED(NEPTUNE_3_PLUS)
                LCD_SERIAL_2.printf("main.va0.val=2");  
                LCD_SERIAL_2.printf("\xff\xff\xff");
              #elif ENABLED(NEPTUNE_3_PRO)
                LCD_SERIAL_2.printf("main.va0.val=1");  
                LCD_SERIAL_2.printf("\xff\xff\xff");                  
              #elif ENABLED(NEPTUNE_3_MAX)
                LCD_SERIAL_2.printf("main.va0.val=3");  
                LCD_SERIAL_2.printf("\xff\xff\xff");
              #endif 
            #endif
          }
        }
        else if(recdat.data[0] == 0x0D) //第1点
        {
          if (!planner.has_blocks_queued())
          {
            //waitway = 4;

            #if ENABLED(NEPTUNE_3_PLUS)
              queue.enqueue_now_P(PSTR("G28 Z0"));
              queue.enqueue_now_P(PSTR("G1 F200 Z0.0"));
            #elif ENABLED(NEPTUNE_3_PRO)
              queue.enqueue_now_P(PSTR("G28 Z0"));
              queue.enqueue_now_P(PSTR("G1 F200 Z0.0"));                
            #elif ENABLED(NEPTUNE_3_MAX)
              queue.enqueue_now_P(PSTR("G28 Z0"));
              queue.enqueue_now_P(PSTR("G1 F200 Z0.0"));
            #endif

            //waitway = 0;
          }
        }
        else if(recdat.data[0] == 0x0E) //第2点 
        {
          if (!planner.has_blocks_queued())
          {
            //waitway = 4;

            #if ENABLED(NEPTUNE_3_PLUS)
              queue.enqueue_now_P(PSTR("G1 F600 Z3")); 
              queue.enqueue_now_P(PSTR("G1 X37.5 Y32.5 F8000"));
              queue.enqueue_now_P(PSTR("G1 F200 Z0"));
            #elif ENABLED(NEPTUNE_3_PRO)
              queue.enqueue_now_P(PSTR("G1 F600 Z3")); 
              queue.enqueue_now_P(PSTR("G1 X37.5 Y215 F8000"));
              queue.enqueue_now_P(PSTR("G1 F200 Z0")); 
            #elif ENABLED(NEPTUNE_3_MAX)
              queue.enqueue_now_P(PSTR("G1 F600 Z3")); 
              queue.enqueue_now_P(PSTR("G1 X37.5 Y37.5 F8000"));
              queue.enqueue_now_P(PSTR("G1 F200 Z0"));
            #endif

            //waitway = 0;
          }
        }
        else if(recdat.data[0] == 0x0F) //第3点  
        {
          if (!planner.has_blocks_queued())
          {
            //waitway = 4;

            #if ENABLED(NEPTUNE_3_PLUS)
              queue.enqueue_now_P(PSTR("G1 F600 Z3")); 
              queue.enqueue_now_P(PSTR("G1 X37.5 Y165 F8000"));
              queue.enqueue_now_P(PSTR("G1 F200 Z0"));
            #elif ENABLED(NEPTUNE_3_PRO)
              queue.enqueue_now_P(PSTR("G1 F600 Z3")); 
              queue.enqueue_now_P(PSTR("G1 X37.5 Y37.5 F8000"));
              queue.enqueue_now_P(PSTR("G1 F200 Z0"));
            #elif ENABLED(NEPTUNE_3_MAX)
              queue.enqueue_now_P(PSTR("G1 F600 Z3")); 
              queue.enqueue_now_P(PSTR("G1 X37.5 Y215 F8000"));
              queue.enqueue_now_P(PSTR("G1 F200 Z0"));
            #endif  

            //waitway = 0;
          }
        }
        else if(recdat.data[0] == 0x10) //第4点 
        {
          if (!planner.has_blocks_queued())
          {
            //waitway = 4;

            #if ENABLED(NEPTUNE_3_PLUS)
              queue.enqueue_now_P(PSTR("G1 F600 Z3")); 
              queue.enqueue_now_P(PSTR("G1 X37.5 Y292.5 F8000"));
              queue.enqueue_now_P(PSTR("G1 F200 Z0"));
            #elif ENABLED(NEPTUNE_3_PRO)
              queue.enqueue_now_P(PSTR("G1 F600 Z3")); 
              queue.enqueue_now_P(PSTR("G1 X37.5 Y37.5 F8000"));
              queue.enqueue_now_P(PSTR("G1 F200 Z0"));
            #elif ENABLED(NEPTUNE_3_MAX)
              queue.enqueue_now_P(PSTR("G1 F600 Z3")); 
              queue.enqueue_now_P(PSTR("G1 X37.5 Y392.5 F8000"));
              queue.enqueue_now_P(PSTR("G1 F200 Z0"));
            #endif  

            //waitway = 0;
          }
        }
        else if(recdat.data[0] == 0x11) //第5点 
        {
          if (!planner.has_blocks_queued())
          {
            //waitway = 4;

            #if ENABLED(NEPTUNE_3_PLUS)
              queue.enqueue_now_P(PSTR("G1 F600 Z3")); 
              queue.enqueue_now_P(PSTR("G1 X292.5 Y297.5 F8000"));
              queue.enqueue_now_P(PSTR("G1 F200 Z0"));
            #elif ENABLED(NEPTUNE_3_PRO)
              queue.enqueue_now_P(PSTR("G1 F600 Z3")); 
              queue.enqueue_now_P(PSTR("G1 X37.5 Y165 F8000"));
              queue.enqueue_now_P(PSTR("G1 F200 Z0"));
            #elif ENABLED(NEPTUNE_3_MAX)
              queue.enqueue_now_P(PSTR("G1 F600 Z3")); 
              queue.enqueue_now_P(PSTR("G1 X392.5 Y392.5 F8000"));
              queue.enqueue_now_P(PSTR("G1 F200 Z0"));
            #endif  

            //waitway = 0;
          }
        }
        else if(recdat.data[0] == 0x12) //第6点  
        {
          if (!planner.has_blocks_queued())
          {
            //waitway = 4;

            #if ENABLED(NEPTUNE_3_PLUS)
              queue.enqueue_now_P(PSTR("G1 F600 Z3")); 
              queue.enqueue_now_P(PSTR("G1 X292.5 Y165 F8000"));
              queue.enqueue_now_P(PSTR("G1 F200 Z0"));
            #elif ENABLED(NEPTUNE_3_PRO)
              queue.enqueue_now_P(PSTR("G1 F600 Z3")); 
              queue.enqueue_now_P(PSTR("G1 X37.5 Y165 F8000"));
              queue.enqueue_now_P(PSTR("G1 F200 Z0"));
            #elif ENABLED(NEPTUNE_3_MAX)
              queue.enqueue_now_P(PSTR("G1 F600 Z3")); 
              queue.enqueue_now_P(PSTR("G1 X392.5 Y215 F8000"));
              queue.enqueue_now_P(PSTR("G1 F200 Z0"));
            #endif  

            //waitway = 0;
          }
        }
        else if(recdat.data[0] == 0x13) //第7点 
        {
          if (!planner.has_blocks_queued())
          {
            //waitway = 4;

            #if ENABLED(NEPTUNE_3_PLUS)
              queue.enqueue_now_P(PSTR("G1 F600 Z3")); 
              queue.enqueue_now_P(PSTR("G1 X292.5 Y32.5 F8000"));
              queue.enqueue_now_P(PSTR("G1 F200 Z0"));
            #elif ENABLED(NEPTUNE_3_PRO)
              queue.enqueue_now_P(PSTR("G1 F600 Z3")); 
              queue.enqueue_now_P(PSTR("G1 X37.5 Y165 F8000"));
              queue.enqueue_now_P(PSTR("G1 F200 Z0"));
            #elif ENABLED(NEPTUNE_3_MAX)
              queue.enqueue_now_P(PSTR("G1 F600 Z3")); 
              queue.enqueue_now_P(PSTR("G1 X392.5 Y37.5 F8000"));
              queue.enqueue_now_P(PSTR("G1 F200 Z0"));
            #endif  

            //waitway = 0;
          }
        }
        else if(recdat.data[0] == 0x14)
        {
          TERN_(HAS_LEVELING, reset_bed_level());
        }
        else if(recdat.data[0] == 0x15)
        {
          #if ENABLED(EEPROM_SETTINGS)
            settings.save();
          #endif
        }
        else if(recdat.data[0] == 0x16) //printpause恢复页面后获取一次信息
        {
          //机型信息
          #if ENABLED(NEPTUNE_3_PLUS)
            LCD_SERIAL_2.printf("main.va0.val=2");  
            LCD_SERIAL_2.printf("\xff\xff\xff");
          #elif ENABLED(NEPTUNE_3_PRO)
            LCD_SERIAL_2.printf("main.va0.val=1");  
            LCD_SERIAL_2.printf("\xff\xff\xff");                  
          #elif ENABLED(NEPTUNE_3_MAX)
            LCD_SERIAL_2.printf("main.va0.val=3");  
            LCD_SERIAL_2.printf("\xff\xff\xff");
          #endif   

          //打印文件名
          memset(temp,0,sizeof(temp));
          sprintf(temp, "printpause.t0.txt=\"%s\"", CardRecbuf.Cardshowfilename[CardRecbuf.recordcount]);
          LCD_SERIAL_2.printf(temp); 
          LCD_SERIAL_2.printf("\xff\xff\xff");

          //打印百分比
          Percentrecord = card.percentDone();
          if(Percentrecord <= 100)
          {
            #if ENABLED(RTS_AVAILABLE) 
              rtscheck.RTS_SndData((unsigned char)Percentrecord, PRINT_PROCESS_ICON_VP);
              #if ENABLED(TJC_AVAILABLE)   
                memset(temp,0,sizeof(temp));
                sprintf(temp, "printpause.printprocess.val=%d", Percentrecord);
                LCD_SERIAL_2.printf(temp); 
                LCD_SERIAL_2.printf("\xff\xff\xff");

                memset(temp,0,sizeof(temp));
                sprintf(temp, "printpause.printvalue.txt=\"%d\"", Percentrecord);
                LCD_SERIAL_2.printf(temp); 
                LCD_SERIAL_2.printf("\xff\xff\xff");                
              #endif  
            #endif
          }
        }

        RTS_SndData(0, MOTOR_FREE_ICON_VP);
      }
      break;

      case XaxismoveKey:
      {
        waitway = 4;

        #if ENABLED(DUAL_X_CARRIAGE)
        if(active_extruder == 0)
        {
          active_extruder_flag = false;
        }
        else if(active_extruder == 1)
        {
          active_extruder_flag = true;
        }
        #endif

        #if ENABLED(DUAL_X_CARRIAGE)
          if(active_extruder == 1)
          {
              if(recdat.data[0] >= 32768)
              {
                current_position_x1_axis = ((float)recdat.data[0] - 65536) / 10;
              }
              else
              {
                current_position_x1_axis = ((float)recdat.data[0]) / 10;
              }

              delay(2);

              if(current_position_x1_axis > X2_MAX_POS)
              {
                current_position_x1_axis = X2_MAX_POS;
              }
              else if((TEST(axis_known_position, X_AXIS)) && (current_position_x1_axis < (current_position_x0_axis - X_MIN_POS)))
              {
                current_position_x1_axis = current_position_x0_axis - X_MIN_POS;
              }
              else if(current_position_x1_axis < (X_MIN_POS - X_MIN_POS))
              {
                current_position_x1_axis = X_MIN_POS - X_MIN_POS;
              }
              current_position[X_AXIS] = current_position_x1_axis;
          }
          else if(active_extruder == 0)
          {
            if(recdat.data[0] >= 32768)
            {
              current_position_x0_axis = ((float)recdat.data[0] - 65536) / 10;
            }
            else
            {
              current_position_x0_axis = ((float)recdat.data[0]) / 10;
            }

            delay(2);

            if(current_position_x0_axis < X_MIN_POS)
            {
              current_position_x0_axis = X_MIN_POS;
            }

            #if ENABLED(DUAL_X_CARRIAGE)
              else if((TEST(axis_known_position, X_AXIS)) && (current_position_x0_axis > (current_position_x1_axis + X_MIN_POS)))
              {
                current_position_x0_axis = current_position_x1_axis + X_MIN_POS;
              }
              else if(current_position_x0_axis > (X2_MAX_POS + X_MIN_POS))
              {
                current_position_x0_axis = X2_MAX_POS + X_MIN_POS;
              }
              current_position[X_AXIS] = current_position_x0_axis;
            #endif
          }
        #endif

        float x_min, x_max;
        waitway = 4;
        x_min = X_MIN_POS;
        x_max = X_MAX_POS;
        if(recdat.data[0] == 1)
        {
          current_position[X_AXIS] = (current_position[X_AXIS] + 1*axis_unit);
        }
        else
        {
          current_position[X_AXIS] = (current_position[X_AXIS] - 1*axis_unit);
        }
        if (current_position[X_AXIS] < x_min)
        {
          current_position[X_AXIS] = x_min;
        }
        else if (current_position[X_AXIS] > x_max)
        {
          current_position[X_AXIS] = x_max;
        }

        RTS_line_to_current(X_AXIS);
        RTS_SndData(10 * current_position[X_AXIS], AXIS_X_COORD_VP);
        RTS_SndData(0, MOTOR_FREE_ICON_VP);
        waitway = 0;
      }
      break;

      case YaxismoveKey:
      {
        float y_min, y_max;
        waitway = 4;
        y_min = Y_MIN_POS;
        y_max = Y_MAX_POS;

        if(recdat.data[0] == 1)
        {
          current_position[Y_AXIS] = (current_position[Y_AXIS] + 1*axis_unit);
        }
        else
        {
          current_position[Y_AXIS] = (current_position[Y_AXIS] - 1*axis_unit);
        }

        if (current_position[Y_AXIS] < y_min)
        {
          current_position[Y_AXIS] = y_min;
        }
        else if (current_position[Y_AXIS] > y_max)
        {
          current_position[Y_AXIS] = y_max;
        }
        RTS_line_to_current(Y_AXIS);
        RTS_SndData(10 * current_position[Y_AXIS], AXIS_Y_COORD_VP);
        RTS_SndData(0, MOTOR_FREE_ICON_VP);
        waitway = 0;
      }
      break;

      case ZaxismoveKey:
      {
        float z_min, z_max;
        waitway = 4;
        z_min = Z_MIN_POS;
        z_max = Z_MAX_POS;

        if(recdat.data[0] == 1)
        {
          current_position[Z_AXIS] = (current_position[Z_AXIS] + 1*axis_unit);
        }
        else
        {
          current_position[Z_AXIS] = (current_position[Z_AXIS] - 1*axis_unit);
        }

        if (current_position[Z_AXIS] < z_min)
        {
          current_position[Z_AXIS] = z_min;
        }
        else if (current_position[Z_AXIS] > z_max)
        {
          current_position[Z_AXIS] = z_max;
        }
        RTS_line_to_current(Z_AXIS);
        RTS_SndData(10 * current_position[Z_AXIS], AXIS_Z_COORD_VP);
        RTS_SndData(0, MOTOR_FREE_ICON_VP);
        waitway = 0;
      }
      break;

      case SelectExtruderKey:
      {
        #if ENABLED(DUAL_X_CARRIAGE)
          if(recdat.data[0] == 1)
          {
            if(!planner.has_blocks_queued())
            {
              if(active_extruder == 0)
              {
                current_position_x0_axis = current_position[X_AXIS];
                queue.enqueue_now_P(PSTR("T1"));
                //active_extruder = 1;
                active_extruder_flag = true;
                active_extruder_font = active_extruder;

                memset(commandbuf, 0, sizeof(commandbuf));
                sprintf_P(commandbuf, PSTR("G92.9 X%6.3f"), current_position_x1_axis);
                queue.enqueue_one_now(commandbuf);

                if(active_extruder == 0)
                {
                  RTS_SndData(10 * current_position_x0_axis, AXIS_X_COORD_VP);
                }
                else if(active_extruder == 1)
                {
                  RTS_SndData(10 * current_position_x1_axis, AXIS_X_COORD_VP);
                }
                RTS_SndData(1, EXCHANGE_NOZZLE_ICON_VP);
              }
              else if(active_extruder == 1)
              {
                #if ENABLED(DUAL_X_CARRIAGE)
                  current_position_x1_axis = current_position[X_AXIS];
                  queue.enqueue_now_P(PSTR("T0"));
                  //active_extruder = 0;
                  active_extruder_flag = false;
                  active_extruder_font = active_extruder;

                  memset(commandbuf, 0, sizeof(commandbuf));
                  sprintf_P(commandbuf, PSTR("G92.9 X%6.3f"), current_position_x0_axis);
                  queue.enqueue_one_now(commandbuf);

                  if(active_extruder == 0)
                  {
                    RTS_SndData(10 * current_position_x0_axis, AXIS_X_COORD_VP);
                  }
                  else if(active_extruder == 1)
                  {
                    RTS_SndData(10 * current_position_x1_axis, AXIS_X_COORD_VP);
                  }
                  RTS_SndData(0, EXCHANGE_NOZZLE_ICON_VP);
                #endif
              }
            }
          }
          else if (recdat.data[0] == 2)
          {
            if(!planner.has_blocks_queued())
            {
              waitway = 4;
              if(active_extruder == 0)
              {
                queue.enqueue_now_P(PSTR("G28"));

                queue.enqueue_now_P(PSTR("T1"));
                active_extruder_flag = true;
                //active_extruder = 1;
                active_extruder_font = active_extruder;
                RTS_SndData(1, EXCHANGE_NOZZLE_ICON_VP);
                RTS_SndData(10 * current_position[X_AXIS], AXIS_X_COORD_VP);
              }
              else if(active_extruder == 1)
              {
                queue.enqueue_now_P(PSTR("T0"));

                queue.enqueue_now_P(PSTR("G28"));
                active_extruder_flag = false;
                //active_extruder = 1;
                active_extruder_font = active_extruder;
                RTS_SndData(0, EXCHANGE_NOZZLE_ICON_VP);
                RTS_SndData(10 * current_position[X_AXIS], AXIS_X_COORD_VP);
              }
              RTS_SndData(0, MOTOR_FREE_ICON_VP);
              waitway = 0;
            }
          }
        #endif
      }
      break;

      case FilamentLoadKey:
      {
        if(recdat.data[0] == 1)
        {
          if(printJobOngoing())
          {
            RTS_SndData(ExchangePageBase + 23, ExchangepageAddr);
            #if ENABLED(TJC_AVAILABLE) 
              LCD_SERIAL_2.printf("page warn1_filament");
              LCD_SERIAL_2.printf("\xff\xff\xff");               
            #endif
          }
          else
          {
            if(!planner.has_blocks_queued())
            {
              // if(enable_filment_check)
              // {
              //   #if ENABLED(CHECKFILEMENT)
              //     if(0 == READ(CHECKFILEMENT0_PIN))
              //     {
              //       RTS_SndData(ExchangePageBase + 20, ExchangepageAddr);
              //       #if ENABLED(TJC_AVAILABLE)
              //         LCD_SERIAL_2.printf("page nofilament");
              //         LCD_SERIAL_2.printf("\xff\xff\xff");
              //       #endif                     
              //     }
              //   #endif
              // }

              current_position[E_AXIS] -= Filament0LOAD;
              //active_extruder = 0;
              queue.enqueue_now_P(PSTR("T0"));

              if(thermalManager.temp_hotend[0].celsius < (ChangeFilament0Temp - 5))
              {
                RTS_SndData((int)ChangeFilament0Temp, CHANGE_FILAMENT0_TEMP_VP);
                RTS_SndData(ExchangePageBase + 24, ExchangepageAddr);
                #if ENABLED(TJC_AVAILABLE)
                  LCD_SERIAL_2.printf("page warn2_filament");
                  LCD_SERIAL_2.printf("\xff\xff\xff");
                #endif  
              }
              else
              {
                RTS_line_to_current(E_AXIS);
                RTS_SndData(10 * Filament0LOAD, HEAD0_FILAMENT_LOAD_DATA_VP);
                //planner.synchronize();
              }
            }
          }
        }
        else if(recdat.data[0] == 2)
        {
          if(printJobOngoing())
          {
            RTS_SndData(ExchangePageBase + 23, ExchangepageAddr);
            #if ENABLED(TJC_AVAILABLE) 
              LCD_SERIAL_2.printf("page warn1_filament");
              LCD_SERIAL_2.printf("\xff\xff\xff");               
            #endif
          }
          else
          {
            if(!planner.has_blocks_queued())
            {
              // if(enable_filment_check)
              // {
              //   #if ENABLED(CHECKFILEMENT)
              //     if(0 == READ(CHECKFILEMENT0_PIN))
              //     {
              //       RTS_SndData(ExchangePageBase + 20, ExchangepageAddr);
              //       #if ENABLED(TJC_AVAILABLE)
              //         LCD_SERIAL_2.printf("page nofilament");
              //         LCD_SERIAL_2.printf("\xff\xff\xff");
              //       #endif      
              //     }
              //   #endif
              // }
              current_position[E_AXIS] += Filament0LOAD;
              //active_extruder = 0;
              queue.enqueue_now_P(PSTR("T0"));

              if(thermalManager.temp_hotend[0].celsius < (ChangeFilament0Temp - 5))
              {
                RTS_SndData((int)ChangeFilament0Temp, CHANGE_FILAMENT0_TEMP_VP);
                RTS_SndData(ExchangePageBase + 24, ExchangepageAddr);
                #if ENABLED(TJC_AVAILABLE)
                  LCD_SERIAL_2.printf("page warn2_filament");
                  LCD_SERIAL_2.printf("\xff\xff\xff");
                #endif 
              }
              else
              {
                RTS_line_to_current(E_AXIS);
                RTS_SndData(10 * Filament0LOAD, HEAD0_FILAMENT_LOAD_DATA_VP);
                //planner.synchronize();
              }
            }
          }
        }
        else if(recdat.data[0] == 3)
        {
          if(!planner.has_blocks_queued())
          {
            // if(enable_filment_check)
            // {
            //   #if ENABLED(CHECKFILEMENT)
            //     #if ENABLED(DUAL_X_CARRIAGE)
            //       if(0 == READ(CHECKFILEMENT1_PIN))
            //       {
            //         RTS_SndData(ExchangePageBase + 20, ExchangepageAddr);
            //         #if ENABLED(TJC_AVAILABLE)
            //           LCD_SERIAL_2.printf("page nofilament");
            //           LCD_SERIAL_2.printf("\xff\xff\xff");
            //         #endif      
            //       }
            //     #endif
            //   #endif
            // }

            #if ENABLED(DUAL_X_CARRIAGE)
              current_position[E_AXIS] -= Filament1LOAD;
              active_extruder = 1;
              queue.enqueue_now_P(PSTR("T1"));

              if (thermalManager.temp_hotend[1].celsius < (ChangeFilament1Temp - 5))
              {
                RTS_SndData((int)ChangeFilament1Temp, CHANGE_FILAMENT1_TEMP_VP);
                RTS_SndData(ExchangePageBase + 25, ExchangepageAddr);
              }
              else
              {
                RTS_line_to_current(E_AXIS);
                RTS_SndData(10 * Filament1LOAD, HEAD1_FILAMENT_LOAD_DATA_VP);
                planner.synchronize();
              }
            #endif
          }
        }
        else if(recdat.data[0] == 4)
        {
          if(!planner.has_blocks_queued())
          {
            // if(enable_filment_check)
            // {
            //   #if ENABLED(CHECKFILEMENT)
            //     #if ENABLED(DUAL_X_CARRIAGE)
            //       if(0 == READ(CHECKFILEMENT1_PIN))
            //       {
            //         RTS_SndData(ExchangePageBase + 20, ExchangepageAddr);
            //         #if ENABLED(TJC_AVAILABLE)
            //           LCD_SERIAL_2.printf("page nofilament");
            //           LCD_SERIAL_2.printf("\xff\xff\xff");
            //         #endif      
            //       }
            //     #endif
            //   #endif
            // }

            #if ENABLED(DUAL_X_CARRIAGE)
              current_position[E_AXIS] += Filament1LOAD;
              active_extruder = 1;
              queue.enqueue_now_P(PSTR("T1"));

              if(thermalManager.temp_hotend[1].celsius < (ChangeFilament1Temp - 5))
              {
                RTS_SndData((int)ChangeFilament1Temp, CHANGE_FILAMENT1_TEMP_VP);
                RTS_SndData(ExchangePageBase + 25, ExchangepageAddr);
              }
              else
              {
                RTS_line_to_current(E_AXIS);
                RTS_SndData(10 * Filament1LOAD, HEAD1_FILAMENT_LOAD_DATA_VP);
                planner.synchronize();
              }
            #endif
          }
        }
        else if(recdat.data[0] == 5)
        {
          if(!planner.has_blocks_queued())
          {
            RTS_SndData(thermalManager.temp_hotend[0].celsius, HEAD0_CURRENT_TEMP_VP);
            thermalManager.setTargetHotend(ChangeFilament0Temp, 0);
            RTS_SndData(ChangeFilament0Temp, HEAD0_SET_TEMP_VP);
            RTS_SndData(ExchangePageBase + 26, ExchangepageAddr);
            #if ENABLED(TJC_AVAILABLE)
              LCD_SERIAL_2.printf("page heatfilament");
              LCD_SERIAL_2.printf("\xff\xff\xff");
            #endif 
            heatway = 1;
          }
        }
        else if(recdat.data[0] == 6)
        {
          if(!planner.has_blocks_queued())
          {
            Filament0LOAD = 10;
            Filament1LOAD = 10;
            RTS_SndData(10 * Filament0LOAD, HEAD0_FILAMENT_LOAD_DATA_VP);
            RTS_SndData(10 * Filament1LOAD, HEAD1_FILAMENT_LOAD_DATA_VP);
            //RTS_SndData(ExchangePageBase + 23, ExchangepageAddr);
            RTS_SndData(ExchangePageBase + 31, ExchangepageAddr);
            #if ENABLED(TJC_AVAILABLE)
              LCD_SERIAL_2.printf("page prefilament");
              LCD_SERIAL_2.printf("\xff\xff\xff");
            #endif 
            heatway = 1;
          }
        }
        else if(recdat.data[0] == 7)
        {
          if(!planner.has_blocks_queued())
          {
            #if ENABLED(DUAL_X_CARRIAGE)
              RTS_SndData(thermalManager.temp_hotend[1].celsius, HEAD1_CURRENT_TEMP_VP);
            #endif
            thermalManager.setTargetHotend(ChangeFilament1Temp, 1);
            RTS_SndData(ChangeFilament1Temp, HEAD1_SET_TEMP_VP);
            RTS_SndData(ExchangePageBase + 26, ExchangepageAddr);
            #if ENABLED(TJC_AVAILABLE)
              LCD_SERIAL_2.printf("page heatfilament");
              LCD_SERIAL_2.printf("\xff\xff\xff");
            #endif 
            heatway = 2;
          }
        }
        else if (recdat.data[0] == 8)
        {
          if(!planner.has_blocks_queued())
          {
            Filament0LOAD = 10;
            Filament1LOAD = 10;
            RTS_SndData(10 * Filament0LOAD, HEAD0_FILAMENT_LOAD_DATA_VP);
            RTS_SndData(10 * Filament1LOAD, HEAD1_FILAMENT_LOAD_DATA_VP);
            //RTS_SndData(ExchangePageBase + 23, ExchangepageAddr);
            RTS_SndData(ExchangePageBase + 31, ExchangepageAddr);
          }
        }
        else if(recdat.data[0] == 9)
        {
            RTS_SndData(ExchangePageBase + 40, ExchangepageAddr);
            
            #if ENABLED(TJC_AVAILABLE) 
              LCD_SERIAL_2.printf("page wait");
              LCD_SERIAL_2.printf("\xff\xff\xff");               
            #endif

            //reject to receive cmd
            waitway = 1;
            pause_z = current_position[Z_AXIS];
            pause_e = current_position[E_AXIS] - 3;

            card.pauseSDPrint();
            print_job_timer.pause();

            pause_action_flag = true;
            Update_Time_Value = 0;
            planner.synchronize();
            sdcard_pause_check = false;          
        }
        else if(recdat.data[0] == 0x0A)
        {
          if(!planner.has_blocks_queued())
          {
            #if ENABLED(TJC_AVAILABLE) 
              LCD_SERIAL_2.printf("page main");
              LCD_SERIAL_2.printf("\xff\xff\xff");           
            #endif
          }
        }
        else if(recdat.data[0] == 0x0B)
        {
          #if ENABLED(TJC_AVAILABLE)
            memset(temp,0,sizeof(temp));
            sprintf(temp, "motorsetvalue.motorvalue.val=%d",(int)(planner.settings.axis_steps_per_mm[E_AXIS_N(0)]));
            LCD_SERIAL_2.printf(temp);
            LCD_SERIAL_2.printf("\xff\xff\xff");
            unit = 10;   //默认调整单位            
          #endif         
        }
        else if(recdat.data[0] == 0x0C)
        {
          #if ENABLED(TJC_AVAILABLE)
            planner.settings.axis_steps_per_mm[E_AXIS_N(0)] = (planner.settings.axis_steps_per_mm[E_AXIS_N(0)] + unit);
            memset(temp,0,sizeof(temp));
            sprintf(temp, "motorsetvalue.motorvalue.val=%d",(int)(planner.settings.axis_steps_per_mm[E_AXIS_N(0)]));
            LCD_SERIAL_2.printf(temp);
            LCD_SERIAL_2.printf("\xff\xff\xff");               
          #endif  
        }
        else if(recdat.data[0] == 0x0D)
        {
          #if ENABLED(TJC_AVAILABLE)
            planner.settings.axis_steps_per_mm[E_AXIS_N(0)] = (planner.settings.axis_steps_per_mm[E_AXIS_N(0)] - unit);
            if( planner.settings.axis_steps_per_mm[E_AXIS_N(0)]<=0)
            {
              planner.settings.axis_steps_per_mm[E_AXIS_N(0)] = 0;
            }
            memset(temp,0,sizeof(temp));
            sprintf(temp, "motorsetvalue.motorvalue.val=%d",(int)(planner.settings.axis_steps_per_mm[E_AXIS_N(0)]));
            LCD_SERIAL_2.printf(temp);
            LCD_SERIAL_2.printf("\xff\xff\xff");               
          #endif  
        }
        else if(recdat.data[0] == 0x0E)
        {
          current_position[E_AXIS] -= Filament0LOAD;
          RTS_line_to_current(E_AXIS);
        }
        else if(recdat.data[0] == 0x0F)
        {
          current_position[E_AXIS] += Filament0LOAD;
          RTS_line_to_current(E_AXIS);
        }
        else if(recdat.data[0] == 0x10)
        {
          if(printingIsPaused())
          {
            //quickstop_stepper();
            planner.quick_stop();
            planner.synchronize();
          }
        }        
        else if(recdat.data[0] == 0xF1)
        {
          if(!planner.has_blocks_queued())
          {
            thermalManager.temp_hotend[0].target = 0;
            RTS_SndData(thermalManager.temp_hotend[0].target, HEAD0_SET_TEMP_VP);
          #if ENABLED(DUAL_X_CARRIAGE)
            thermalManager.temp_hotend[1].target = 0;
            RTS_SndData(thermalManager.temp_hotend[1].target, HEAD1_SET_TEMP_VP);
          #endif
            //RTS_SndData(ExchangePageBase + 23, ExchangepageAddr);
            RTS_SndData(ExchangePageBase + 31, ExchangepageAddr);
            Filament0LOAD = 10;
            Filament1LOAD = 10;
            RTS_SndData(10 * Filament0LOAD, HEAD0_FILAMENT_LOAD_DATA_VP);
            RTS_SndData(10 * Filament1LOAD, HEAD1_FILAMENT_LOAD_DATA_VP);
            break;
          }
        }
        else if(recdat.data[0] == 0xF0)
        {
          break;
        }
      }
      break;

      case FilamentCheckKey:
      {
        if (recdat.data[0] == 1)
        {
          if(enable_filment_check)
          {
            #if ENABLED(CHECKFILEMENT)
              #if ENABLED(DUAL_X_CARRIAGE)
                if((0 == READ(CHECKFILEMENT0_PIN)) && (active_extruder == 0))
                {
                  RTS_SndData(ExchangePageBase + 20, ExchangepageAddr);
                  #if ENABLED(TJC_AVAILABLE)
                      LCD_SERIAL_2.printf("page nofilament");
                      LCD_SERIAL_2.printf("\xff\xff\xff");
                  #endif      
                }
                else if((0 == READ(CHECKFILEMENT1_PIN)) && (active_extruder == 1))
                {
                  RTS_SndData(ExchangePageBase + 20, ExchangepageAddr);
                  #if ENABLED(TJC_AVAILABLE)
                      LCD_SERIAL_2.printf("page nofilament");
                      LCD_SERIAL_2.printf("\xff\xff\xff");
                  #endif 
                }
              #else
                if(0 == READ(CHECKFILEMENT0_PIN))
                {
                  RTS_SndData(ExchangePageBase + 20, ExchangepageAddr);
                  #if ENABLED(TJC_AVAILABLE)
                      LCD_SERIAL_2.printf("page nofilament");
                      LCD_SERIAL_2.printf("\xff\xff\xff");
                  #endif 
                }
              #endif
                else
                {
                  //RTS_SndData(ExchangePageBase + 23, ExchangepageAddr);
                  RTS_SndData(ExchangePageBase + 31, ExchangepageAddr);
                }
            #endif
          }
        }
        else if (recdat.data[0] == 2)
        {
          RTS_SndData(ExchangePageBase + 21, ExchangepageAddr);
          Filament0LOAD = 10;
          Filament1LOAD = 10;
        }
      }
      break;

      case PowerContinuePrintKey:
      {
        if (recdat.data[0] == 1)
        {
          #if ENABLED(DUAL_X_CARRIAGE)
            save_dual_x_carriage_mode = dualXPrintingModeStatus;
            switch(save_dual_x_carriage_mode)
            {
              case 1:
                queue.enqueue_now_P(PSTR("M605 S1"));
                break;
              case 2:
                queue.enqueue_now_P(PSTR("M605 S2"));
                break;
              case 3:
                queue.enqueue_now_P(PSTR("M605 S2 X68 R0"));
                queue.enqueue_now_P(PSTR("M605 S3"));
                break;
              default:
                queue.enqueue_now_P(PSTR("M605 S0"));
                break;
            }
          #endif

          #if ENABLED(POWER_LOSS_RECOVERY)
            //if (recovery.info.recovery_flag)
            {
              power_off_type_yes = 1;
              Update_Time_Value  = 0;
              
              RTS_SndData(ExchangePageBase + 10, ExchangepageAddr);
              #if ENABLED(TJC_AVAILABLE)
                LCD_SERIAL_2.printf("restFlag2=0");
                LCD_SERIAL_2.printf("\xff\xff\xff");
                
                LCD_SERIAL_2.printf("page printpause");
                LCD_SERIAL_2.printf("\xff\xff\xff");

                pause_count_pos = 0;
              #endif

              PoweroffContinue = true;
              sdcard_pause_check = true;
              zprobe_zoffset = probe.offset.z;
              
              recovery.resume();

              //PoweroffContinue   = true;
              //sdcard_pause_check = true;
              //zprobe_zoffset = probe.offset.z;

              RTS_SndData(zprobe_zoffset * 100, AUTO_BED_LEVEL_ZOFFSET_VP);
              #if ENABLED(TJC_AVAILABLE) 
                memset(temp,0,sizeof(temp));
                sprintf(temp, "leveldata.z_offset.val=%d",(int)(zprobe_zoffset * 100));
                LCD_SERIAL_2.printf(temp);
                LCD_SERIAL_2.printf("\xff\xff\xff");
              #endif
              RTS_SndData(feedrate_percentage, PRINT_SPEED_RATE_VP);
            }
          #endif
        }
        else if (recdat.data[0] == 2)
        {
          Update_Time_Value = RTS_UPDATE_VALUE;
          #if ENABLED(DUAL_X_CARRIAGE)
            extruder_duplication_enabled = false;
            dual_x_carriage_mode = DEFAULT_DUAL_X_CARRIAGE_MODE;
            active_extruder = 0;
          #endif
          RTS_SndData(ExchangePageBase + 1, ExchangepageAddr);
          #if ENABLED(TJC_AVAILABLE) 
            LCD_SERIAL_2.printf("page main");
            LCD_SERIAL_2.printf("\xff\xff\xff");             
          #endif
          RTS_SndData(0, PRINT_TIME_HOUR_VP);
          RTS_SndData(0, PRINT_TIME_MIN_VP);
          RTS_SndData(0, PRINT_SURPLUS_TIME_HOUR_VP);
          RTS_SndData(0, PRINT_SURPLUS_TIME_MIN_VP);
          Update_Time_Value = 0;
          RTS_SDcard_Stop();
        }
        else if(recdat.data[0] == 3)
        {
          #if ENABLED(POWER_LOSS_RECOVERY)
            #if ENABLED(TJC_AVAILABLE)
              #if ENABLED(POWER_LOSS_RECOVERY)
                if(recovery.enabled==0)
                { 
                  recovery.enable(true);
                  LCD_SERIAL_2.printf("multiset.plrbutton.val=1");
                  LCD_SERIAL_2.printf("\xff\xff\xff"); 
                }
                else if(recovery.enabled==1)
                {
                  recovery.enable(false);
                  LCD_SERIAL_2.printf("multiset.plrbutton.val=0");
                  LCD_SERIAL_2.printf("\xff\xff\xff"); 
                }
              #endif              
            #endif
          #endif
        }
      }
      break;

      case SelectFileKey:
      {
        if (RTS_SD_Detected())
        {
          if (recdat.data[0] > CardRecbuf.Filesum)
          {
            break;
          }

          //文件序号
          CardRecbuf.recordcount = recdat.data[0] - 1;

          //清选中文件名区域
          for (int j = 0; j < 10; j ++)
          {
            RTS_SndData(0, SELECT_FILE_TEXT_VP + j);
          }

          //清标题
          #if ENABLED(TJC_AVAILABLE) 
            LCD_SERIAL_2.printf("askprint.t0.txt=\"\"");
            LCD_SERIAL_2.printf("\xff\xff\xff");

            LCD_SERIAL_2.printf("printpause.t0.txt=\"\"");
            LCD_SERIAL_2.printf("\xff\xff\xff"); 
          #endif          

          RTS_SndData(CardRecbuf.Cardshowfilename[CardRecbuf.recordcount], SELECT_FILE_TEXT_VP);

          #if ENABLED(TJC_AVAILABLE)
            memset(temp,0,sizeof(temp));
            sprintf(temp, "askprint.t0.txt=\"%s\"", CardRecbuf.Cardshowfilename[CardRecbuf.recordcount]);
            LCD_SERIAL_2.printf(temp); 
            LCD_SERIAL_2.printf("\xff\xff\xff");

            memset(temp,0,sizeof(temp));
            sprintf(temp, "printpause.t0.txt=\"%s\"", CardRecbuf.Cardshowfilename[CardRecbuf.recordcount]);
            LCD_SERIAL_2.printf(temp); 
            LCD_SERIAL_2.printf("\xff\xff\xff");  
          #endif  

          delay(2);
          for(int j = 1;j <= CardRecbuf.Filesum;j ++)
          {
            RTS_SndData((unsigned long)0xA514, FilenameNature + j * 16);
          }
          RTS_SndData((unsigned long)0x073F, FilenameNature + recdat.data[0] * 16);
          RTS_SndData(1, FILE1_SELECT_ICON_VP + (recdat.data[0] - 1));
          RTS_SndData(ExchangePageBase + 28, ExchangepageAddr);

          #if ENABLED(TJC_AVAILABLE)
            LCD_SERIAL_2.printf("page askprint"); 
            LCD_SERIAL_2.printf("\xff\xff\xff");  
          #endif
        }
      }
      break;

      case PrintFileKey:
      {
        if((recdat.data[0] == 1) && RTS_SD_Detected())
        {

          if(thermalManager.wholeDegHotend(0) < 0)
          {
            #if ENABLED(TJC_AVAILABLE)
              LCD_SERIAL_2.printf("page err_nozzleunde");
              LCD_SERIAL_2.printf("\xff\xff\xff");
            #endif
            break;
          }
          else if(thermalManager.wholeDegBed() < 0)
          {
            #if ENABLED(TJC_AVAILABLE)
              LCD_SERIAL_2.printf("page err_bedunder");
              LCD_SERIAL_2.printf("\xff\xff\xff");
            #endif
            break;
          }  

          if(CardRecbuf.recordcount < 0)
          {
            break;
          }

          //打印文件颜色显示
          #if ENABLED(TJC_AVAILABLE) 
            memset(temp,0,sizeof(temp));
            sprintf(temp, "file%d.t%d.pco=65504", (CardRecbuf.recordcount/5) + 1 , CardRecbuf.recordcount);
            LCD_SERIAL_2.printf(temp);
            LCD_SERIAL_2.printf("\xff\xff\xff");           
          #endif

          char cmd[30];
          char *c;
          sprintf_P(cmd, PSTR("M23 %s"), CardRecbuf.Cardfilename[CardRecbuf.recordcount]);
          for (c = &cmd[4]; *c; c++)
          {
            *c = tolower(*c);
          }

          memset(cmdbuf, 0, sizeof(cmdbuf));
          strcpy(cmdbuf, cmd);
          FilenamesCount = CardRecbuf.recordcount;

          save_dual_x_carriage_mode = dualXPrintingModeStatus;
          switch(save_dual_x_carriage_mode)
          {
            case 1:
              queue.enqueue_now_P(PSTR("M605 S1"));
              break;
            case 2:
              queue.enqueue_now_P(PSTR("M605 S2"));
              break;
            case 3:
              queue.enqueue_now_P(PSTR("M605 S2 X68 R0"));
              queue.enqueue_now_P(PSTR("M605 S3"));
              break;
            default:
              queue.enqueue_now_P(PSTR("M605 S0"));
              queue.enqueue_now_P(PSTR("T0"));
              break;
          }

          if(enable_filment_check)
          {
            #if ENABLED(CHECKFILEMENT)
              #if ENABLED(DUAL_X_CARRIAGE)
                if((0 == save_dual_x_carriage_mode) && (0 == READ(CHECKFILEMENT0_PIN)) && (active_extruder == 0))
                {
                  RTS_SndData(ExchangePageBase + 39, ExchangepageAddr);
                  sdcard_pause_check = false;
                  break;
                }
                else if((0 == save_dual_x_carriage_mode) && (0 == READ(CHECKFILEMENT1_PIN)) && (active_extruder == 1))
                {
                  RTS_SndData(ExchangePageBase + 39, ExchangepageAddr);
                  sdcard_pause_check = false;
                  break;
                }
                else if((0 != save_dual_x_carriage_mode) && ((0 == READ(CHECKFILEMENT0_PIN)) || (0 == READ(CHECKFILEMENT1_PIN))))
                {
                  RTS_SndData(ExchangePageBase + 39, ExchangepageAddr);
                  sdcard_pause_check = false;
                  break;
                }
              #else
                {
                  if(0 == READ(CHECKFILEMENT0_PIN))
                  {
                    RTS_SndData(ExchangePageBase + 39, ExchangepageAddr);
                    #if ENABLED(TJC_AVAILABLE)
                      LCD_SERIAL_2.printf("page nofilament"); 
                      LCD_SERIAL_2.printf("\xff\xff\xff");  
                    #endif                    
                    sdcard_pause_check = false;
                    break;
                  }                  
                }
              #endif
            #endif
          }

          //清屏
          for (int j = 0; j < 20; j ++)
          {
            RTS_SndData(0, PRINT_FILE_TEXT_VP + j);
          }

          RTS_SndData(CardRecbuf.Cardshowfilename[CardRecbuf.recordcount], PRINT_FILE_TEXT_VP);

          delay(2);

          #if ENABLED(BABYSTEPPING)
            RTS_SndData(0, AUTO_BED_LEVEL_ZOFFSET_VP);
            #if ENABLED(TJC_AVAILABLE) 
              memset(temp,0,sizeof(temp));
              sprintf(temp, "leveldata.z_offset.val=%d", 0);
              LCD_SERIAL_2.printf(temp);
              LCD_SERIAL_2.printf("\xff\xff\xff");           
            #endif
          #endif

          feedrate_percentage = 100;
          RTS_SndData(feedrate_percentage, PRINT_SPEED_RATE_VP);
          zprobe_zoffset = last_zoffset;
          RTS_SndData(zprobe_zoffset * 100, AUTO_BED_LEVEL_ZOFFSET_VP);

          #if ENABLED(TJC_AVAILABLE)
            LCD_SERIAL_2.printf("printpause.printvalue.txt=\"0\""); 
            LCD_SERIAL_2.printf("\xff\xff\xff");

            LCD_SERIAL_2.printf("printpause.printprocess.val=0"); 
            LCD_SERIAL_2.printf("\xff\xff\xff");  

            memset(temp,0,sizeof(temp));
            sprintf(temp, "leveldata.z_offset.val=%d", (int)(zprobe_zoffset * 100));
            LCD_SERIAL_2.printf(temp);
            LCD_SERIAL_2.printf("\xff\xff\xff");           
          #endif

          PoweroffContinue = true;

          //切换正在打印页面
          RTS_SndData(ExchangePageBase + 10, ExchangepageAddr);
          #if ENABLED(TJC_AVAILABLE) 
            LCD_SERIAL_2.printf("page printpause");
            LCD_SERIAL_2.printf("\xff\xff\xff");

            LCD_SERIAL_2.printf("restFlag2=0");
            LCD_SERIAL_2.printf("\xff\xff\xff");

            pause_count_pos = 0;
          #endif

          //图片预览
          #if ENABLED(TJC_AVAILABLE)

            LCD_SERIAL_2.printf("printpause.cp0.close()");
            LCD_SERIAL_2.printf("\xff\xff\xff");

            LCD_SERIAL_2.printf("printpause.cp0.aph=0");
            LCD_SERIAL_2.printf("\xff\xff\xff");          

            LCD_SERIAL_2.printf("printpause.va0.txt=\"\"");
            LCD_SERIAL_2.printf("\xff\xff\xff");

            LCD_SERIAL_2.printf("printpause.va1.txt=\"\"");
            LCD_SERIAL_2.printf("\xff\xff\xff");

            char picname[64];
            uint8_t public_buf[512];
            MediaFileReader file;
            sprintf(picname,"%s.txt",CardRecbuf.Cardshowfilename[CardRecbuf.recordcount]);
            bool run = file.open(picname);
            if(run)
            {
              while(1)
              {
                TERN_(USE_WATCHDOG, hal.watchdog_refresh());

                LCD_SERIAL_2.printf("printpause.cp0.aph=0");
                LCD_SERIAL_2.printf("\xff\xff\xff");

                LCD_SERIAL_2.printf("printpause.va0.txt=\"\"");
                LCD_SERIAL_2.printf("\xff\xff\xff");

                memset(public_buf,0,sizeof(public_buf));
                int16_t byte = file.read(public_buf, 512);
                
                LCD_SERIAL_2.printf("printpause.va0.txt=");
                LCD_SERIAL_2.write(0x22);
                LCD_SERIAL_2.write(public_buf,sizeof(public_buf));
                LCD_SERIAL_2.write(0x22);
                LCD_SERIAL_2.printf("\xff\xff\xff");

                LCD_SERIAL_2.printf("printpause.va1.txt+=printpause.va0.txt");
                LCD_SERIAL_2.printf("\xff\xff\xff");
                      
                if(byte<=0) 
                {
                  LCD_SERIAL_2.printf("printpause.cp0.aph=127");
                  LCD_SERIAL_2.printf("\xff\xff\xff");

                  LCD_SERIAL_2.printf("printpause.cp0.write(printpause.va1.txt)");
                  LCD_SERIAL_2.printf("\xff\xff\xff");

                  file.close();
                  break;
                };
              }
            }
            else
            {
              file.close();
              memset(picname,0,sizeof(picname));

              SdFile *diveDir;
              const char * const fname = card.diveToFile(true, diveDir, CardRecbuf.Cardfilename[CardRecbuf.recordcount]);
              bool run = file.open(fname);
              if(run)
              {
                uint32_t gPicturePreviewStart = 0;
                uint64_t cnt_pre = 0;

                uint8_t public_buf[1024];

                while(1)
                {
                  LCD_SERIAL_2.printf("printpause.va0.txt=\"\"");
                  LCD_SERIAL_2.printf("\xff\xff\xff");

                  memset(public_buf,0,sizeof(public_buf));
                  int16_t  byte = file.read(public_buf,sizeof(public_buf));
                  if((unsigned int)(byte)<sizeof(public_buf))  break;
                  cnt_pre = (cnt_pre + byte);
                  //uint32_t *p1  = (uint32_t *)strstr((char *)public_buf, ";gimage:");
                  uint32_t *p1  = (uint32_t *)strstr((char *)public_buf, ";simage:");
                  uint32_t *m1  = (uint32_t *)strstr((char *)public_buf, "M10086");

                  if(m1)
                  {
                    cnt_pre = 0;

                    LCD_SERIAL_2.printf("printpause.cp0.aph=0");
                    LCD_SERIAL_2.printf("\xff\xff\xff");

                    LCD_SERIAL_2.printf("printpause.cp0.close()");
                    LCD_SERIAL_2.printf("\xff\xff\xff");
                    break;
                  }
                  
                  if(p1)
                  { 
                    cnt_pre = 0;

                    while(1)
                    {
                      LCD_SERIAL_2.printf("printpause.va0.txt=\"\"");
                      LCD_SERIAL_2.printf("\xff\xff\xff");

                      memset(public_buf,0,sizeof(public_buf));
                      int16_t  byte = file.read(public_buf,sizeof(public_buf));
                      if(byte<0)  break;
                      cnt_pre = (cnt_pre + byte);

                      //uint32_t *p2  = (uint32_t *)strstr((char *)public_buf, ";;gimage:");
                      uint32_t *p2  = (uint32_t *)strstr((char *)public_buf, ";;simage:");
                      uint32_t *p3  = (uint32_t *)strstr((char *)public_buf, ";00000");
                      
                      //if(p2)
                      if(p2||p3)
                      {
                        file.rewind(); //文件读取指针复位

                        while(1) //";gimage:"起始位置
                        {
                          while(1)
                          {
                            LCD_SERIAL_2.printf("printpause.va0.txt=\"\"");
                            LCD_SERIAL_2.printf("\xff\xff\xff");

                            TERN_(USE_WATCHDOG, hal.watchdog_refresh());
                            memset(public_buf,0,sizeof(public_buf));
                            int16_t byte = file.read(public_buf,1024);
                            if(byte<0)  break;
                            gPicturePreviewStart+=byte;

                            //uint32_t *p1 = (uint32_t *)strstr((char *)public_buf, ";gimage:");
                            //uint32_t *p2 = (uint32_t *)strstr((char *)public_buf, ";;gimage:");
                            uint32_t *p1 = (uint32_t *)strstr((char *)public_buf, ";simage:");
                            uint32_t *p2 = (uint32_t *)strstr((char *)public_buf, ";;simage:");
                            uint32_t *p3 = (uint32_t *)strstr((char *)public_buf, ";00000");
                            uint32_t *p4 = (uint32_t *)strstr((char *)public_buf, ";;gimage:");

                            //if(p2)

                            if( (p1==0) && (p2==0) && (p4==0) && p3)
                            {
                              LCD_SERIAL_2.printf("printpause.cp0.aph=127");
                              LCD_SERIAL_2.printf("\xff\xff\xff");

                              LCD_SERIAL_2.printf("printpause.cp0.write(printpause.va1.txt)");
                              LCD_SERIAL_2.printf("\xff\xff\xff");
                              break;
                            }

                            if(p2)
                            {
                              LCD_SERIAL_2.printf("printpause.va0.txt=");
                              LCD_SERIAL_2.write(0x22);
                              LCD_SERIAL_2.write(&public_buf[9],1023-9);
                              LCD_SERIAL_2.write(0x22);
                              LCD_SERIAL_2.printf("\xff\xff\xff");
                              LCD_SERIAL_2.printf("printpause.va1.txt+=printpause.va0.txt");
                              LCD_SERIAL_2.printf("\xff\xff\xff");                                     

                              LCD_SERIAL_2.printf("printpause.cp0.aph=127");
                              LCD_SERIAL_2.printf("\xff\xff\xff");

                              TERN_(USE_WATCHDOG, hal.watchdog_refresh());
                              delay(200);
                              TERN_(USE_WATCHDOG, hal.watchdog_refresh());

                              LCD_SERIAL_2.printf("printpause.cp0.write(printpause.va1.txt)");
                              LCD_SERIAL_2.printf("\xff\xff\xff");

                              cnt_pre = 102400;

                              break;
                            }

                            if(p1)
                            {
                              LCD_SERIAL_2.printf("printpause.va0.txt=");
                              LCD_SERIAL_2.write(0x22);
                              LCD_SERIAL_2.write(&public_buf[8],1023-8);
                              LCD_SERIAL_2.write(0x22);
                              LCD_SERIAL_2.printf("\xff\xff\xff");
                              LCD_SERIAL_2.printf("printpause.va1.txt+=printpause.va0.txt");
                              LCD_SERIAL_2.printf("\xff\xff\xff");

                              TERN_(USE_WATCHDOG, hal.watchdog_refresh());
                              delay(200);
                              TERN_(USE_WATCHDOG, hal.watchdog_refresh());
                            }
                          }

                          break;
                        }
                      }

                      if(cnt_pre>=102400)  break;
                    }

                  }

                  if(cnt_pre>=102400)  break;

                }

                file.close();
                delay(20);
              }

              file.close();
            }
          #endif

          queue.enqueue_one_now(cmd);        //打开选中文件
          delay(20);
          queue.enqueue_now_P(PSTR("M24"));  //开始打印

          // if (!filelist.seek(CardRecbuf.recordcount)) return;
          // ExtUI::printFile(filelist.shortFilename());
        
          Update_Time_Value = 0;
        }
        else if(recdat.data[0] == 2)
        {
          RTS_SndData(ExchangePageBase + 3, ExchangepageAddr);
        }
        else if(recdat.data[0] == 3)
        {
          RTS_SndData(ExchangePageBase + 2, ExchangepageAddr);
        }
        else if(recdat.data[0] == 4)
        {
          RTS_SndData(ExchangePageBase + 4, ExchangepageAddr);
        }
        else if(recdat.data[0] == 5)
        {
          RTS_SndData(ExchangePageBase + 3, ExchangepageAddr);
        }
        else if(recdat.data[0] == 6)
        {
          RTS_SndData(ExchangePageBase + 5, ExchangepageAddr);
        }
        else if(recdat.data[0] == 7)
        {
          RTS_SndData(ExchangePageBase + 4, ExchangepageAddr);
        }
        else if(recdat.data[0] == 8)
        {
          RTS_SndData(ExchangePageBase + 6, ExchangepageAddr);
        }
        else if(recdat.data[0] == 9)
        {
          RTS_SndData(ExchangePageBase + 5, ExchangepageAddr);
        }
        else if(recdat.data[0] == 10)
        {
          //RTS_SndData(ExchangePageBase + 1, ExchangepageAddr);
        }
        else if(recdat.data[0] == 0x0A)
        {
          LCD_SERIAL_2.printf("page main");
          LCD_SERIAL_2.printf("\xff\xff\xff");       
        }
      }
      break;

      case PrintSelectModeKey:
      {
        #if ENABLED(DUAL_X_CARRIAGE)
          if (recdat.data[0] == 1)
          {
            RTS_SndData(1, TWO_COLOR_MODE_ICON_VP);
            RTS_SndData(0, COPY_MODE_ICON_VP);
            RTS_SndData(0, MIRROR_MODE_ICON_VP);
            RTS_SndData(0, SINGLE_MODE_ICON_VP);
            dualXPrintingModeStatus = 1;
            RTS_SndData(1, PRINT_MODE_ICON_VP);
            RTS_SndData(1, SELECT_MODE_ICON_VP);
          }
          else if (recdat.data[0] == 2)
          {
            RTS_SndData(1, COPY_MODE_ICON_VP);
            RTS_SndData(0, TWO_COLOR_MODE_ICON_VP);
            RTS_SndData(0, MIRROR_MODE_ICON_VP);
            RTS_SndData(0, SINGLE_MODE_ICON_VP);
            dualXPrintingModeStatus = 2;
            RTS_SndData(2, PRINT_MODE_ICON_VP);
            RTS_SndData(2, SELECT_MODE_ICON_VP);
          }
          else if (recdat.data[0] == 3)
          {
            RTS_SndData(1, MIRROR_MODE_ICON_VP);
            RTS_SndData(0, TWO_COLOR_MODE_ICON_VP);
            RTS_SndData(0, COPY_MODE_ICON_VP);
            RTS_SndData(0, SINGLE_MODE_ICON_VP);
            dualXPrintingModeStatus = 3;
            RTS_SndData(3, PRINT_MODE_ICON_VP);
            RTS_SndData(3, SELECT_MODE_ICON_VP);
          }
          else if (recdat.data[0] == 4)
          {
            RTS_SndData(0, MIRROR_MODE_ICON_VP);
            RTS_SndData(0, TWO_COLOR_MODE_ICON_VP);
            RTS_SndData(0, COPY_MODE_ICON_VP);
            RTS_SndData(1, SINGLE_MODE_ICON_VP);
            dualXPrintingModeStatus = 0;
            RTS_SndData(4, PRINT_MODE_ICON_VP);
            RTS_SndData(4, SELECT_MODE_ICON_VP);
          }
          else if (recdat.data[0] == 5)
          {
            #if ENABLED(EEPROM_SETTINGS)
              settings.save();
            #endif
            //srtscheck.RTS_SndData(ExchangePageBase + 1, ExchangepageAddr);
          }
        #endif
      }
      break;

      case StoreMemoryKey:
      {
        if(recdat.data[0] == 0xF1)
        {
          //queue.enqueue_now_P(PSTR("M502"));
          rtscheck.RTS_SndData(ExchangePageBase + 21, ExchangepageAddr);
          #if ENABLED(EEPROM_SETTINGS)
            settings.init_eeprom();
            delay(500);
            settings.load();
          #endif

          rtscheck.RTS_Init();

          //RTS_SndData((hotend_offset[1].x - X2_MAX_POS) * 10, TWO_EXTRUDER_HOTEND_XOFFSET_VP);
          RTS_SndData(hotend_offset[1].y * 10, TWO_EXTRUDER_HOTEND_YOFFSET_VP);
          RTS_SndData(hotend_offset[1].z * 10, TWO_EXTRUDER_HOTEND_ZOFFSET_VP);
        }
        else if (recdat.data[0] == 0xF0)
        {
          memset(commandbuf, 0, sizeof(commandbuf));
          sprintf_P(commandbuf, PSTR("M218 T1 X%4.1f"), hotend_offset[1].x);
          queue.enqueue_now_P(commandbuf);
          sprintf_P(commandbuf, PSTR("M218 T1 Y%4.1f"), hotend_offset[1].y);
          queue.enqueue_now_P(commandbuf);
          sprintf_P(commandbuf, PSTR("M218 T1 Z%4.1f"), hotend_offset[1].z);
          queue.enqueue_now_P(commandbuf);
          #if ENABLED(EEPROM_SETTINGS)
            settings.save();
          #endif
          rtscheck.RTS_SndData(ExchangePageBase + 35, ExchangepageAddr);
        }
      }
      break;

      case XhotendOffsetKey:
      {
        if (recdat.data[0] >= 32768)
        {
          XoffsetValue = (recdat.data[0] - 65536) / 10.0;
          XoffsetValue = XoffsetValue - 0.0001;
        }
        else
        {
          XoffsetValue = (recdat.data[0]) / 10.0;
          XoffsetValue = XoffsetValue + 0.0001;
        }

        RTS_SndData(XoffsetValue * 10, TWO_EXTRUDER_HOTEND_XOFFSET_VP);
      }
      break;

      case YhotendOffsetKey:
      {
        #if ENABLED(DUAL_X_CARRIAGE)
          if (recdat.data[0] >= 32768)
          {
            hotend_offset[1].y = (recdat.data[0] - 65536) / 10.0;
            hotend_offset[1].y = hotend_offset[1].y - 0.0001;
          }
          else
          {
            hotend_offset[1].y = (recdat.data[0]) / 10.0;
            hotend_offset[1].y = hotend_offset[1].y + 0.0001;
          }
          RTS_SndData(hotend_offset[1].y * 10, TWO_EXTRUDER_HOTEND_YOFFSET_VP);
        #endif
      }
      break;

      case ZhotendOffsetKey:
      {
        #if ENABLED(DUAL_X_CARRIAGE)
          if (recdat.data[0] >= 32768)
          {
            hotend_offset[1].z = (recdat.data[0] - 65536) / 10.0;
            hotend_offset[1].z = hotend_offset[1].z - 0.0001;
          }
          else
          {
            hotend_offset[1].z = (recdat.data[0]) / 10.0;
            hotend_offset[1].z = hotend_offset[1].z + 0.0001;
          }
          RTS_SndData(hotend_offset[1].z * 10, TWO_EXTRUDER_HOTEND_ZOFFSET_VP);
        #endif
      }
      break;

      case ChangePageKey:
      {
        if ((change_page_number == 36) || (change_page_number == 76))
        {
          break;
        }

        for (int i = 0; i < MaxFileNumber; i ++)
        {
          for (int j = 0; j < 20; j ++)
          {
            RTS_SndData(0, FILE1_TEXT_VP + i * 20 + j);
          }
        }

        for (int i = 0; i < CardRecbuf.Filesum; i++)
        {
          for (int j = 0; j < 20; j++)
          {
            RTS_SndData(0, CardRecbuf.addr[i] + j);
          }
          RTS_SndData((unsigned long)0xA514, FilenameNature + (i + 1) * 16);
        }

        for (int j = 0; j < 20; j ++)
        {
          // clean screen.
          RTS_SndData(0, PRINT_FILE_TEXT_VP + j);
          // clean filename
          RTS_SndData(0, SELECT_FILE_TEXT_VP + j);
        }
        // clean filename Icon
        for (int j = 0; j < 20; j ++)
        {
          RTS_SndData(10, FILE1_SELECT_ICON_VP + j);
        }

        RTS_SndData(CardRecbuf.Cardshowfilename[CardRecbuf.recordcount], PRINT_FILE_TEXT_VP);

        // represents to update file list
        if (CardUpdate && lcd_sd_status && IS_SD_INSERTED())
        {
          for (uint16_t i = 0; i < CardRecbuf.Filesum; i++)
          {
            delay(3);
            RTS_SndData(CardRecbuf.Cardshowfilename[i], CardRecbuf.addr[i]);
            RTS_SndData((unsigned long)0xA514, FilenameNature + (i + 1) * 16);
            RTS_SndData(0, FILE1_SELECT_ICON_VP + i);
          }
        }

        char sizeBuf[20];
        sprintf(sizeBuf, "%d X %d X %d", X_BED_SIZE, Y_BED_SIZE, Z_MAX_POS);
        RTS_SndData(MACVERSION, PRINTER_MACHINE_TEXT_VP);
        RTS_SndData(SOFTVERSION, PRINTER_VERSION_TEXT_VP);
        RTS_SndData(sizeBuf, PRINTER_PRINTSIZE_TEXT_VP);

        RTS_SndData(CORP_WEBSITE, PRINTER_WEBSITE_TEXT_VP);

        if (thermalManager.fan_speed[0] == 0)
        {
          RTS_SndData(1, HEAD0_FAN_ICON_VP);
        }
        else
        {
          RTS_SndData(0, HEAD0_FAN_ICON_VP);
        }
        #if HAS_FAN1
        if (thermalManager.fan_speed[1] == 0)
        {
          RTS_SndData(1, HEAD1_FAN_ICON_VP);
        }
        else
        {
          RTS_SndData(0, HEAD1_FAN_ICON_VP);
        }
        #endif
        Percentrecord = card.percentDone() + 1;
        if (Percentrecord <= 100)
        {
          rtscheck.RTS_SndData((unsigned char)Percentrecord, PRINT_PROCESS_ICON_VP);
        }
        rtscheck.RTS_SndData((unsigned char)card.percentDone(), PRINT_PROCESS_VP);

        RTS_SndData(zprobe_zoffset * 100, AUTO_BED_LEVEL_ZOFFSET_VP);
        #if ENABLED(TJC_AVAILABLE) 
          memset(temp,0,sizeof(temp));
          sprintf(temp, "leveldata.z_offset.val=%d", (int)(zprobe_zoffset * 100));
          LCD_SERIAL_2.printf(temp);
          LCD_SERIAL_2.printf("\xff\xff\xff");           
        #endif
        RTS_SndData(feedrate_percentage, PRINT_SPEED_RATE_VP);
        #if HAS_HOTEND
          RTS_SndData(thermalManager.temp_hotend[0].target, HEAD0_SET_TEMP_VP);
        #endif
        #if HAS_MULTI_HOTEND
           RTS_SndData(thermalManager.temp_hotend[1].target, HEAD1_SET_TEMP_VP);
        #endif
       
        RTS_SndData(thermalManager.temp_bed.target, BED_SET_TEMP_VP);

        //RTS_SndData(change_page_number + ExchangePageBase, ExchangepageAddr);
      }
      break;

      case HardwareTest:
      {
        if (recdat.data[0] == 0x00) //X限位
        {
          if(READ(X_MIN_PIN))
          {
            LCD_SERIAL_2.printf("x.bco=1024");
            LCD_SERIAL_2.printf("\xff\xff\xff");             
          }
          else
          {
            LCD_SERIAL_2.printf("x.bco=50712");
            LCD_SERIAL_2.printf("\xff\xff\xff");   
          }
        }
        else if (recdat.data[0] == 0x01) //Y限位
        {
          if(READ(Y_MIN_PIN))
          {
            LCD_SERIAL_2.printf("y.bco=1024");
            LCD_SERIAL_2.printf("\xff\xff\xff");             
          }
          else
          {
            LCD_SERIAL_2.printf("y.bco=50712");
            LCD_SERIAL_2.printf("\xff\xff\xff");   
          }
        }
        else if (recdat.data[0] == 0x02) //Z限位
        {
          if(READ(Z_MIN_PROBE_PIN))
          {
            LCD_SERIAL_2.printf("z.bco=1024");
            LCD_SERIAL_2.printf("\xff\xff\xff");             
          }
          else
          {
            LCD_SERIAL_2.printf("z.bco=50712");
            LCD_SERIAL_2.printf("\xff\xff\xff");   
          }
        }
        else if (recdat.data[0] == 0x03) //mtd状态
        {
          if(READ(CHECKFILEMENT0_PIN))
          {
            LCD_SERIAL_2.printf("mtd.bco=1024");
            LCD_SERIAL_2.printf("\xff\xff\xff");             
          }
          else
          {
            LCD_SERIAL_2.printf("mtd.bco=50712");
            LCD_SERIAL_2.printf("\xff\xff\xff");   
          }
        }
        else if (recdat.data[0] == 0x04) //加热喷头
        {
          thermalManager.temp_hotend[0].target = 260;
          thermalManager.setTargetHotend(thermalManager.temp_hotend[0].target, 0);
          LCD_SERIAL_2.printf("nozzle.bco=1024");
          LCD_SERIAL_2.printf("\xff\xff\xff"); 
        }
        else if (recdat.data[0] == 0x05) //加热热床
        {
          thermalManager.temp_bed.target = 100;
          thermalManager.setTargetBed(thermalManager.temp_bed.target);
          LCD_SERIAL_2.printf("bed.bco=1024");
          LCD_SERIAL_2.printf("\xff\xff\xff");    
        }
        else if (recdat.data[0] == 0x06) //开启模型散热风扇
        {
          thermalManager.set_fan_speed(0, 255);
          LCD_SERIAL_2.printf("fan.bco=1024");
          LCD_SERIAL_2.printf("\xff\xff\xff");
          LCD_SERIAL_2.printf("set.va0.val=1");
          LCD_SERIAL_2.printf("\xff\xff\xff"); 
        }  
        else if (recdat.data[0] == 0x07) //开照明灯
        {
          status_led2 = true;
          #if ENABLED(TJC_AVAILABLE) 
            LCD_SERIAL_2.printf("status_led2=1");
            LCD_SERIAL_2.printf("\xff\xff\xff");
          #endif
          LCD_SERIAL_2.printf("led.bco=1024");
          LCD_SERIAL_2.printf("\xff\xff\xff");
          #if PIN_EXISTS(LED3)
            OUT_WRITE(LED3_PIN, HIGH);
          #endif
        }  
        else if (recdat.data[0] == 0x08) //关喷头
        {
          thermalManager.temp_hotend[0].target = 0;
          thermalManager.setTargetHotend(thermalManager.temp_hotend[0].target, 0);
          LCD_SERIAL_2.printf("nozzle.bco=50712");
          LCD_SERIAL_2.printf("\xff\xff\xff"); 
        }
        else if (recdat.data[0] == 0x09) //关热床
        {
          thermalManager.temp_bed.target = 0;
          thermalManager.setTargetBed(thermalManager.temp_bed.target);
          LCD_SERIAL_2.printf("bed.bco=50712");
          LCD_SERIAL_2.printf("\xff\xff\xff");
        }
        else if (recdat.data[0] == 0x0A) //关模型风扇
        {
          thermalManager.set_fan_speed(0, 0);
          LCD_SERIAL_2.printf("fan.bco=50712");
          LCD_SERIAL_2.printf("\xff\xff\xff");
          LCD_SERIAL_2.printf("set.va0.val=0");
          LCD_SERIAL_2.printf("\xff\xff\xff");   
        }
        else if (recdat.data[0] == 0x0B) //关照明灯
        {
          status_led2 = false;
          #if ENABLED(TJC_AVAILABLE) 
            LCD_SERIAL_2.printf("status_led2=0");
            LCD_SERIAL_2.printf("\xff\xff\xff");
          #endif
          LCD_SERIAL_2.printf("led.bco=50712");
          LCD_SERIAL_2.printf("\xff\xff\xff");
          #if PIN_EXISTS(LED3)
            OUT_WRITE(LED3_PIN, LOW);
          #endif
        }
        else if (recdat.data[0] == 0x0C) //正转
        {
          thermalManager.allow_cold_extrude=false;
          thermalManager.extrude_min_temp = 0;

          destination.set(15,15,15,15);
          prepare_internal_move_to_destination(500);
          LCD_SERIAL_2.printf("motor1.bco=1024");
          LCD_SERIAL_2.printf("\xff\xff\xff");
          LCD_SERIAL_2.printf("motor2.bco=50712");
          LCD_SERIAL_2.printf("\xff\xff\xff");
        }
        else if (recdat.data[0] == 0x0D) //反转
        {
          destination.set(10,10,10,10);
          prepare_internal_move_to_destination(500);
          LCD_SERIAL_2.printf("motor1.bco=50712");
          LCD_SERIAL_2.printf("\xff\xff\xff");
          LCD_SERIAL_2.printf("motor2.bco=1024");
          LCD_SERIAL_2.printf("\xff\xff\xff");
        }
        else if (recdat.data[0] == 0x0E) //
        {

        }
        else if (recdat.data[0] == 0x0F) 
        {
          const char *MKSTestPath = "MKS_TEST";
          //const char *TJCUpdate   = "3D30.tft";
          SdFile dir, root = card.getroot();
          if (dir.open(&root, MKSTestPath, O_RDONLY))
          {
            LCD_SERIAL_2.printf("page hardwaretest");
            LCD_SERIAL_2.printf("\xff\xff\xff");
          }
          // else if(dir.open(&root, TJCUpdate, O_RDONLY))
          // {
          //   char tmpfilename[128];
          //   uint16_t hmi_filesize = dir.fileSize();
          //   memset(tmpfilename,0,sizeof(tmpfilename));
          //   sprintf(tmpfilename, "whmi-wri %d,115200,0", hmi_filesize); 
          //   LCD_SERIAL_2.printf(tmpfilename);
          //   LCD_SERIAL_2.printf("\xff\xff\xff");
            
          // }
        }                         
      }
      break;

      case Err_Control:
      {

      }
      break;

      default:
        break;
    }
    memset(&recdat, 0, sizeof(recdat));
    recdat.head[0] = FHONE;
    recdat.head[1] = FHTWO;
  }
#endif

