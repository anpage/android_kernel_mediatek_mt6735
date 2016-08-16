

#ifndef __HI842_SENSOR_FUNCTION_DEFINE
#define __HI842_SENSOR_FUNCTION_DEFINE

/*******************************************************************************
* Fixed Value

  Don't change this block code 
********************************************************************************/
#define HI842_CHECKSUM_ON 0
#define HI842_CHECKSUM_OFF 1
#define HI842_GOLDEN_WB_SW 0        
#define HI842_GOLDEN_WB_OTP 1       

/*******************************************************************************
* User Define 

  Select function. User Select Enable
********************************************************************************/

// * Kernel Log Enable
#define HI842_WB_DBG        // define open : Log On /// close :  Log Off
#define HI842_AF_DBG        // define open : Log On /// close :  Log Off
#define HI842_CHIPINFO_DBG  // define open : Log On /// close :  Log Off
#define HI842_LSC_DBG       // define open : Log On /// close :  Log Off

// * Check Sum 
#define HI842_CHECKSUM_ENABLE HI842_CHECKSUM_ON // Checksum function on / off select 

// * WB Cal 
#define HI842_WB_METHOD HI842_GOLDEN_WB_SW // Select WB Cal Data Load. OTP or Hardcoding data 

// * Set OTP Register
#define WB_FLAG_A 0x835    // white balance. flag address 
#define WB_B1_SA 0x836     // white balance. Bank1 start address 
#define WB_B2_SA 0x854     // white balance. Bank2 start address 
#define WB_B3_SA 0x872     // white balance. Bank3 start address 
#define WB_CS1_SA 0x853    // white balance. Bank1 checksum address 
#define WB_CS2_SA 0x871    // white balance. Bank2 checksum address 
#define WB_CS3_SA 0x88F    // white balance. Bank3 checksum address 

#define AF_FLAG_A 0x890    // Auto Focus. flag address 
#define AF_B1_SA 0x891     // Auto Focus. Bank1 start address 
#define AF_B2_SA 0x89D     // Auto Focus. Bank2 start address 
#define AF_B3_SA 0x8A9     // Auto Focus. Bank3 start address 
#define AF_CS1_SA 0x89C    // Auto Focus. Bank1 checksum address 
#define AF_CS2_SA 0x8A8    // Auto Focus. Bank2 checksum address 
#define AF_CS3_SA 0x8B4    // Auto Focus. Bank3 checksum address 

#define CI_FLAG_A 0x801    // Chip Information. flag address 
#define CI_B1_SA 0x802     // Chip Information. Bank1 start address 
#define CI_B2_SA 0x813     // Chip Information. Bank2 start address 
#define CI_B3_SA 0x824     // Chip Information. Bank3 start address 
#define CI_CS1_SA 0x812    // Chip Information. Bank1 checksum address 
#define CI_CS2_SA 0x823    // Chip Information. Bank2 checksum address 
#define CI_CS3_SA 0x834    // Chip Information. Bank3 checksum address 

#define LSC_FLAG_A 0x8B5    // LSC. flag address 
#define LSC_B1_SA 0x8B6     // LSC. Bank1 start address 
#define LSC_B2_SA 0xFAA     // LSC. Bank2 start address 
#define LSC_B3_SA 0x169E    // LSC. Bank3 start address 
#define LSC_CS1_SA 0x1DAB   // LSC. Bank1 checksum address 
#define LSC_CS2_SA 0x1DAC   // LSC. Bank2 checksum address 
#define LSC_CS3_SA 0x1DAD   // LSC. Bank3 checksum address 

#endif __HI842_SENSOR_FUNCTION_DEFINE
