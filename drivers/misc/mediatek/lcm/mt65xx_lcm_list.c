#include <lcm_drv.h>
#ifdef BUILD_LK
#include <platform/disp_drv_platform.h>
#else
#include <linux/delay.h>
#include <mach/mt_gpio.h>
#endif
#include <cust_gpio_usage.h>
//used to identify float ID PIN status
#define LCD_HW_ID_STATUS_LOW      0
#define LCD_HW_ID_STATUS_HIGH     1
#define LCD_HW_ID_STATUS_FLOAT 0x02
#define LCD_HW_ID_STATUS_ERROR  0x03

#ifdef BUILD_LK
#define LCD_DEBUG(fmt)  dprintf(CRITICAL,fmt)
#else
#define LCD_DEBUG(fmt)  printk(fmt)
#endif
extern LCM_DRIVER hx8394f_hd720_dsi_vdo_txd;
extern LCM_DRIVER hct_otm8019a_dsi_vdo_wvga_boe;
extern LCM_DRIVER hct_ili9806e_dsi_vdo_wvga_boe;
extern LCM_DRIVER hct_ili9806e_dsi_vdo_wvga_tm;
extern LCM_DRIVER hct_s6d2aa0_dsi_vdo_hd_boe;
extern LCM_DRIVER hct_nt35521_dsi_vdo_hd_auo;
extern LCM_DRIVER hct_nt35521_dsi_vdo_hd_boe;
extern LCM_DRIVER hct_r61318_dsi_vdo_hd_cpt;
extern LCM_DRIVER hct_r61318_dsi_vdo_hd_auo;
extern LCM_DRIVER hct_nt35517_dsi_vdo_qhd_cpt;
extern LCM_DRIVER hct_rm68172_dsi_vdo_fwvga_auo;
extern LCM_DRIVER hct_nt35596_dsi_vdo_fhd_auo;
extern LCM_DRIVER hct_otm1283a_dsi_vdo_hd_tm;
extern LCM_DRIVER hct_otm1284a_dsi_vdo_hd_lg;
extern LCM_DRIVER hct_otm1284a_dsi_vdo_hd_boe;
extern LCM_DRIVER hct_otm1285a_dsi_vdo_hd_tm;
extern LCM_DRIVER hct_otm1283a_dsi_vdo_hd_cmo;
extern LCM_DRIVER hct_otm1287a_dsi_vdo_hd_auo;
extern LCM_DRIVER hct_nt35517_dsi_vdo_qhd_lg;
extern LCM_DRIVER hct_otm1287a_dsi_vdo_hd_hsd;
extern LCM_DRIVER hct_hx8389c_dsi_vdo_qhd_cpt;
extern LCM_DRIVER hct_otm9605a_dsi_vdo_qhd_lg;
extern LCM_DRIVER hct_otm1283a_dsi_vdo_hd_lg;
extern LCM_DRIVER hct_nt35521_dsi_vdo_hd_cmi;
extern LCM_DRIVER hct_rm68191_dsi_vdo_qhd_cpt;
extern LCM_DRIVER hct_rm68191_dsi_vdo_qhd_cmi;
extern LCM_DRIVER hct_otm1287a_dsi_vdo_hd_cmi;
extern LCM_DRIVER hct_otm8019a_dsi_vdo_fwvga_auo;
extern LCM_DRIVER hct_rm68191_dsi_vdo_qhd_ivo;
extern LCM_DRIVER hct_ili9881_dsi_vdo_hd_idt;
extern LCM_DRIVER hct_ili9881_dsi_vdo_hd_cmi;
extern LCM_DRIVER hct_rm68191_dsi_vdo_qhd_tm;
extern LCM_DRIVER hct_rm68191_dsi_vdo_qhd_auo;
extern LCM_DRIVER hct_hx8369b_dsi_vdo_wvga_boe;
extern LCM_DRIVER hct_rm68172_dsi_vdo_wvga_boe;
extern LCM_DRIVER hct_jd9161_dsi_vdo_fwvga_tm;
extern LCM_DRIVER hct_nt35512_dsi_vdo_qhd_lg;
extern LCM_DRIVER hct_nt35512_dsi_vdo_fwvga_lg;
extern LCM_DRIVER hct_otm1283a_dsi_vdo_hd_cmi;
extern LCM_DRIVER hct_nt35521_dsi_vdo_hd_tm;
extern LCM_DRIVER hct_hx8379c_dsi_vdo_fwvga_ivo;
extern LCM_DRIVER hct_hx8379c_dsi_vdo_fwvga_hsd;
extern LCM_DRIVER hct_hx8394f_dsi_vdo_hd_auo;
extern LCM_DRIVER hct_hx8394f_dsi_vdo_hd_auo2;
extern LCM_DRIVER hct_hx8394f_dsi_vdo_hd_xp;
extern LCM_DRIVER hct_hx8394f_dsi_vdo_hd_cpt;
extern LCM_DRIVER hct_hx8394f_dsi_vdo_hd_cmi;
extern LCM_DRIVER hct_ili9881_dsi_vdo_hd_ivo;
extern LCM_DRIVER hct_otm1287a_dsi_vdo_hd_tm;
extern LCM_DRIVER hct_otm1901a_dsi_vdo_fhd_lg;
extern LCM_DRIVER hct_hx8399_dsi_vdo_fhd_lg;
extern LCM_DRIVER hct_otm1282a_dsi_vdo_hd_auo;
extern LCM_DRIVER hct_nt35595_dsi_vdo_fhd_lg;
extern LCM_DRIVER hct_rm68200_dsi_vdo_hd_auo;
extern LCM_DRIVER hct_rm68191_dsi_vdo_qhd_boe;
extern LCM_DRIVER hct_nt35596_dsi_vdo_fhd_lg;
extern LCM_DRIVER hct_otm1287a_dsi_vdo_hd_lg;
extern LCM_DRIVER hct_hx8394f_dsi_vdo_hd_lg;
extern LCM_DRIVER hct_rm68172_dsi_vdo_fwvga_ivo;
extern LCM_DRIVER hct_ili9806e_dsi_vdo_fwvga_ivo;
extern LCM_DRIVER hct_otm9608a_dsi_vdo_qhd_lg;
extern LCM_DRIVER hct_nt35521_dsi_vdo_hd_lg;
extern LCM_DRIVER hct_ili9881_dsi_vdo_hd_cpt;
extern LCM_DRIVER hct_otm1283a_dsi_vdo_hd_auo;
extern LCM_DRIVER hct_jd9367_dsi_vdo_hd_tm;
extern LCM_DRIVER hct_jd9367_dsi_vdo_hd_lg;
extern LCM_DRIVER hct_otm1283a_dsi_vdo_hd_hsd;
extern LCM_DRIVER hct_rm69052_dsi_vdo_hd_auo;
extern LCM_DRIVER hct_rm68200_dsi_vdo_hd_lg;
extern LCM_DRIVER hct_rm68200_dsi_vdo_hd_tm;
extern LCM_DRIVER hct_r63317_dsi_vdo_hd_jdi;
extern LCM_DRIVER hct_r63311_dsi_vdo_fhd_jdi;
extern LCM_DRIVER hct_r63315_dsi_vdo_fhd_tm;
extern LCM_DRIVER hct_ili9881_dsi_vdo_hd_tft;
extern LCM_DRIVER hct_r61318_dsi_vdo_hd_lg;
extern LCM_DRIVER hct_ili9881c_dsi_vdo_hd_hsd;
extern LCM_DRIVER hct_rm68200_dsi_vdo_hd_auo_100k47k;
extern LCM_DRIVER hct_nt35596_dsi_vdo_fhd_tm;
extern LCM_DRIVER hct_otm1287a_dsi_vdo_hd_boe;
extern LCM_DRIVER hct_rm68200_dsi_vdo_hd_hsd;
LCM_DRIVER* lcm_driver_list[] = 
{
#if defined(HX8394F_HD720_DSI_VDO_TXD)
	&hx8394f_hd720_dsi_vdo_txd,
#endif

#if defined(HCT_OTM8019A_DSI_VDO_WVGA_BOE)
	&hct_otm8019a_dsi_vdo_wvga_boe,
#endif
#if defined(HCT_ILI9806E_DSI_VDO_WVGA_BOE)
	&hct_ili9806e_dsi_vdo_wvga_boe,
#endif
#if defined(HCT_ILI9806E_DSI_VDO_WVGA_TM)
	&hct_ili9806e_dsi_vdo_wvga_tm,
#endif
#if defined(HCT_NT35521_DSI_VDO_HD_AUO)
	&hct_nt35521_dsi_vdo_hd_auo,
#endif
#if defined(HCT_NT35521_DSI_VDO_HD_BOE)
	&hct_nt35521_dsi_vdo_hd_boe,
#endif
#if defined(HCT_S6D2AA0_DSI_VDO_HD_BOE)
	&hct_s6d2aa0_dsi_vdo_hd_boe,
#endif
#if defined(HCT_R61318_DSI_VDO_HD_CPT)
	&hct_r61318_dsi_vdo_hd_cpt,
#endif
#if defined(HCT_R61318_DSI_VDO_HD_AUO)
	&hct_r61318_dsi_vdo_hd_auo,
#endif
#if defined(HCT_NT35517_DSI_VDO_QHD_CPT)
	&hct_nt35517_dsi_vdo_qhd_cpt,
#endif
#if defined(HCT_RM68172_DSI_VDO_FWVGA_AUO)
	&hct_rm68172_dsi_vdo_fwvga_auo,
#endif
#if defined(HCT_NT35596_DSI_VDO_FHD_AUO)
	&hct_nt35596_dsi_vdo_fhd_auo,
#endif
#if defined(HCT_OTM1283A_DSI_VDO_HD_TM)
	&hct_otm1283a_dsi_vdo_hd_tm,
#endif
#if defined(HCT_OTM1284A_DSI_VDO_HD_LG)
	&hct_otm1284a_dsi_vdo_hd_lg,
#endif
#if defined(HCT_OTM1284A_DSI_VDO_HD_BOE)
	&hct_otm1284a_dsi_vdo_hd_boe,
#endif
#if defined(HCT_OTM1285A_DSI_VDO_HD_TM)
	&hct_otm1285a_dsi_vdo_hd_tm,
#endif
#if defined(HCT_OTM1283A_DSI_VDO_HD_CMO)
	&hct_otm1283a_dsi_vdo_hd_cmo,
#endif
#if defined(HCT_OTM1287A_DSI_VDO_HD_AUO)
	&hct_otm1287a_dsi_vdo_hd_auo,
#endif
#if defined(HCT_NT35517_DSI_VDO_QHD_LG)
	&hct_nt35517_dsi_vdo_qhd_lg,
#endif
#if defined(HCT_OTM1287A_DSI_VDO_HD_HSD)
	&hct_otm1287a_dsi_vdo_hd_hsd,
#endif
#if defined(HCT_HX8389C_DSI_VDO_QHD_CPT)
	&hct_hx8389c_dsi_vdo_qhd_cpt,
#endif
#if defined(HCT_OTM1283A_DSI_VDO_HD_LG)
	&hct_otm1283a_dsi_vdo_hd_lg,
#endif
#if defined(HCT_NT35521_DSI_VDO_HD_CMI)
	&hct_nt35521_dsi_vdo_hd_cmi,
#endif
#if defined(HCT_RM68191_DSI_VDO_QHD_CPT)
	&hct_rm68191_dsi_vdo_qhd_cpt,
#endif
#if defined(HCT_RM68191_DSI_VDO_QHD_CMI)
	&hct_rm68191_dsi_vdo_qhd_cmi,
#endif
#if defined(HCT_OTM1287A_DSI_VDO_HD_CMI)
	&hct_otm1287a_dsi_vdo_hd_cmi,
#endif
#if defined(HCT_OTM9605A_DSI_VDO_QHD_LG)
	&hct_otm9605a_dsi_vdo_qhd_lg,
#endif
#if defined(HCT_OTM8019A_DSI_VDO_FWVGA_AUO)
	&hct_otm8019a_dsi_vdo_fwvga_auo,
#endif
#if defined(HCT_RM68191_DSI_VDO_QHD_IVO)
	&hct_rm68191_dsi_vdo_qhd_ivo,
#endif
#if defined(HCT_ILI9881_DSI_VDO_HD_IDT)
	&hct_ili9881_dsi_vdo_hd_idt,
#endif
#if defined(HCT_RM68191_DSI_VDO_QHD_TM)
	&hct_rm68191_dsi_vdo_qhd_tm,
#endif
#if defined(HCT_RM68191_DSI_VDO_QHD_AUO)
	&hct_rm68191_dsi_vdo_qhd_auo,
#endif
#if defined(HCT_HX8369B_DSI_VDO_WVGA_BOE)
    &hct_hx8369b_dsi_vdo_wvga_boe,
#endif    
#if defined(HCT_RM68172_DSI_VDO_WVGA_BOE)
    &hct_rm68172_dsi_vdo_wvga_boe,
#endif 
#if defined(HCT_JD9161_DSI_VDO_FWVGA_TM)
    &hct_jd9161_dsi_vdo_fwvga_tm,
#endif 
#if defined(HCT_NT35512_DSI_VDO_QHD_LG)
   &hct_nt35512_dsi_vdo_qhd_lg,
#endif
#if defined(HCT_NT35512_DSI_VDO_FWVGA_LG)
   &hct_nt35512_dsi_vdo_fwvga_lg,
#endif
#if defined(HCT_OTM1283A_DSI_VDO_HD_CMI)
	&hct_otm1283a_dsi_vdo_hd_cmi,
#endif
#if defined(HCT_NT35521_DSI_VDO_HD_TM)
	&hct_nt35521_dsi_vdo_hd_tm,
#endif
#if defined(HCT_ILI9881_DSI_VDO_HD_CMI)
	&hct_ili9881_dsi_vdo_hd_cmi,
#endif
#if defined(HCT_HX8379C_DSI_VDO_FWVGA_IVO)
   &hct_hx8379c_dsi_vdo_fwvga_ivo,
#endif
#if defined(HCT_HX8394F_DSI_VDO_HD_AUO2)
	&hct_hx8394f_dsi_vdo_hd_auo2,
#endif
#if defined(HCT_HX8394F_DSI_VDO_HD_AUO)
	&hct_hx8394f_dsi_vdo_hd_auo,
#endif
#if defined(HCT_HX8394F_DSI_VDO_HD_XP)
	&hct_hx8394f_dsi_vdo_hd_xp,
#endif
#if defined(HCT_HX8394F_DSI_VDO_HD_CPT)
	&hct_hx8394f_dsi_vdo_hd_cpt,
#endif
#if defined(HCT_HX8394F_DSI_VDO_HD_CMI)
	&hct_hx8394f_dsi_vdo_hd_cmi,
#endif
#if defined(HCT_ILI9881_DSI_VDO_HD_IVO)
	&hct_ili9881_dsi_vdo_hd_ivo,
#endif
#if defined(HCT_OTM1287A_DSI_VDO_HD_TM)
	&hct_otm1287a_dsi_vdo_hd_tm,
#endif
#if defined(HCT_OTM1901A_DSI_VDO_FHD_LG)
	&hct_otm1901a_dsi_vdo_fhd_lg,
#endif
#if defined(HCT_HX8399_DSI_VDO_FHD_LG)
	&hct_hx8399_dsi_vdo_fhd_lg,
#endif
#if defined(HCT_OTM1282A_DSI_VDO_HD_AUO)
	&hct_otm1282a_dsi_vdo_hd_auo,
#endif
#if defined(HCT_NT35595_DSI_VDO_FHD_LG)
	&hct_nt35595_dsi_vdo_fhd_lg,
#endif
#if defined(HCT_RM68191_DSI_VDO_QHD_BOE)
	&hct_rm68191_dsi_vdo_qhd_boe,
#endif
#if defined(HCT_RM68200_DSI_VDO_HD_AUO)
	&hct_rm68200_dsi_vdo_hd_auo,
#endif
#if defined(HCT_NT35596_DSI_VDO_FHD_LG)
	&hct_nt35596_dsi_vdo_fhd_lg,
#endif
#if defined(HCT_OTM1287A_DSI_VDO_HD_LG)
	&hct_otm1287a_dsi_vdo_hd_lg,
#endif
#if defined(HCT_HX8394F_DSI_VDO_HD_LG)
	&hct_hx8394f_dsi_vdo_hd_lg,
#endif
#if defined(HCT_RM68172_DSI_VDO_FWVGA_IVO)
	&hct_rm68172_dsi_vdo_fwvga_ivo,
#endif
#if defined(HCT_ILI9806E_DSI_VDO_FWVGA_IVO)
	&hct_ili9806e_dsi_vdo_fwvga_ivo,
#endif
#if defined(HCT_HX8379C_DSI_VDO_FWVGA_HSD)
   &hct_hx8379c_dsi_vdo_fwvga_hsd,
#endif
#if defined(HCT_OTM9608A_DSI_VDO_QHD_LG)
	&hct_otm9608a_dsi_vdo_qhd_lg,
#endif
#if defined(HCT_NT35521_DSI_VDO_HD_LG)
	&hct_nt35521_dsi_vdo_hd_lg,
#endif
#if defined(HCT_OTM1283A_DSI_VDO_HD_AUO)
	&hct_otm1283a_dsi_vdo_hd_auo,
#endif
#if defined(HCT_JD9367_DSI_VDO_HD_TM)
	&hct_jd9367_dsi_vdo_hd_tm,
#endif
#if defined(HCT_JD9367_DSI_VDO_HD_LG)
	&hct_jd9367_dsi_vdo_hd_lg,
#endif
#if defined(HCT_OTM1283A_DSI_VDO_HD_HSD)
	&hct_otm1283a_dsi_vdo_hd_hsd,
#endif
#if defined(HCT_RM69052_DSI_VDO_HD_AUO)
	&hct_rm69052_dsi_vdo_hd_auo,
#endif
#if defined(HCT_RM68200_DSI_VDO_HD_LG)
	&hct_rm68200_dsi_vdo_hd_lg,
#endif
#if defined(HCT_RM68200_DSI_VDO_HD_TM)
	&hct_rm68200_dsi_vdo_hd_tm,
#endif
#if defined(HCT_R63317_DSI_VDO_HD_JDI)
	&hct_r63317_dsi_vdo_hd_jdi,
#endif
#if defined(HCT_R63311_DSI_VDO_FHD_JDI)
	&hct_r63311_dsi_vdo_fhd_jdi,
#endif
#if defined(HCT_R63315_DSI_VDO_FHD_TM)
	&hct_r63315_dsi_vdo_fhd_tm,
#endif
#if defined(HCT_ILI9881_DSI_VDO_HD_TFT)
	&hct_ili9881_dsi_vdo_hd_tft,
#endif
#if defined(HCT_R61318_DSI_VDO_HD_LG)
	&hct_r61318_dsi_vdo_hd_lg,
#endif
#if defined(HCT_ILI9881C_DSI_VDO_HD_HSD)
	&hct_ili9881c_dsi_vdo_hd_hsd,
#endif
#if defined(HCT_RM68200_DSI_VDO_HD_AUO_100K47K)
	&hct_rm68200_dsi_vdo_hd_auo_100k47k,
#endif
#if defined(HCT_NT35596_DSI_VDO_FHD_TM)
	&hct_nt35596_dsi_vdo_fhd_tm,
#endif
#if defined(HCT_OTM1287A_DSI_VDO_HD_BOE)
	&hct_otm1287a_dsi_vdo_hd_boe,
#endif
#if defined(HCT_RM68200_DSI_VDO_HD_HSD)
	&hct_rm68200_dsi_vdo_hd_hsd,
#endif
#if defined(HCT_ILI9881_DSI_VDO_HD_CPT)
	&hct_ili9881_dsi_vdo_hd_cpt,
#endif
};

#define LCM_COMPILE_ASSERT(condition) LCM_COMPILE_ASSERT_X(condition, __LINE__)
#define LCM_COMPILE_ASSERT_X(condition, line) LCM_COMPILE_ASSERT_XX(condition, line)
#define LCM_COMPILE_ASSERT_XX(condition, line) char assertion_failed_at_line_##line[(condition)?1:-1]

unsigned int lcm_count = sizeof(lcm_driver_list)/sizeof(LCM_DRIVER*);
LCM_COMPILE_ASSERT(0 != sizeof(lcm_driver_list)/sizeof(LCM_DRIVER*));
#if defined(NT35520_HD720_DSI_CMD_TM) | defined(NT35520_HD720_DSI_CMD_BOE) | defined(NT35521_HD720_DSI_VDO_BOE) | defined(NT35521_HD720_DSI_VIDEO_TM)
#ifdef BUILD_LK
extern void mdelay(unsigned long msec);
#endif
static unsigned char lcd_id_pins_value = 0xFF;


/******************************************************************************
Function:       which_lcd_module_triple
  Description:    read LCD ID PIN status,could identify three status:highlowfloat
  Input:           none
  Output:         none
  Return:         LCD ID1|ID0 value
  Others:         
******************************************************************************/
unsigned char which_lcd_module_triple(void)
{
    unsigned char  high_read0 = 0;
    unsigned char  low_read0 = 0;
    unsigned char  high_read1 = 0;
    unsigned char  low_read1 = 0;
    unsigned char  lcd_id0 = 0;
    unsigned char  lcd_id1 = 0;
    unsigned char  lcd_id = 0;
    //Solve Coverity scan warning : check return value
    unsigned int ret = 0;
    //only recognise once
    if(0xFF != lcd_id_pins_value) 
    {
        return lcd_id_pins_value;
    }
    //Solve Coverity scan warning : check return value
    ret = mt_set_gpio_mode(GPIO_DISP_ID0_PIN, GPIO_MODE_00);
    if(0 != ret)
    {
        LCD_DEBUG("ID0 mt_set_gpio_mode fail\n");
    }
    ret = mt_set_gpio_dir(GPIO_DISP_ID0_PIN, GPIO_DIR_IN);
    if(0 != ret)
    {
        LCD_DEBUG("ID0 mt_set_gpio_dir fail\n");
    }
    ret = mt_set_gpio_pull_enable(GPIO_DISP_ID0_PIN, GPIO_PULL_ENABLE);
    if(0 != ret)
    {
        LCD_DEBUG("ID0 mt_set_gpio_pull_enable fail\n");
    }
    ret = mt_set_gpio_mode(GPIO_DISP_ID1_PIN, GPIO_MODE_00);
    if(0 != ret)
    {
        LCD_DEBUG("ID1 mt_set_gpio_mode fail\n");
    }
    ret = mt_set_gpio_dir(GPIO_DISP_ID1_PIN, GPIO_DIR_IN);
    if(0 != ret)
    {
        LCD_DEBUG("ID1 mt_set_gpio_dir fail\n");
    }
    ret = mt_set_gpio_pull_enable(GPIO_DISP_ID1_PIN, GPIO_PULL_ENABLE);
    if(0 != ret)
    {
        LCD_DEBUG("ID1 mt_set_gpio_pull_enable fail\n");
    }
    //pull down ID0 ID1 PIN    
    ret = mt_set_gpio_pull_select(GPIO_DISP_ID0_PIN,GPIO_PULL_DOWN);
    if(0 != ret)
    {
        LCD_DEBUG("ID0 mt_set_gpio_pull_select->Down fail\n");
    }
    ret = mt_set_gpio_pull_select(GPIO_DISP_ID1_PIN,GPIO_PULL_DOWN);
    if(0 != ret)
    {
        LCD_DEBUG("ID1 mt_set_gpio_pull_select->Down fail\n");
    }
    //delay 100ms , for discharging capacitance 
    mdelay(100);
    //get ID0 ID1 status
    low_read0 = mt_get_gpio_in(GPIO_DISP_ID0_PIN);
    low_read1 = mt_get_gpio_in(GPIO_DISP_ID1_PIN);
    //pull up ID0 ID1 PIN 
    ret = mt_set_gpio_pull_select(GPIO_DISP_ID0_PIN,GPIO_PULL_UP);
    if(0 != ret)
    {
        LCD_DEBUG("ID0 mt_set_gpio_pull_select->UP fail\n");
    }
    ret = mt_set_gpio_pull_select(GPIO_DISP_ID1_PIN,GPIO_PULL_UP);
    if(0 != ret)
    {
        LCD_DEBUG("ID1 mt_set_gpio_pull_select->UP fail\n");
    }
    //delay 100ms , for charging capacitance 
    mdelay(100);
    //get ID0 ID1 status
    high_read0 = mt_get_gpio_in(GPIO_DISP_ID0_PIN);
    high_read1 = mt_get_gpio_in(GPIO_DISP_ID1_PIN);

    if( low_read0 != high_read0 )
    {
        /*float status , pull down ID0 ,to prevent electric leakage*/
        ret = mt_set_gpio_pull_select(GPIO_DISP_ID0_PIN,GPIO_PULL_DOWN);
        if(0 != ret)
        {
            LCD_DEBUG("ID0 mt_set_gpio_pull_select->Down fail\n");
        }
        lcd_id0 = LCD_HW_ID_STATUS_FLOAT;
    }
    else if((LCD_HW_ID_STATUS_LOW == low_read0) && (LCD_HW_ID_STATUS_LOW == high_read0))
    {
        /*low status , pull down ID0 ,to prevent electric leakage*/
        ret = mt_set_gpio_pull_select(GPIO_DISP_ID0_PIN,GPIO_PULL_DOWN);
        if(0 != ret)
        {
            LCD_DEBUG("ID0 mt_set_gpio_pull_select->Down fail\n");
        }
        lcd_id0 = LCD_HW_ID_STATUS_LOW;
    }
    else if((LCD_HW_ID_STATUS_HIGH == low_read0) && (LCD_HW_ID_STATUS_HIGH == high_read0))
    {
        /*high status , pull up ID0 ,to prevent electric leakage*/
        ret = mt_set_gpio_pull_select(GPIO_DISP_ID0_PIN,GPIO_PULL_UP);
        if(0 != ret)
        {
            LCD_DEBUG("ID0 mt_set_gpio_pull_select->UP fail\n");
        }
        lcd_id0 = LCD_HW_ID_STATUS_HIGH;
    }
    else
    {
        LCD_DEBUG(" Read LCD_id0 error\n");
        ret = mt_set_gpio_pull_select(GPIO_DISP_ID0_PIN,GPIO_PULL_DISABLE);
        if(0 != ret)
        {
            LCD_DEBUG("ID0 mt_set_gpio_pull_select->Disbale fail\n");
        }
        lcd_id0 = LCD_HW_ID_STATUS_ERROR;
    }


    if( low_read1 != high_read1 )
    {
        /*float status , pull down ID1 ,to prevent electric leakage*/
        ret = mt_set_gpio_pull_select(GPIO_DISP_ID1_PIN,GPIO_PULL_DOWN);
        if(0 != ret)
        {
            LCD_DEBUG("ID1 mt_set_gpio_pull_select->Down fail\n");
        }
        lcd_id1 = LCD_HW_ID_STATUS_FLOAT;
    }
    else if((LCD_HW_ID_STATUS_LOW == low_read1) && (LCD_HW_ID_STATUS_LOW == high_read1))
    {
        /*low status , pull down ID1 ,to prevent electric leakage*/
        ret = mt_set_gpio_pull_select(GPIO_DISP_ID1_PIN,GPIO_PULL_DOWN);
        if(0 != ret)
        {
            LCD_DEBUG("ID1 mt_set_gpio_pull_select->Down fail\n");
        }
        lcd_id1 = LCD_HW_ID_STATUS_LOW;
    }
    else if((LCD_HW_ID_STATUS_HIGH == low_read1) && (LCD_HW_ID_STATUS_HIGH == high_read1))
    {
        /*high status , pull up ID1 ,to prevent electric leakage*/
        ret = mt_set_gpio_pull_select(GPIO_DISP_ID1_PIN,GPIO_PULL_UP);
        if(0 != ret)
        {
            LCD_DEBUG("ID1 mt_set_gpio_pull_select->UP fail\n");
        }
        lcd_id1 = LCD_HW_ID_STATUS_HIGH;
    }
    else
    {

        LCD_DEBUG(" Read LCD_id1 error\n");
        ret = mt_set_gpio_pull_select(GPIO_DISP_ID1_PIN,GPIO_PULL_DISABLE);
        if(0 != ret)
        {
            LCD_DEBUG("ID1 mt_set_gpio_pull_select->Disable fail\n");
        }
        lcd_id1 = LCD_HW_ID_STATUS_ERROR;
    }
#ifdef BUILD_LK
    dprintf(CRITICAL,"which_lcd_module_triple,lcd_id0:%d\n",lcd_id0);
    dprintf(CRITICAL,"which_lcd_module_triple,lcd_id1:%d\n",lcd_id1);
#else
    printk("which_lcd_module_triple,lcd_id0:%d\n",lcd_id0);
    printk("which_lcd_module_triple,lcd_id1:%d\n",lcd_id1);
#endif
    lcd_id =  lcd_id0 | (lcd_id1 << 2);

#ifdef BUILD_LK
    dprintf(CRITICAL,"which_lcd_module_triple,lcd_id:%d\n",lcd_id);
#else
    printk("which_lcd_module_triple,lcd_id:%d\n",lcd_id);
#endif

    lcd_id_pins_value = lcd_id;
    return lcd_id;
}
#endif
