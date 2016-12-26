#ifdef BUILD_LK
#include <string.h>
#include <mt_gpio.h>
#include <platform/mt_pmic.h>
#elif defined(BUILD_UBOOT)
#include <asm/arch/mt_gpio.h>
#else
#include <linux/string.h>
#include <mach/mt_gpio.h>
#include <mach/mt_pm_ldo.h>
#endif

#include "lcm_drv.h"

////////////////////////////////////////////////////
#define FRAME_WIDTH					(480)
#define FRAME_HEIGHT					(854)

#define REGFLAG_DELAY 					0xAB
#define REGFLAG_END_OF_TABLE				0xAA // END OF REGISTERS MARKER

#define LCM_DSI_CMD_MODE				0

#define LCM_ID						0x980604
/////////////////////////////////////////////////////////

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)				(lcm_util.set_reset_pin((v)))
#define UDELAY(n)					(lcm_util.udelay(n))
#define MDELAY(n)					(lcm_util.mdelay(n))

static unsigned int lcm_compare_id(void);
// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)						lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)			lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)						lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)			lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)       

struct LCM_setting_table {
    unsigned char cmd;
    unsigned char count;
    unsigned char para_list[64];
};


static struct LCM_setting_table lcm_initialization_setting[] = {
//parsed by batya aka vitek999 XD
{0xFF,5,{0xFF,0x98,0x06,0x04,0x01}},	// Change to Page 1
{0x08,1,{0x10}},			// output SDA
{0x21,1,{0x01}},			// DE = 1 Active
{0x30,1,{0x01}},			// 480 X 854
{0x31,1,{0x02}},			// 2-dot Inversion
{0x40,1,{0x11}},
{0x41,1,{0x44}},
{0x42,1,{0x02}},
{0x43,1,{0x09}},
{0x44,1,{0x07}},
{0x50,1,{0x78}},
{0x51,1,{0x78}},
{0x52,1,{0x00}},
{0x53,1,{0x4A}},
{0x60,1,{0x0A}},
{0x61,1,{0x00}},
{0x62,1,{0x0A}},
{0x63,1,{0x00}},

{0xA0,1,{0x00}},
{0xA1,1,{0x06}},
{0xA2,1,{0x0E}},
{0xA3,1,{0x0F}},
{0xA4,1,{0x07}},
{0xA5,1,{0x15}},
{0xA6,1,{0x07}},
{0xA7,1,{0x07}},
{0xA8,1,{0x03}},
{0xA9,1,{0x08}},
{0xAA,1,{0x0E}},
{0xAB,1,{0x09}},
{0xAC,1,{0x0C}},
{0xAD,1,{0x2D}},
{0xAE,1,{0x28}},
{0xAF,1,{0x00}},

{0xC0,1,{0x00}},
{0xC1,1,{0x05}},
{0xC2,1,{0x0C}},
{0xC3,1,{0x10}},
{0xC4,1,{0x0A}},
{0xC5,1,{0x16}},
{0xC6,1,{0x0B}},
{0xC7,1,{0x08}},
{0xC8,1,{0x04}},
{0xC9,1,{0x09}},
{0xCA,1,{0x01}},
{0xCB,1,{0x01}},
{0xCC,1,{0x0B}},
{0xCD,1,{0x2D}},
{0xCE,1,{0x29}},
{0xCF,1,{0x00}},

{0xFF,5,{0xFF,0x98,0x06,0x04,0x06}},
{0x00,1,{0x21}},
{0x01,1,{0x0A}},
{0x02,1,{0x00}},
{0x03,1,{0x00}},
{0x04,1,{0x01}},
{0x05,1,{0x01}},
{0x06,1,{0x80}},
{0x07,1,{0x06}},
{0x08,1,{0x01}},
{0x09,1,{0x80}},
{0x0A,1,{0x00}},
{0x0B,1,{0x00}},
{0x0C,1,{0x0A}},
{0x0D,1,{0x0A}},
{0x0E,1,{0x00}},
{0x0F,1,{0x00}},
{0x10,1,{0xF0}},
{0x11,1,{0xF4}},
{0x12,1,{0x04}},
{0x13,1,{0x00}},
{0x14,1,{0x00}},
{0x15,1,{0xC0}},
{0x16,1,{0x08}},
{0x17,1,{0x00}},
{0x18,1,{0x00}},
{0x19,1,{0x00}},
{0x1A,1,{0x00}},
{0x1B,1,{0x00}},
{0x1C,1,{0x00}},
{0x1D,1,{0x00}},
{0x20,1,{0x01}},
{0x21,1,{0x23}},
{0x22,1,{0x45}},
{0x23,1,{0x67}},
{0x24,1,{0x01}},
{0x25,1,{0x23}},
{0x26,1,{0x45}},
{0x27,1,{0x67}},
{0x30,1,{0x01}},
{0x31,1,{0x11}},
{0x32,1,{0x00}},
{0x33,1,{0xEE}},
{0x34,1,{0xFF}},
{0x35,1,{0xBB}},
{0x36,1,{0xCA}},
{0x37,1,{0xDD}},
{0x38,1,{0xAC}},
{0x39,1,{0x76}},
{0x3A,1,{0x67}},
{0x3B,1,{0x22}},
{0x3C,1,{0x22}},
{0x3D,1,{0x22}},
{0x3E,1,{0x22}},
{0x40,1,{0x22}},
{0x52,1,{0x10}},
{0x53,1,{0x10}},

{0xFF,5,{0xFF,0x98,0x06,0x04,0x07}},
{0x17,1,{0x22}},
{0x02,1,{0x77}},
{0xE1,1,{0x79}},

{0xFF,5,{0xFF,0x98,0x06,0x04,0x00}},	// Change to Page 0

{0x11, 1, {0x00}},			// Sleep-Out
{REGFLAG_DELAY, 120, {}},
{0x29, 1, {0x00}},			// Display on
{REGFLAG_DELAY, 5, {}},
{0x2c,1,{0x00}},
{REGFLAG_DELAY, 5, {}},

{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_set_window[] = {
	{0x2A,	4,	{0x00, 0x00, (FRAME_WIDTH>>8), (FRAME_WIDTH&0xFF)}},
	{0x2B,	4,	{0x00, 0x00, (FRAME_HEIGHT>>8), (FRAME_HEIGHT&0xFF)}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};
static struct LCM_setting_table lcm_sleep_out_setting[] = {
	{0x11, 0, {0x00}},
    {REGFLAG_DELAY, 120, {}},
	{0x29, 0, {0x00}},
    {REGFLAG_DELAY, 100, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};
static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] = {
	{0x28, 0, {0x00}},
    {REGFLAG_DELAY, 120, {}},
	{0x10, 0, {0x00}},
    {REGFLAG_DELAY, 20, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};
static struct LCM_setting_table lcm_backlight_level_setting[] = {
	{0x51, 1, {0xFF}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
	unsigned int i;
    for(i = 0; i < count; i++) {
        unsigned cmd;
        cmd = table[i].cmd;
        switch (cmd) {
            case REGFLAG_DELAY :
                MDELAY(table[i].count);
                break;
            case REGFLAG_END_OF_TABLE :
                break;
            default:
				dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
       	}
    }
	
}


// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
    memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}


static void lcm_get_params(LCM_PARAMS *params)
{
	memset(params, 0, sizeof(LCM_PARAMS)); 

	params->type   = LCM_TYPE_DSI;
	params->width  = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;
	// enable tearing-free
	params->dbi.te_mode          = LCM_DBI_TE_MODE_VSYNC_ONLY;
	params->dbi.te_edge_polarity = LCM_POLARITY_RISING;

#if (LCM_DSI_CMD_MODE)
	params->dsi.mode = CMD_MODE;
#else
	params->dsi.mode = SYNC_PULSE_VDO_MODE;//SYNC_EVENT_VDO_MODE;
#endif

	// DSI
	/* Command mode setting */
	params->dsi.LANE_NUM = LCM_TWO_LANE;
	//The following defined the fomat for data coming from LCD engine. 

	params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;	
	params->dsi.data_format.trans_seq = LCM_DSI_TRANS_SEQ_MSB_FIRST; 
	params->dsi.data_format.padding = LCM_DSI_PADDING_ON_LSB; 
	params->dsi.data_format.format = LCM_DSI_FORMAT_RGB888; 

	// Highly depends on LCD driver capability. 
	params->dsi.packet_size = 256; 
	// Video mode setting 
	params->dsi.intermediat_buffer_num = 2; 
	params->dsi.PS = LCM_PACKED_PS_24BIT_RGB888; 

	params->dsi.vertical_sync_active = 4; 
	params->dsi.vertical_backporch = 16; 
	params->dsi.vertical_frontporch = 20; 
	params->dsi.esd_check_enable = 1;
	params->dsi.vertical_active_line = FRAME_HEIGHT; 

	params->dsi.horizontal_sync_active				= 10;
	params->dsi.horizontal_backporch				= 80;
	params->dsi.horizontal_frontporch				= 80;
	params->dsi.horizontal_blanking_pixel				= 60;
	params->dsi.horizontal_active_pixel 			= FRAME_WIDTH;
	// Bit rate calculation
	params->dsi.PLL_CLOCK = 260;
	// Bit rate calculation
    	params->dsi.pll_div1=0;		// div1=0,1,2,3;div1_real=1,2,4,4
    	params->dsi.pll_div2=0;		// div2=0,1,2,3;div2_real=1,2,4,4
    	params->dsi.fbk_div =28;	// fref=26MHz, fvco=fref*(fbk_div+1)*2/(div1_real*div2_real)		
}

static void lcm_init(void)
{
    	SET_RESET_PIN(1);
    	SET_RESET_PIN(0);
    	MDELAY(10);
    	SET_RESET_PIN(1);
    	MDELAY(120);
	push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
}

static void lcm_suspend(void)
{
	push_table(lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);
}

static void lcm_resume(void)
{
	lcm_init();
}

static unsigned int lcm_compare_id(void)
{
    return 1;
}

LCM_DRIVER ili9806e_dsi_vdo_lcm_drv = 
{
    .name			= "ili9806e_dsi_vdo",
	.set_util_funcs = lcm_set_util_funcs,
    	.compare_id    = lcm_compare_id,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
};
