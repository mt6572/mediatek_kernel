/*
 * Copyright (C) 2010 MediaTek, Inc.
 *
 * Author: Terry Chang <terry.chang@mediatek.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */


/*kpd.h file path: ALPS/mediatek/kernel/include/linux */
#include <linux/kpd.h>
#include <mach/mt_pm_ldo.h>
#include <mach/mt_typedefs.h>
#include <mach/mt_boot.h>

#define KPD_NAME	"mtk-kpd"
#define MTK_KP_WAKESOURCE//this is for auto set wake up source

struct input_dev *kpd_input_dev;
static bool kpd_suspend = false;
static int kpd_show_hw_keycode = 1;
static int kpd_show_register = 1;
static volatile int call_status = 0;

/*for kpd_memory_setting() function*/
static u16 kpd_keymap[KPD_NUM_KEYS];
static u16 kpd_keymap_state[KPD_NUM_MEMS];
/***********************************/

/* for slide QWERTY */
#if KPD_HAS_SLIDE_QWERTY
static void kpd_slide_handler(unsigned long data);
static DECLARE_TASKLET(kpd_slide_tasklet, kpd_slide_handler, 0);
static u8 kpd_slide_state = !KPD_SLIDE_POLARITY;
#endif

/* for Power key using EINT */
#if KPD_PWRKEY_USE_EINT
static void kpd_pwrkey_handler(unsigned long data);
static DECLARE_TASKLET(kpd_pwrkey_tasklet, kpd_pwrkey_handler, 0);
#endif

/* for keymap handling */
static void kpd_keymap_handler(unsigned long data);
static DECLARE_TASKLET(kpd_keymap_tasklet, kpd_keymap_handler, 0);

/*********************************************************************/
static void kpd_memory_setting(void);

/*********************************************************************/
static int kpd_pdrv_probe(struct platform_device *pdev);
static int kpd_pdrv_remove(struct platform_device *pdev);
#ifndef USE_EARLY_SUSPEND	
static int kpd_pdrv_suspend(struct platform_device *pdev, pm_message_t state);
static int kpd_pdrv_resume(struct platform_device *pdev);
#endif


static struct platform_driver kpd_pdrv = {
	.probe		= kpd_pdrv_probe,
	.remove		= kpd_pdrv_remove,
#ifndef USE_EARLY_SUSPEND	
	.suspend	= kpd_pdrv_suspend,
	.resume		= kpd_pdrv_resume,
#endif	
	.driver		= {
		.name	= KPD_NAME,
		.owner	= THIS_MODULE,
	},
};
/********************************************************************/
static void kpd_memory_setting(void)
{
	kpd_init_keymap(kpd_keymap);
	kpd_init_keymap_state(kpd_keymap_state);
	return;
}


/*****************for kpd auto set wake up source*************************/

static ssize_t kpd_store_call_state(struct device_driver *ddri, const char *buf, size_t count)
{
	if (sscanf(buf, "%u", &call_status) != 1) {
			kpd_print("kpd call state: Invalid values\n");
			return -EINVAL;
		}

	switch(call_status)
    	{
        	case 1 :
			kpd_print("kpd call state: Idle state!\n");
     		break;
		case 2 :
			kpd_print("kpd call state: ringing state!\n");
		break;
		case 3 :
			kpd_print("kpd call state: active or hold state!\n");	
		break;
            
		default:
   			kpd_print("kpd call state: Invalid values\n");
        	break;
  	}
	return count;
}

static ssize_t kpd_show_call_state(struct device_driver *ddri, char *buf)
{
	ssize_t res;
	res = snprintf(buf, PAGE_SIZE, "%d\n", call_status);     
	return res;   
}
static DRIVER_ATTR(kpd_call_state,	S_IWUSR | S_IRUGO,	kpd_show_call_state,	kpd_store_call_state);

static struct driver_attribute *kpd_attr_list[] = {
	&driver_attr_kpd_call_state,
};

/*----------------------------------------------------------------------------*/
static int kpd_create_attr(struct device_driver *driver) 
{
	int idx, err = 0;
	int num = (int)(sizeof(kpd_attr_list)/sizeof(kpd_attr_list[0]));
	if (driver == NULL)
	{
		return -EINVAL;
	}

	for(idx = 0; idx < num; idx++)
	{
		if((err = driver_create_file(driver, kpd_attr_list[idx])))
		{            
			kpd_print("driver_create_file (%s) = %d\n", kpd_attr_list[idx]->attr.name, err);
			break;
		}
	}    
	return err;
}
/*----------------------------------------------------------------------------*/
static int kpd_delete_attr(struct device_driver *driver)
	{
	int idx ,err = 0;
	int num = (int)(sizeof(kpd_attr_list)/sizeof(kpd_attr_list[0]));

	if (!driver)
	return -EINVAL;

	for (idx = 0; idx < num; idx++) 
	{
		driver_remove_file(driver, kpd_attr_list[idx]);
	}
	
	return err;
}
/*----------------------------------------------------------------------------*/
/********************************************************************************************/
/************************************************************************************************************************************************/
/* for autotest */
#if KPD_AUTOTEST
static const u16 kpd_auto_keymap[] = {
	KEY_MENU,
	KEY_HOME, KEY_BACK,
	KEY_CALL, KEY_ENDCALL,
	KEY_VOLUMEUP, KEY_VOLUMEDOWN,
	KEY_FOCUS, KEY_CAMERA,
};
#endif
/* for AEE manual dump */
#define AEE_VOLUMEUP_BIT	0
#define AEE_VOLUMEDOWN_BIT	1
#define AEE_DELAY_TIME		15
/* enable volup + voldown was pressed 5~15 s Trigger aee manual dump */
#define AEE_ENABLE_5_15		1
static struct hrtimer aee_timer;
static unsigned long  aee_pressed_keys;
static bool aee_timer_started;

#if AEE_ENABLE_5_15
#define AEE_DELAY_TIME_5S	5
static struct hrtimer aee_timer_5s;
static bool aee_timer_5s_started;
static bool flags_5s;
#endif

static inline void kpd_update_aee_state(void) {
	if(aee_pressed_keys == ((1<<AEE_VOLUMEUP_BIT) | (1<<AEE_VOLUMEDOWN_BIT))) {
		/* if volumeup and volumedown was pressed the same time then start the time of ten seconds */
		aee_timer_started = true;
		
#if AEE_ENABLE_5_15
		aee_timer_5s_started = true;
		hrtimer_start(&aee_timer_5s, 
				ktime_set(AEE_DELAY_TIME_5S, 0),
				HRTIMER_MODE_REL);
#endif
		hrtimer_start(&aee_timer, 
				ktime_set(AEE_DELAY_TIME, 0),
				HRTIMER_MODE_REL);
		kpd_print("aee_timer started\n");
	} else {
		if(aee_timer_started) {
/*
  * hrtimer_cancel - cancel a timer and wait for the handler to finish.
  * Returns:
  *	0 when the timer was not active. 
  *	1 when the timer was active.
 */
			if(hrtimer_cancel(&aee_timer))
			{
				kpd_print("try to cancel hrtimer \n");
#if AEE_ENABLE_5_15
				if(flags_5s)
				{
					printk("Pressed Volup + Voldown5s~15s then trigger aee manual dump.\n");
					aee_kernel_reminding("manual dump", "Trigger Vol Up +Vol Down 5s");
				}
#endif
					
			}
#if AEE_ENABLE_5_15
			flags_5s = false;
#endif
			aee_timer_started = false;
			kpd_print("aee_timer canceled\n");
		}

#if AEE_ENABLE_5_15
		if(aee_timer_5s_started) {
/*
  * hrtimer_cancel - cancel a timer and wait for the handler to finish.
  * Returns:
  *	0 when the timer was not active. 
  *	1 when the timer was active.
 */
			if(hrtimer_cancel(&aee_timer_5s))
			{
				kpd_print("try to cancel hrtimer (5s) \n");
			}
			aee_timer_5s_started = false;
			kpd_print("aee_timer canceled (5s)\n");
		}

#endif
	}
}

static void kpd_aee_handler(u32 keycode, u16 pressed) {
	if(pressed) {
		if(keycode == KEY_VOLUMEUP) {
			__set_bit(AEE_VOLUMEUP_BIT, &aee_pressed_keys);
		} else if(keycode == KEY_VOLUMEDOWN) {
			__set_bit(AEE_VOLUMEDOWN_BIT, &aee_pressed_keys);
		} else {
			return;
		}
		kpd_update_aee_state();
	} else {
		if(keycode == KEY_VOLUMEUP) {
			__clear_bit(AEE_VOLUMEUP_BIT, &aee_pressed_keys);
		} else if(keycode == KEY_VOLUMEDOWN) {
			__clear_bit(AEE_VOLUMEDOWN_BIT, &aee_pressed_keys);
		} else {
			return;
		}
		kpd_update_aee_state();
	}
}

static enum hrtimer_restart aee_timer_func(struct hrtimer *timer) {
	//printk("kpd: vol up+vol down AEE manual dump!\n");
	//aee_kernel_reminding("manual dump ", "Triggered by press KEY_VOLUMEUP+KEY_VOLUMEDOWN");
	aee_trigger_kdb();
	return HRTIMER_NORESTART;
}

#if AEE_ENABLE_5_15
static enum hrtimer_restart aee_timer_5s_func(struct hrtimer *timer) {
	
	//printk("kpd: vol up+vol down AEE manual dump timer 5s !\n");
	flags_5s = true;
	return HRTIMER_NORESTART;
}
#endif

/************************************************************************************************************************************************/

#if KPD_HAS_SLIDE_QWERTY
static void kpd_slide_handler(unsigned long data)
{
	bool slid;
	u8 old_state = kpd_slide_state;

	kpd_slide_state = !kpd_slide_state;
	slid = (kpd_slide_state == !!KPD_SLIDE_POLARITY);
	/* for SW_LID, 1: lid open => slid, 0: lid shut => closed */
	input_report_switch(kpd_input_dev, SW_LID, slid);
	input_sync(kpd_input_dev);
	kpd_print("report QWERTY = %s\n", slid ? "slid" : "closed");

	if(old_state) {
		mt_set_gpio_pull_select(GPIO_QWERTYSLIDE_EINT_PIN, 0);
	} else {
		mt_set_gpio_pull_select(GPIO_QWERTYSLIDE_EINT_PIN, 1);
	}
	/* for detecting the return to old_state */
	mt65xx_eint_set_polarity(KPD_SLIDE_EINT, old_state);
	mt65xx_eint_unmask(KPD_SLIDE_EINT);
}

static void kpd_slide_eint_handler(void)
{
	tasklet_schedule(&kpd_slide_tasklet);
}
#endif

#if KPD_PWRKEY_USE_EINT
static void kpd_pwrkey_handler(unsigned long data)
{
	kpd_pwrkey_handler_hal(data);
}

static void kpd_pwrkey_eint_handler(void)
{
	tasklet_schedule(&kpd_pwrkey_tasklet);
}
#endif

//heyong
#if defined(EXTEND_KEY_AW9523_SUPPORT)

#define GPIO_SIMULATE_I2C

#define AW9523_EINT_GPIO               GPIO25
#define AW9523_EINT_NO                   CUST_EINT_MHALL_NUM

#define KPD_AW9523_SWITCH_DEBOUNCE       10   //50  // 30
#define KPD_AW9523_SWITCH_POLARITY          CUST_EINT_MHALL_POLARITY
#define KPD_AW9523_SWITCH_SENSITIVE      CUST_EINT_EDGE_SENSITIVE  // CUST_EINT_MHALL_SENSITIVE

static void kpd_aw9523_handler(unsigned long data);
static DECLARE_TASKLET(kpd_aw9523_tasklet, kpd_aw9523_handler, 0);
static u8 kpd_aw9523_state = !CUST_EINT_POLARITY_LOW; 

#define LED_SLAVE_ADDR        0xB6
//#if defined(GPIO_SIMULATE_I2C)
#define I2C_SDA_GPIO         GPIO112      // ROW1
#define I2C_SCL_GPIO         GPIO110    // COL1

extern U32 pmic_config_interface (U32 RegNum, U32 val, U32 MASK, U32 SHIFT);
typedef enum {
P0_0=0,
P0_1,
P0_2,
P0_3,
P0_4,
P0_5,
P0_6,
P0_7
} P0_Enum;

typedef enum {
P1_0=0,
P1_1,
P1_2, 
P1_3,
P1_4, 
P1_5,
P1_6,
P1_7
} P1_Enum;

#define AW9523_I2C_MAX_LOOP 50

#define Y_NUM  8  
#define X_NUM  8              // has pullup resistor 

//  P0 ---> X_NUM ---> col         input
//  P1 ---> Y_NUM ---> line(row)   output
const P0_Enum COL[X_NUM] =  {P0_0, P0_1, P0_2, P0_3, P0_4, P0_5, P0_6,P0_7};
const P1_Enum Line[Y_NUM] = {P1_0,P1_1,P1_2, P1_3, P1_4, P1_5, P1_6, P1_7};  

const u8  aw9523_key[Y_NUM][X_NUM]={
//    0              1          2          3          4              5              6           7

       {0xff,    0xff,    0xff,   0xff,     KEY_F1,     KEY_F2,   KEY_TAB,    KEY_LEFTSHIFT},
	{0xff,    0xff,    0xff,   0xff,     KEY_ENTER, KEY_LEFTCTRL,   KEY_7,     KEY_STAR},
	{0xff,    0xff,    0xff,   0xff,     KEY_UP,    KEY_DOWN,KEY_1,     KEY_4},
	{0xff,    0xff,    0xff,   0xff,     KEY_2,       KEY_0,     KEY_8,     KEY_5},
	{0xff,    0xff,    0xff,   0xff,     KEY_3,        0xE4,     KEY_9,     KEY_6},
	{0xff,    0xff,    0xff,   0xff,  KEY_DOWN,   KEY_LEFT, KEY_F3,     KEY_F12},
	{0xff,    0xff,    0xff,   0xff,  KEY_ESC, KEY_DELETE, KEY_F4,     0xff},
	{0xff,    0xff,    0xff,   0xff,     0xff,         0xff,     0xff,     0xff},//0xe4
	
	/*{KEY_1,  0xff,    0xff,   0xff,     0xff,         0xff,     0xff,     0xff},
	{0xff,    0xff,    0xff,   0xff,     0xff,         0xff,     0xff,     KEY_6},
	{0xff,    0xff,    0xff,   0xff,     0xff,         0xff,     0xff,     0xff},
	{0xff,    0xff,    0xff,   0xff,     0xff,         0xff,     0xff,     0xff},
	{0xff,    KEY_5,  KEY_ENDCALL,   KEY_9,   0xff,         0xff,     0xff,     0xff},
	{0xff, KEY_STAR, KEY_3, KEY_0,    0xff,         0xff,     0xff,     0xff},
	{0xff, KEY_CALL, KEY_4,0xe4, 0xff,         0xff,     KEY_8,     0xff},
	{0xff,    KEY_7,    KEY_2, 0xff,     0xff,         0xff,     0xff,     0xff},*/


	/*{KEY_P,     KEY_DEL,    KEY_ENTER, KEY_B,         KEY_H,         KEY_Y},
	{KEY_O,     KEY_L,      KEY_MUTE,  KEY_RIGHTSHIFT,KEY_SPACE,     KEY_SYM},
	{KEY_V,     KEY_G,      KEY_T,     KEY_C,         KEY_F,         KEY_R},
	{KEY_LEFTSHIFT,KEY_0,   KEY_SPACE, KEY_M,         KEY_K,         KEY_I},
	{KEY_N,     KEY_J,      KEY_U,     KEY_LEFTALT,   KEY_A,         KEY_VOLUMEDOWN},
	{KEY_Z,     KEY_S,      KEY_W,     KEY_X,         KEY_D,         KEY_E},*/
};

//u8 P0_kbd_used[8]={1, 1, 1, 1, 1, 1, 0, 0};
//
u8 P0_kbd_used[8]={0, 0, 0, 0, 1, 1, 1, 1};
//u8 P1_kbd_used[8]={0, 0, 1, 1, 1, 1, 1, 1}; 
u8 P1_kbd_used[8]={1, 1, 1, 1, 1, 1, 1, 0}; 

typedef  enum {
    KEY_STATE_PRESSED=0,
    KEY_STATE_RELEASED,
    KEY_STATE_LONGPRESS,
    KEY_STATE_REPEATED, 
    KEY_STATE_NULL 
}TOUCHKEY_STATE;


u8  P0_INT_STATE=0x0;
u8  P1_INT_STATE=0x0;
u8  P0_IN_OUT_STATE=0x0;
u8  P1_IN_OUT_STATE=0x0;
u8  P0_kbd_used_temp=0x0;
u8  pre_x=0x00;
u8  pre_y=0x00;
u8  P0_X[X_NUM];
u8  P1_Y[Y_NUM];
u8  P1_VALUE=0;
u8 KeyBoard_Key=0xFF;
u8 KeyBoard_Key_Previous=0xFF;
TOUCHKEY_STATE KeyBoardKey_State=KEY_STATE_NULL;

//extern void kpled_ctrl_open(u8 enable);
#define AW9523_delay_1us(ms)     udelay(ms)


#define GPIO_ModeSetup(x, y)      mt_set_gpio_mode(x, y);
#define GPIO_InitIO(x, y)                mt_set_gpio_dir(y, x)
#define GPIO_WriteIO(x, y)            mt_set_gpio_out(y, x)
#define GPIO_ReadIO(x)                  mt_get_gpio_in(x)


#define I2C_SDA_MODE           mt_set_gpio_mode(I2C_SDA_GPIO, GPIO_MODE_GPIO)
#define I2C_SCL_MODE           mt_set_gpio_mode(I2C_SCL_GPIO, GPIO_MODE_GPIO)
#define I2C_SDA_OUTPUT      mt_set_gpio_dir(I2C_SDA_GPIO, GPIO_DIR_OUT)
#define I2C_SDA_INPUT          mt_set_gpio_dir(I2C_SDA_GPIO, GPIO_DIR_IN)
#define I2C_SCL_OUTPUT       mt_set_gpio_dir(I2C_SCL_GPIO, GPIO_DIR_OUT)
#define I2C_SDA_HIGH            mt_set_gpio_out(I2C_SDA_GPIO, GPIO_OUT_ONE)
#define I2C_SDA_LOW             mt_set_gpio_out(I2C_SDA_GPIO, GPIO_OUT_ZERO)
#define I2C_SCL_HIGH            mt_set_gpio_out(I2C_SCL_GPIO, GPIO_OUT_ONE)
#define I2C_SCL_LOW             mt_set_gpio_out(I2C_SCL_GPIO, GPIO_OUT_ZERO)
#define I2C_SDA_READ           mt_get_gpio_in(I2C_SDA_GPIO)

#define NEW_I2C_TIMING              1


void AW9523_i2c_initial(void)
{
#if NEW_I2C_TIMING
        I2C_SDA_MODE;
	I2C_SCL_MODE;
	I2C_SDA_OUTPUT;
	I2C_SCL_OUTPUT;

   mt_set_gpio_pull_enable(I2C_SDA_GPIO, GPIO_PULL_DISABLE); 
   mt_set_gpio_pull_enable(I2C_SCL_GPIO, GPIO_PULL_DISABLE); 

	I2C_SDA_HIGH;
	I2C_SCL_HIGH;
#else
	GPIO_ModeSetup(I2C_SCL_GPIO, 0);
	GPIO_InitIO(1, I2C_SCL_GPIO);
	mt_set_gpio_pull_enable(I2C_SCL_GPIO, GPIO_PULL_DISABLE);
	GPIO_WriteIO(1, I2C_SCL_GPIO);
	
	GPIO_ModeSetup(I2C_SDA_GPIO, 0);
	GPIO_InitIO(1,I2C_SDA_GPIO);
	mt_set_gpio_pull_enable(I2C_SDA_GPIO, GPIO_PULL_DISABLE);
	GPIO_WriteIO(0, I2C_SDA_GPIO);
	AW9523_delay_1us(5);
	GPIO_WriteIO(1, I2C_SDA_GPIO);//Îª±ÜÃâi2c initial Ê±²úÉú\u017díÎóµÄ×\u017dÌ¬£¬ÏÈ·¢Ò»\u017eöÍ£Ö¹Ìõ\u0152þ
#endif
}

void AW9523_Hw_reset(void)
{   
	//kpled_ctrl_open(1);           // Kpled on , then reset be pulled down
  mt_set_gpio_mode(GPIO30, GPIO_MODE_00);    
  mt_set_gpio_dir(GPIO30, GPIO_DIR_OUT);
   mt_set_gpio_out(GPIO30, GPIO_OUT_ZERO); 

	AW9523_delay_1us(200); //\u017e\u017dÎ»ÐÅºÅÎªµÍµçÆ\u0153µÄ³ÖÐøÊ±\u0152ä±ØÐëÖÁÉÙ20us²ÅÄÜÕý³£\u017e\u017dÎ»  
	//GPIO_WriteIO(1, AW9523_RESET_PIN); 
	//kpled_ctrl_open(0);            // Kpled off, then reset be pulled up by VIO18
    mt_set_gpio_mode(GPIO30, GPIO_MODE_00);    
    mt_set_gpio_dir(GPIO30, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO30, GPIO_OUT_ONE); 
	//AW9523_delay_1us(30); 
	mdelay(20);

  
}

static void AW9523_i2c_start(void)
{
#if NEW_I2C_TIMING
        I2C_SDA_MODE;
	I2C_SCL_MODE;
	I2C_SDA_OUTPUT;
	I2C_SCL_OUTPUT;
        //spin_lock_irqsave(&gpio_i2c_spinLock, flags_spin);
	I2C_SDA_HIGH;
	udelay(5);  //udelay(10); //udelay(100); //mdelay(1); 
	I2C_SCL_HIGH;
	udelay(5);  //udelay(10); //udelay(100); //mdelay(1); 
	I2C_SDA_LOW;
	udelay(5);  //udelay(10); //udelay(100); //mdelay(1); 
#else
	GPIO_InitIO(1,I2C_SDA_GPIO);
	GPIO_InitIO(1,I2C_SCL_GPIO);
	
	GPIO_WriteIO(1, I2C_SDA_GPIO);
	GPIO_WriteIO(1, I2C_SCL_GPIO);
	AW9523_delay_1us(2);
	GPIO_WriteIO(0, I2C_SDA_GPIO);
	AW9523_delay_1us(2);	
	GPIO_WriteIO(0, I2C_SCL_GPIO);
	AW9523_delay_1us(2);	
#endif 
}

static void AW9523_i2c_stop(void)
{
#if NEW_I2C_TIMING
	I2C_SDA_OUTPUT;
	I2C_SCL_OUTPUT;
	//spin_lock_irqsave(&gpio_i2c_spinLock, flags_spin);
	I2C_SCL_LOW;   // test @20131009
	udelay(5);  //udelay(10); //udelay(100); //mdelay(1);     // 20131010
	I2C_SDA_LOW;
	udelay(5);  //udelay(10); //udelay(100); //mdelay(1); 
	I2C_SCL_HIGH;
	udelay(5);  //udelay(10); //udelay(100); //mdelay(1); 
	I2C_SDA_HIGH;
        udelay(5);  //udelay(10); //udelay(100); //mdelay(1); 
#else
	GPIO_InitIO(1,I2C_SDA_GPIO);
	GPIO_InitIO(1,I2C_SCL_GPIO);	
	GPIO_WriteIO(0, I2C_SCL_GPIO);
	AW9523_delay_1us(2);
	GPIO_WriteIO(0, I2C_SDA_GPIO);
	GPIO_WriteIO(1, I2C_SCL_GPIO);
	AW9523_delay_1us(2);
	GPIO_WriteIO(1, I2C_SDA_GPIO);
#endif
}

static char AW9523_i2c_write_byte(unsigned char data)
{
#if NEW_I2C_TIMING

        char  i = 0;
        char times = 0;
	//unsigned long flags_spin;

        //spin_lock_irqsave(&gpio_i2c_spinLock, flags_spin);
        I2C_SCL_LOW;
	udelay(5);  //udelay(10); //udelay(100); //mdelay(1); 
	I2C_SDA_OUTPUT;

        for(i=0;i<8;i++)
        {
            if((data<<i)&0x80)
                     I2C_SDA_HIGH;
            else
                     I2C_SDA_LOW;               
            udelay(5);  //udelay(10); //udelay(100); //mdelay(1); 
            I2C_SCL_HIGH;           
            udelay(5);  //udelay(10); //udelay(100); //mdelay(1); 
            I2C_SCL_LOW;
	    udelay(5);  //udelay(10); //udelay(100); //mdelay(1); 
        }        

        I2C_SDA_INPUT;
        udelay(5);  //udelay(10); //udelay(100); //mdelay(1);
	I2C_SCL_HIGH;
	udelay(5);  //udelay(10); //udelay(100); //mdelay(1); 
	//spin_unlock_irqrestore(&gpio_i2c_spinLock, flags_spin);
	while  (I2C_SDA_READ==1)
	{
	     udelay(5);  //udelay(10);//return 1;
        times++;
        if (times==10)
          break;
	}
	
	I2C_SCL_LOW;
	udelay(5);  //udelay(10); //udelay(100); //mdelay(1); 
	
        return 1;

#else

	u8 i;
	char ack;
	
	GPIO_InitIO(1,I2C_SDA_GPIO);
	for	(i=0; i<8; i++)
	{
		if (data & 0x80)
			GPIO_WriteIO(1,I2C_SDA_GPIO);
		else
			GPIO_WriteIO(0,I2C_SDA_GPIO);
		data <<= 1;
		AW9523_delay_1us(1);
		GPIO_WriteIO(1,I2C_SCL_GPIO);
		AW9523_delay_1us(1);
		GPIO_WriteIO(0,I2C_SCL_GPIO);
		AW9523_delay_1us(1);
	}
	GPIO_InitIO(0,I2C_SDA_GPIO);
	AW9523_delay_1us(6);
	GPIO_WriteIO(1,I2C_SCL_GPIO);
	ack = GPIO_ReadIO(I2C_SDA_GPIO); /// ack   
	AW9523_delay_1us(1);
	GPIO_WriteIO(0,I2C_SCL_GPIO);
	return ack;	
#endif
}

static u8 AW9523_i2c_read_byte(void)
{
#if NEW_I2C_TIMING
        u8 rec_byte = 0x00;
	u8 i = 0;

	//I2C_SCL_LOW;
	//mdelay(1); 
	
	for (i=0; i<8; i++)
	{
	      rec_byte <<=1;
	     //I2C_SCL_LOW;
             //mdelay(1);
	     I2C_SCL_HIGH;
	     udelay(5);  //udelay(10); //udelay(100); //mdelay(1);

	     rec_byte |= I2C_SDA_READ;
	     I2C_SCL_LOW;
             udelay(5);  //udelay(10); //udelay(100); //mdelay(1);
	}

         I2C_SDA_OUTPUT;
	 I2C_SDA_HIGH;
	 udelay(5);  //udelay(10); //udelay(100); //mdelay(1);

	 I2C_SCL_HIGH;  
         udelay(5);  //udelay(10); //udelay(100); //mdelay(1);
	 I2C_SCL_LOW;  
         udelay(5);  //udelay(10); //udelay(100); //mdelay(1);	
         
	 return rec_byte;
#else
	u8 i;
	u8 bData;
	GPIO_InitIO(0, I2C_SDA_GPIO);
	//Êý\u0178Ý¶Á³ö
	  bData = 0x00;
	  for (i=0;i<8;i++) {
		  bData <<= 1;
		  AW9523_delay_1us(4);
		  GPIO_WriteIO(1, I2C_SCL_GPIO);
		  if (GPIO_ReadIO(I2C_SDA_GPIO)) {
			  bData |= 0x01;
		  } else {
			  bData &= 0xfe;
		  }
		  AW9523_delay_1us(1);
		   GPIO_WriteIO(0, I2C_SCL_GPIO);
	  }
	  AW9523_delay_1us(1);
	  GPIO_WriteIO(1, I2C_SCL_GPIO);	
	  AW9523_delay_1us(1);
	  GPIO_WriteIO(0, I2C_SCL_GPIO);
	  return bData;
#endif
}

static char AW9523_i2c_write_reg_org(unsigned char reg,unsigned char data)
{
#if NEW_I2C_TIMING
        AW9523_i2c_start();
	AW9523_i2c_write_byte(LED_SLAVE_ADDR);
	AW9523_i2c_write_byte(reg);
	AW9523_i2c_write_byte(data);
	AW9523_i2c_stop();
	return 1;
#else

	char ack=0;
	AW9523_i2c_start();	
	ack|=AW9523_i2c_write_byte(LED_SLAVE_ADDR); 	//write device address
	ack|=AW9523_i2c_write_byte(reg);  	// reg address
	ack|= AW9523_i2c_write_byte(data);	// data
	AW9523_i2c_stop();
	return ack;
#endif

}

static char AW9523_i2c_write_reg(unsigned char reg,unsigned char data)
{
#if NEW_I2C_TIMING
        AW9523_i2c_start();
	AW9523_i2c_write_byte(LED_SLAVE_ADDR);
	AW9523_i2c_write_byte(reg);
	AW9523_i2c_write_byte(data);
	AW9523_i2c_stop();
	return 1;
#else
	char ack=0;
	char i;
	for (i=0;i<AW9523_I2C_MAX_LOOP;i++)
		{
			ack=AW9523_i2c_write_reg_org(reg,data);
			if(ack==0) // ack success
				break;
		}
	return ack;
#endif
}

u8 AW9523_i2c_read_reg(u8 regaddr) 
{
#if NEW_I2C_TIMING
        u8 read_byte = 0;

        AW9523_i2c_start();
        AW9523_i2c_write_byte(LED_SLAVE_ADDR);
	AW9523_i2c_write_byte(regaddr);
	AW9523_i2c_stop();             /////////////////////
	AW9523_i2c_start();            //restart signal
	AW9523_i2c_write_byte(LED_SLAVE_ADDR|0x01);
	read_byte = AW9523_i2c_read_byte();
	AW9523_i2c_stop();
	return read_byte;
#else
	u8 mask,i, bData;
	char ack1,ack2,ack3;
	u8 i2caddr;
	for (i=0;i<AW9523_I2C_MAX_LOOP;i++)
		{
			AW9523_i2c_start();	
			ack1=AW9523_i2c_write_byte(LED_SLAVE_ADDR); 	//write device address
			ack2=AW9523_i2c_write_byte(regaddr);  	// reg address
			AW9523_i2c_stop();
			AW9523_i2c_start();	
			ack3=AW9523_i2c_write_byte((LED_SLAVE_ADDR|0x01)); 	//write device address
			if((ack1 || ack2 || ack3)==0) // ack success
				break;
		}
	bData=AW9523_i2c_read_byte();
	AW9523_i2c_stop();
    	return bData;
#endif

}

static void AW9523_SDA_Change(void) 
{
	u8 SDA_index = 0;
	GPIO_WriteIO(0,I2C_SDA_GPIO);
	AW9523_delay_1us(80);
	for (SDA_index=0;SDA_index<50;SDA_index++)
		{			
			GPIO_InitIO(0, I2C_SDA_GPIO);
			AW9523_delay_1us(420);  //SDAµÄÖÜÆÚ£¬\u017eßµçÆ\u0153420us£¬ ÔÚ\u017e÷Æ\u0153Ì\u0161ÏÂÇë\u017eù\u0178ÝÖ÷ÆµÀ\u017dµ÷Õû£¬±£³ÖÒ»ÖÂ
			GPIO_InitIO(1, I2C_SDA_GPIO);//SDAµÄÖÜÆÚ£¬µÍµçÆ\u015380us£¬ ÔÚ\u017e÷Æ\u0153Ì\u0161ÏÂÇë\u017eù\u0178ÝÖ÷ÆµÀ\u017dµ÷Õû£¬±£³ÖÒ»ÖÂ
			AW9523_delay_1us(80); //SDAµÍµçÆ\u015380us
		}
	GPIO_InitIO(1, I2C_SDA_GPIO);
	GPIO_WriteIO(1,I2C_SDA_GPIO);
	AW9523_delay_1us(420);  
}

char AW9523_POWER_ON(void)
{    // AW9523 POWER-ON£¬ Çë¿Í»§²»Òª\u017eÄ¶¯\u017dËº¯Êý
     // ÔÚaw9523_init()ÖÐ£¬ÏÈ\u0153øÐÐPOWER-ON£¬ÔÙ\u0153øÐÐ¿Í»§×ÔÉíµÄÏà¹Ø²Ù×÷
	char ack=0;
	u8 count=0;
	//hwPowerOn(MT65XX_POWER_LDO_VGP2, VOL_3300, "TESTKPD");
	AW9523_i2c_initial();
	AW9523_Hw_reset();
	  while(count++ < 120) 		//
			  {
				  if(AW9523_i2c_write_reg_org(0x55,0x55))   //ÅÐ¶ÏÓ\u0160\u017dðÄ£Ê\u0153¡£0x55Îª¿Õ\u0152Ä\u017dæÆ÷£¬²»»á¶Ô¿Í»§Ó\u0160ÓÃ²úÉúÓ°Ïì
					  {
					  AW9523_SDA_Change();
					  continue;
					  }
				  if(AW9523_i2c_write_reg_org(0xaa,0xaa))
					  {
					  AW9523_SDA_Change();
					  continue;
					  }   
				  if(AW9523_i2c_write_reg_org(0x55,0xaa))
					  {
					  AW9523_SDA_Change();
					  continue;
					  } 			  
				  if(AW9523_i2c_write_reg_org(0xaa,0x55)) 
					  {
					  AW9523_SDA_Change();
					  continue;
					  } 			  				  

				  break;				  
			  }
	ack |= AW9523_i2c_write_reg_org(0x55,0x55); 
	ack |= AW9523_i2c_write_reg_org(0xaa,0xaa); 
	//SCI_TRACE_LOW("----AW9523 POWER ON -----end =%d\r\n", count);
	return ack;
}

/*=======================================i2c driver end =================================*/

/*===========================aw9523\u0152Ä\u017dæÆ÷µÄ²Ù×÷\u0153Ó¿Ú begin ================================*/
void aw9523_p0_p1_in_out_setting(void)  /* ÉèÖÃ p0 p1 ÊäÈëÊä³ö×\u017dÌ¬*/
{
    P0_IN_OUT_STATE=0xFF;
    AW9523_i2c_write_reg(0x04,P0_IN_OUT_STATE);//ÉèÖÃËùÓÐP0_\u0153ÅÎªÊäÈë×\u017dÌ¬
    P1_IN_OUT_STATE=0x0;
    AW9523_i2c_write_reg(0x05,P1_IN_OUT_STATE);//ÉèÖÃËùÓÐP1_\u0153ÅÎªÊä³ö×\u017dÌ¬
}

void aw9523_p0_p1_interrupt_setting(void)   /* ÉèÖÃ p0 p1 ÔÊÐíÖÐ¶Ï×\u017dÌ¬*/
{
    u8 i=0;
    P0_INT_STATE=0x0; 
    for (i=0;i<X_NUM;i++) {
        P0_INT_STATE=P0_INT_STATE|(1<<COL[i]);//\u017eù\u0178Ý°\u017d\u0152üÅÅ²\u0152À\u017dÉèÖÃÖÐ¶Ï
    }
    P0_INT_STATE=~P0_INT_STATE;
    AW9523_i2c_write_reg(0x06,P0_INT_STATE);//ºÍ°\u017d\u0152üÁÐ¶ÔÓ\u0160µÄP0_\u0153ÅÔÊÐíÖÐ¶Ï

    P1_INT_STATE=0xFF;
    AW9523_i2c_write_reg(0x07,P1_INT_STATE);//p1\u0153Å¶\u0152²»ÄÜÖÐ¶Ï

}

void aw9523_p0_int_restore(void)
{
    AW9523_i2c_write_reg(0x06,P0_INT_STATE);

}

void aw9523_P1_int_restore()
{
    AW9523_i2c_write_reg(0x07,P1_INT_STATE);
}

void aw9523_p0_int_disable()
{
    AW9523_i2c_write_reg(0x06, 0xff);
}

void aw9523_p1_int_disable()
{
    AW9523_i2c_write_reg(0x07, 0xff);
}

u8 aw9523_get_p0(void)  
{
    //AW9523_i2c_read_reg(0x01);       //just for test 20131129
    return AW9523_i2c_read_reg(0x00);
}

u8 aw9523_get_p1(void)
{
    return AW9523_i2c_read_reg(0x01);
}

void aw9523_set_p0(u8 data)
{
    AW9523_i2c_write_reg(0x02,data);
}

void aw9523_set_p1(u8 data)
{
    AW9523_i2c_write_reg(0x03,data);
}

void RE_P0_WRITE(u8 P0,u8 data) 
{
	u8 tmp_value;
	tmp_value = AW9523_i2c_read_reg(0x02);
	if(data==1)
	{
		AW9523_i2c_write_reg(0x02,(tmp_value|(1<<P0)));
	}
	else
	{
		AW9523_i2c_write_reg(0x02,(tmp_value&(~(1<<P0))));
	}	
}

void RE_P1_WRITE(u8 P1,u8 data)
{
	u8 tmp_value;
	tmp_value = AW9523_i2c_read_reg(0x03);
	if(data==1)
	{
		AW9523_i2c_write_reg(0x03,(tmp_value|(1<<P1)));
	}
	else
	{
		AW9523_i2c_write_reg(0x03,(tmp_value&(~(1<<P1))));
	}	
}

void aw9523_keylight_open(u8 enable)
{
        if (enable)
        {
              AW9523_i2c_write_reg(0x20, 0x40);       // led current P1_0
              AW9523_i2c_write_reg(0x21, 0x40);       // led current P1_1
              
              AW9523_i2c_write_reg(0x13, 0xFF);       // P1_0, P1_1 working mode
        }
	else
	{
              AW9523_i2c_write_reg(0x20, 0x00);       // led current P1_0
              AW9523_i2c_write_reg(0x21, 0x00);       // led current P1_1

              AW9523_i2c_write_reg(0x13, 0xFF);       // P1_0, P1_1 working mode			  
	}
}

void aw9523_test(void)
{
// i2c_initial();  
	// AW9523_Hw_reset();
	// AW9523_i2c_initial();
   printk("\naw9523_test_entry=\r\n");
   printk("\naw9523_i2c_read_reg_0x00=0x%x\r\n",AW9523_i2c_read_reg(0x00));
   printk("\naw9523_i2c_read_reg_0x01=0x%x\r\n",AW9523_i2c_read_reg(0x01)); 
   printk("\naw9523_i2c_read_reg_0x02=0x%x\r\n",AW9523_i2c_read_reg(0x02));
   printk("\naw9523_i2c_read_reg_0x03=0x%x\r\n",AW9523_i2c_read_reg(0x03)); 
   printk("\naw9523_i2c_read_reg_0x04=0x%x\r\n",AW9523_i2c_read_reg(0x04));
   printk("\naw9523_i2c_read_reg_0x05=0x%x\r\n",AW9523_i2c_read_reg(0x05)); 
   printk("\naw9523_i2c_read_reg_0x06=0x%x\r\n",AW9523_i2c_read_reg(0x06));
   printk("\naw9523_i2c_read_reg_0x07=0x%x\r\n",AW9523_i2c_read_reg(0x07));    
   printk("\naw9523_i2c_read_reg_0x10=0x%x\r\n",AW9523_i2c_read_reg(0x10));    
   printk("\naw9523_i2c_read_reg_0x11=0x%x\r\n",AW9523_i2c_read_reg(0x11));    
   printk("\naw9523_i2c_read_reg_0x12=0x%x\r\n",AW9523_i2c_read_reg(0x12));
   printk("\naw9523_i2c_read_reg_0x13=0x%x\r\n",AW9523_i2c_read_reg(0x13));   
}

void aw9523_init()
{
	//AW9523_POWER_ON();          // test new i2c timing , delete this code 
    /*  hwPowerOn(MT65XX_POWER_LDO_VGP2, VOL_2800, "TESTKPD");	

                     pmic_config_interface( (kal_uint32)(DIGLDO_CON29),
                         5,
                           (kal_uint32)(PMIC_RG_VGP2_VOSEL_MASK),
                           (kal_uint32)(PMIC_RG_VGP2_VOSEL_SHIFT));*/
        AW9523_i2c_initial();
        AW9523_Hw_reset();  
	  
#if 1
        AW9523_i2c_write_reg(0x7F, 0x00);   // sw reset
        mdelay(10);

        aw9523_keylight_open(0);        // close P1_0, P1_1 mos fet
		
        aw9523_p0_int_disable(); 
        aw9523_p0_p1_in_out_setting();
        aw9523_p0_p1_interrupt_setting();
        aw9523_set_p1(P1_VALUE);
        AW9523_i2c_read_reg(0x00); 
        AW9523_i2c_read_reg(0x01);    
#endif
}

void Set_P0_X_AND_P1_Y(void) 
{
    u8 i=0;
    u8 temp=0;
    for (i=0;i<X_NUM;i++) {
        temp=temp|(1<<COL[i]);
  }
    //SCI_TRACE_LOW("temp=%x\r\n",temp);

    for (i=0;i<X_NUM;i++) {
        P0_X[i]=temp&(~(1<<COL[i]));
        //SCI_TRACE_LOW("P0_X[%d]=%x\r\n",i,P0_X[i]);
    }

    temp=0;
    for (i=0;i<Y_NUM;i++) {
        temp=temp|(1<<Line[i]);
    }
    //SCI_TRACE_LOW("temp=%x\r\n",temp);
    for (i=0;i<Y_NUM;i++) {
        P1_Y[i]=temp&(~(1<<Line[i]));
        //SCI_TRACE_LOW("P1_Y[%d]=%x\r\n",i,P1_Y[i]);
    }
   
   	for(i=0;i<8;i++)
	{
		if(P0_kbd_used[i]==1)
		{
			P0_kbd_used_temp|=1<<i;
		}
	}
    //SCI_TRACE_LOW("P0_kbd_used_temp=%x\r\n",P0_kbd_used_temp);
}

u8 keyboard_get_press_key(void)
{
    u8 x=0xFF,y=0XFF;
    u8 i=0,j=0,k=0;
    u8 get_p0=0xff; 
	
    get_p0=aw9523_get_p0();
    i=(get_p0)|(~(P0_kbd_used_temp));  //Î\u017dÓÃ×ö\u0152üÅÌÉ\u0161ÃèµÄP0¿Ú,Æä¶Á³öµÄÖµ±»ÆÁ±Î
    //i=get_p0 & P0_kbd_used_temp;  //Î\u017dÓÃ×ö\u0152üÅÌÉ\u0161ÃèµÄP0¿Ú,Æä¶Á³öµÄÖµ±»ÆÁ±Î
    printk("===aw9523_get_p0()===%x\,%xr\n",get_p0,i);
    if (i==0xff) 
    {
    	//  SCI_TRACE_LOW("------get_key=0xff----\r\n");	
        return 0xFF;
    }
    else 
    {
    //SCI_TRACE_LOW("===aw9523_get_p0()===%x\r\n",i);    
       if(KeyBoardKey_State==KEY_STATE_PRESSED || KeyBoardKey_State==KEY_STATE_LONGPRESS)
	{
        	//Èç¹ûÖ®Ç°ÊÇ\u017d\u0160ÓÚÓÐ°\u017d\u0152üµÄ×\u017dÌ¬,ÔòÖ±\u0153ÓÈ¥\u0152ì²âÕâ\u017eö°\u017d\u0152üÊÇ·ñÊÍ·Å
     	 printk("------aw9523 press or longpress entry ,pre_x=%d,pre_y=%d ----\r\n",pre_x,pre_y); 
     	 	AW9523_i2c_write_reg(0x05,P1_Y[pre_y]);
     	 	get_p0=aw9523_get_p0(); 
          //i=get_p0 & P0_kbd_used_temp;
              if  ((get_p0&(1<<COL[pre_x])) == 0)
		{
         	   AW9523_i2c_write_reg(0x05,P1_VALUE); 
         	   return aw9523_key[pre_y][pre_x];
		}
              else 
         	{
         	   AW9523_i2c_write_reg(0x05,P1_VALUE);
         	   return 0xFF;
         	}
        } 
        else 
	 {
   	//\u0152ì²â°\u017d\u0152üµÄ³ÌÐòÈë¿Ú
           for (j=0;j<X_NUM;j++) 
	    {
    	        if((i&(1<<COL[j]))==0)
		 {
        //if (i==P0_X[j]) {
            //±\u0178ÀýÖÐP0_X[0:7]={0xfe,0xfd,0xfb,0xf7,0xef,0xdf,0xbf,0x7f},
            //Èô¶Á³öµÄP0_ÖµÄ³Ò»Î»Îª0£¬ÔòÅÐ¶Ï³ö°\u017dÏÂµÄÊÇÕâÒ»ÁÐ
                  x=j;
                  break;
               }
           }
		   
           if (x==0xFF) 
	    {
       // SCI_TRACE_LOW("------get_key=0xff----\r\n");	
              return 0xFF;
           }
		   
	    for (j=0;j<Y_NUM;j++) 
	    {
           // aw9523_set_p1(P1_Y[j]); 
             AW9523_i2c_write_reg(0x05,P1_Y[j]);
            /*ÂÖÑ¯É\u0161Ãè¡£ÒÀ\u017dÎ\u0153«Ä³Ò»ÐÐP1ÖµÖÃ0£¬ÆäËûÐÐÖÃ1»ò\u017eß×è£¬Ã¿É\u0161ÃèÒ»ÐÐÊ±£¬¶\u0152¶ÁÒ»\u017dÎP0Öµ
            µ±É\u0161Ãèµ\u0153Ä³Ò»ÐÐÊ±£¬Èô¶Á³öµÄP0ÖµÓëÖ®Ç°ÅÐ¶Ïµ\u0153µÄP0ÖµÏàµÈ£¬ÔòÅÐ¶Ï³ö°\u017dÏÂµÄÊÇÕâÒ»ÐÐ */
             get_p0=aw9523_get_p0(); 
             k=(get_p0)|(~(P0_kbd_used_temp));           
          // if (k==P0_X[x]) { 
             if ((k&(1<<COL[x]))==0)
	      {	
                y=j;
                break;
             }
           }
        //aw9523_set_p1(P1_VALUE);
           AW9523_i2c_write_reg(0x05,P1_VALUE);
          // get_p0=aw9523_get_p0(); 
          // k=get_p0 & P0_kbd_used_temp; 
        printk("------aw9523   ---get_key: x=%d,y=%d----\r\n",x,y);
           if (x!=0xFF && y!=0xFF ) 
	    {
        	  pre_x=x;
        	  pre_y=y;
                return aw9523_key[y][x];
           }
	    else 
	    {
                return 0xFF;
           }
     }}
} 

static void kpd_aw9523_handler(unsigned long data)
{
	//bool pressed;
	u8 old_state = kpd_aw9523_state;
        //static u8 flag = 1;

        //mt65xx_eint_set_polarity(AW9523_EINT_NO, old_state);
        aw9523_p0_int_disable();
        disable_irq(MT_KP_IRQ_ID);
        mt_eint_mask(AW9523_EINT_NO);  // mt65xx_eint_mask(AW9523_EINT_NO);
	//aw9523_p0_int_disable();
        printk("\n<<<<  xxx==kpd_aw9523_handler state=%d\n", kpd_aw9523_state);
        //mt65xx_eint_unmask(AW9523_EINT_NO);
	kpd_aw9523_state = !kpd_aw9523_state;

		#if 0
         if (kpd_aw9523_state!=KPD_AW9523_SWITCH_POLARITY)
         {

	    mt65xx_eint_set_polarity(AW9523_EINT_NO, old_state);
	    enable_irq(MT_KP_IRQ_ID);	
	    mt65xx_eint_unmask(AW9523_EINT_NO);

            aw9523_p0_int_restore();   

	    return;
         }
		 #endif
		
        printk("\n<<<<  xxx===key =%d  pressed=======\r\n",keyboard_get_press_key());
        KeyBoard_Key=keyboard_get_press_key();
        printk("\n<<<<  xxx=aw9523_handler==key =%d  pressed,old_state=%d==\r\n",KeyBoard_Key, old_state);

///*		
        if (KeyBoardKey_State==KEY_STATE_NULL||KeyBoardKey_State==KEY_STATE_RELEASED) 
        {
            if (KeyBoard_Key!=0xFF) 
	    {
               KeyBoardKey_State=KEY_STATE_PRESSED;

               KeyBoard_Key_Previous=KeyBoard_Key;
	       input_report_key(kpd_input_dev, KeyBoard_Key, 1);
	       input_sync(kpd_input_dev);

            }
       } 
       else if (KeyBoardKey_State==KEY_STATE_PRESSED) 
       {
            if (KeyBoard_Key != KeyBoard_Key_Previous) 
	    {
                KeyBoardKey_State=KEY_STATE_RELEASED;
		input_report_key(kpd_input_dev, KeyBoard_Key_Previous, 0);
	        input_sync(kpd_input_dev);
             }
        } 
  //      */

        AW9523_i2c_write_reg(0x05,P1_VALUE);

	enable_irq(MT_KP_IRQ_ID);	
	mt_eint_unmask(AW9523_EINT_NO);  // mt65xx_eint_unmask(AW9523_EINT_NO);

        aw9523_p0_int_restore();  
		
		AW9523_i2c_read_reg(0x00);
		AW9523_i2c_read_reg(0x01);

        //mt65xx_eint_set_polarity(AW9523_EINT_NO, old_state);
}

static void kpd_aw9523_eint_handler(void)
{
        //printk("\nxxx==kpd_halldet_eint_handler===\n");
	tasklet_schedule(&kpd_aw9523_tasklet);
}
#endif

/*********************************************************************/

/*********************************************************************/
#if KPD_PWRKEY_USE_PMIC
void kpd_pwrkey_pmic_handler(unsigned long pressed)
{
	printk(KPD_SAY "Power Key generate, pressed=%ld\n", pressed);
	if(!kpd_input_dev) {
		printk("KPD input device not ready\n");
		return;
	}
	kpd_pmic_pwrkey_hal(pressed);
}
#endif


void kpd_pmic_rstkey_handler(unsigned long pressed)
{
	printk(KPD_SAY "PMIC reset Key generate, pressed=%ld\n", pressed);
	if(!kpd_input_dev) {
		printk("KPD input device not ready\n");
		return;
	}
	kpd_pmic_rstkey_hal(pressed);
#ifdef KPD_PMIC_RSTKEY_MAP
	kpd_aee_handler(KPD_PMIC_RSTKEY_MAP, pressed);
#endif
}
/*********************************************************************/

/*********************************************************************/
static void kpd_keymap_handler(unsigned long data)
{
	int i, j;
	bool pressed;
	u16 new_state[KPD_NUM_MEMS], change, mask;
	u16 hw_keycode, linux_keycode;
	kpd_get_keymap_state(new_state);

	for (i = 0; i < KPD_NUM_MEMS; i++) {
		change = new_state[i] ^ kpd_keymap_state[i];
		if (!change)
			continue;

		for (j = 0; j < 16; j++) {
			mask = 1U << j;
			if (!(change & mask))
				continue;

			hw_keycode = (i << 4) + j;
			/* bit is 1: not pressed, 0: pressed */
			pressed = !(new_state[i] & mask);
			if (kpd_show_hw_keycode) {
				printk(KPD_SAY "(%s) HW keycode = %u\n",
				       pressed ? "pressed" : "released",
				       hw_keycode);
			}
			BUG_ON(hw_keycode >= KPD_NUM_KEYS);
			linux_keycode = kpd_keymap[hw_keycode];			
			if (unlikely(linux_keycode == 0)) {
				kpd_print("Linux keycode = 0\n");
				continue;
			}		
			kpd_aee_handler(linux_keycode, pressed);
			
			kpd_backlight_handler(pressed, linux_keycode);
			input_report_key(kpd_input_dev, linux_keycode, pressed);
			input_sync(kpd_input_dev);
			kpd_print("report Linux keycode = %u\n", linux_keycode);
		}
	}
	
	memcpy(kpd_keymap_state, new_state, sizeof(new_state));
	kpd_print("save new keymap state\n");
	enable_irq(MT_KP_IRQ_ID);
}

static irqreturn_t kpd_irq_handler(int irq, void *dev_id)
{
	/* use _nosync to avoid deadlock */
	disable_irq_nosync(MT_KP_IRQ_ID);
	tasklet_schedule(&kpd_keymap_tasklet);
	return IRQ_HANDLED;
}
/*********************************************************************/

/*****************************************************************************************/
long kpd_dev_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	//void __user *uarg = (void __user *)arg;

	switch (cmd) {
#if KPD_AUTOTEST
	case PRESS_OK_KEY://KPD_AUTOTEST disable auto test setting to resolve CR ALPS00464496
		if(test_bit(KEY_OK, kpd_input_dev->keybit)){
			printk("[AUTOTEST] PRESS OK KEY!!\n");
			input_report_key(kpd_input_dev, KEY_OK, 1);
			input_sync(kpd_input_dev);
		}else{
			printk("[AUTOTEST] Not Support OK KEY!!\n");
		}
		break;
	case RELEASE_OK_KEY:
		if(test_bit(KEY_OK, kpd_input_dev->keybit)){
			printk("[AUTOTEST] RELEASE OK KEY!!\n");
			input_report_key(kpd_input_dev, KEY_OK, 0);
			input_sync(kpd_input_dev);
		}else{
			printk("[AUTOTEST] Not Support OK KEY!!\n");
		}
		break;
	case PRESS_MENU_KEY:
		if(test_bit(KEY_MENU, kpd_input_dev->keybit)){
			printk("[AUTOTEST] PRESS MENU KEY!!\n");
			input_report_key(kpd_input_dev, KEY_MENU, 1);
			input_sync(kpd_input_dev);
		}else{
			printk("[AUTOTEST] Not Support MENU KEY!!\n");
		}
		break;
	case RELEASE_MENU_KEY:
		if(test_bit(KEY_MENU, kpd_input_dev->keybit)){
			printk("[AUTOTEST] RELEASE MENU KEY!!\n");
			input_report_key(kpd_input_dev, KEY_MENU, 0);
			input_sync(kpd_input_dev);
		}else{
			printk("[AUTOTEST] Not Support MENU KEY!!\n");
		}

		break;
	case PRESS_UP_KEY:
		if(test_bit(KEY_UP, kpd_input_dev->keybit)){
			printk("[AUTOTEST] PRESS UP KEY!!\n");
			input_report_key(kpd_input_dev, KEY_UP, 1);
			input_sync(kpd_input_dev);
		}else{
			printk("[AUTOTEST] Not Support UP KEY!!\n");
		}
		break;
	case RELEASE_UP_KEY:
		if(test_bit(KEY_UP, kpd_input_dev->keybit)){
			printk("[AUTOTEST] RELEASE UP KEY!!\n");
			input_report_key(kpd_input_dev, KEY_UP, 0);
			input_sync(kpd_input_dev);
		}else{
			printk("[AUTOTEST] Not Support UP KEY!!\n");
		}
		break;
	case PRESS_DOWN_KEY:
		if(test_bit(KEY_DOWN, kpd_input_dev->keybit)){
			printk("[AUTOTEST] PRESS DOWN KEY!!\n");
			input_report_key(kpd_input_dev, KEY_DOWN, 1);
			input_sync(kpd_input_dev);
		}else{
			printk("[AUTOTEST] Not Support DOWN KEY!!\n");
		}
		break;
	case RELEASE_DOWN_KEY:
		if(test_bit(KEY_DOWN, kpd_input_dev->keybit)){
			printk("[AUTOTEST] RELEASE DOWN KEY!!\n");
			input_report_key(kpd_input_dev, KEY_DOWN, 0);
			input_sync(kpd_input_dev);
		}else{
			printk("[AUTOTEST] Not Support DOWN KEY!!\n");
		}
		break;
	case PRESS_LEFT_KEY:
		if(test_bit(KEY_LEFT, kpd_input_dev->keybit)){
			printk("[AUTOTEST] PRESS LEFT KEY!!\n");
			input_report_key(kpd_input_dev, KEY_LEFT, 1);
			input_sync(kpd_input_dev);
		}else{
			printk("[AUTOTEST] Not Support LEFT KEY!!\n");
		}
		break;		
	case RELEASE_LEFT_KEY:
		if(test_bit(KEY_LEFT, kpd_input_dev->keybit)){
			printk("[AUTOTEST] RELEASE LEFT KEY!!\n");
			input_report_key(kpd_input_dev, KEY_LEFT, 0);
			input_sync(kpd_input_dev);
		}else{
			printk("[AUTOTEST] Not Support LEFT KEY!!\n");
		}
		break;

	case PRESS_RIGHT_KEY:
		if(test_bit(KEY_RIGHT, kpd_input_dev->keybit)){
			printk("[AUTOTEST] PRESS RIGHT KEY!!\n");
			input_report_key(kpd_input_dev, KEY_RIGHT, 1);
			input_sync(kpd_input_dev);
		}else{
			printk("[AUTOTEST] Not Support RIGHT KEY!!\n");
		}
		break;
	case RELEASE_RIGHT_KEY:
		if(test_bit(KEY_RIGHT, kpd_input_dev->keybit)){
			printk("[AUTOTEST] RELEASE RIGHT KEY!!\n");
			input_report_key(kpd_input_dev, KEY_RIGHT, 0);
			input_sync(kpd_input_dev);
		}else{
			printk("[AUTOTEST] Not Support RIGHT KEY!!\n");
		}
		break;
	case PRESS_HOME_KEY:
		if(test_bit(KEY_HOME, kpd_input_dev->keybit)){
			printk("[AUTOTEST] PRESS HOME KEY!!\n");
			input_report_key(kpd_input_dev, KEY_HOME, 1);
			input_sync(kpd_input_dev);
		}else{
			printk("[AUTOTEST] Not Support HOME KEY!!\n");
		}
		break;
	case RELEASE_HOME_KEY:
		if(test_bit(KEY_HOME, kpd_input_dev->keybit)){
			printk("[AUTOTEST] RELEASE HOME KEY!!\n");
			input_report_key(kpd_input_dev, KEY_HOME, 0);
			input_sync(kpd_input_dev);
		}else{
			printk("[AUTOTEST] Not Support HOME KEY!!\n");
		}
		break;
	case PRESS_BACK_KEY:
		if(test_bit(KEY_BACK, kpd_input_dev->keybit)){
			printk("[AUTOTEST] PRESS BACK KEY!!\n");
			input_report_key(kpd_input_dev, KEY_BACK, 1);
			input_sync(kpd_input_dev);
		}else{
			printk("[AUTOTEST] Not Support BACK KEY!!\n");
		}
		break;
	case RELEASE_BACK_KEY:
		if(test_bit(KEY_BACK, kpd_input_dev->keybit)){
			printk("[AUTOTEST] RELEASE BACK KEY!!\n");
			input_report_key(kpd_input_dev, KEY_BACK, 0);
			input_sync(kpd_input_dev);
		}else{
			printk("[AUTOTEST] Not Support BACK KEY!!\n");
		}
		break;
	case PRESS_CALL_KEY:
		if(test_bit(KEY_CALL, kpd_input_dev->keybit)){
			printk("[AUTOTEST] PRESS CALL KEY!!\n");
			input_report_key(kpd_input_dev, KEY_CALL, 1);
			input_sync(kpd_input_dev);
		}else{
			printk("[AUTOTEST] Not Support CALL KEY!!\n");
		}
		break;
	case RELEASE_CALL_KEY:
		if(test_bit(KEY_CALL, kpd_input_dev->keybit)){
			printk("[AUTOTEST] RELEASE CALL KEY!!\n");
			input_report_key(kpd_input_dev, KEY_CALL, 0);
			input_sync(kpd_input_dev);
		}else{
			printk("[AUTOTEST] Not Support CALL KEY!!\n");
		}
		break;

	case PRESS_ENDCALL_KEY:
		if(test_bit(KEY_ENDCALL, kpd_input_dev->keybit)){
			printk("[AUTOTEST] PRESS ENDCALL KEY!!\n");
			input_report_key(kpd_input_dev, KEY_ENDCALL, 1);
			input_sync(kpd_input_dev);
		}else{
			printk("[AUTOTEST] Not Support ENDCALL KEY!!\n");
		}
		break;
	case RELEASE_ENDCALL_KEY:
		if(test_bit(KEY_ENDCALL, kpd_input_dev->keybit)){
			printk("[AUTOTEST] RELEASE ENDCALL KEY!!\n");
			input_report_key(kpd_input_dev, KEY_ENDCALL, 0);
			input_sync(kpd_input_dev);
		}else{
			printk("[AUTOTEST] Not Support ENDCALL KEY!!\n");
		}
		break;
	case PRESS_VLUP_KEY:
		if(test_bit(KEY_VOLUMEUP, kpd_input_dev->keybit)){
			printk("[AUTOTEST] PRESS VOLUMEUP KEY!!\n");
			input_report_key(kpd_input_dev, KEY_VOLUMEUP, 1);
			input_sync(kpd_input_dev);
		}else{
			printk("[AUTOTEST] Not Support VOLUMEUP KEY!!\n");
		}
		break;
	case RELEASE_VLUP_KEY:
		if(test_bit(KEY_VOLUMEUP, kpd_input_dev->keybit)){
			printk("[AUTOTEST] RELEASE VOLUMEUP KEY!!\n");
			input_report_key(kpd_input_dev, KEY_VOLUMEUP, 0);
			input_sync(kpd_input_dev);
		}else{
			printk("[AUTOTEST] Not Support VOLUMEUP KEY!!\n");
		}
		break;
	case PRESS_VLDOWN_KEY:
		if(test_bit(KEY_VOLUMEDOWN, kpd_input_dev->keybit)){
			printk("[AUTOTEST] PRESS VOLUMEDOWN KEY!!\n");
			input_report_key(kpd_input_dev, KEY_VOLUMEDOWN, 1);
			input_sync(kpd_input_dev);
		}else{
			printk("[AUTOTEST] Not Support VOLUMEDOWN KEY!!\n");
		}
		break;
	case RELEASE_VLDOWN_KEY:
		if(test_bit(KEY_VOLUMEDOWN, kpd_input_dev->keybit)){
			printk("[AUTOTEST] RELEASE VOLUMEDOWN KEY!!\n");
			input_report_key(kpd_input_dev, KEY_VOLUMEDOWN, 0);
			input_sync(kpd_input_dev);
		}else{
			printk("[AUTOTEST] Not Support VOLUMEDOWN KEY!!\n");
		}
		break;
	case PRESS_FOCUS_KEY:
		if(test_bit(KEY_FOCUS, kpd_input_dev->keybit)){
			printk("[AUTOTEST] PRESS FOCUS KEY!!\n");
			input_report_key(kpd_input_dev, KEY_FOCUS, 1);
			input_sync(kpd_input_dev);
		}else{
			printk("[AUTOTEST] Not Support FOCUS KEY!!\n");
		}
		break;
	case RELEASE_FOCUS_KEY:
		if(test_bit(KEY_FOCUS, kpd_input_dev->keybit)){
			printk("[AUTOTEST] RELEASE FOCUS KEY!!\n");
			input_report_key(kpd_input_dev, KEY_FOCUS, 0);
			input_sync(kpd_input_dev);
		}else{
			printk("[AUTOTEST] Not Support RELEASE KEY!!\n");
		}
		break;
	case PRESS_CAMERA_KEY:
		if(test_bit(KEY_CAMERA, kpd_input_dev->keybit)){
			printk("[AUTOTEST] PRESS CAMERA KEY!!\n");
			input_report_key(kpd_input_dev, KEY_CAMERA, 1);
			input_sync(kpd_input_dev);
		}else{
			printk("[AUTOTEST] Not Support CAMERA KEY!!\n");
		}
		break;
	case RELEASE_CAMERA_KEY:
		if(test_bit(KEY_CAMERA, kpd_input_dev->keybit)){
			printk("[AUTOTEST] RELEASE CAMERA KEY!!\n");
			input_report_key(kpd_input_dev, KEY_CAMERA, 0);
			input_sync(kpd_input_dev);
		}else{
			printk("[AUTOTEST] Not Support CAMERA KEY!!\n");
		}
		break;
	case PRESS_POWER_KEY:
		if(test_bit(KEY_POWER, kpd_input_dev->keybit)){
			printk("[AUTOTEST] PRESS POWER KEY!!\n");
			input_report_key(kpd_input_dev, KEY_POWER, 1);
			input_sync(kpd_input_dev);
		}else{
			printk("[AUTOTEST] Not Support POWER KEY!!\n");
		}
		break;
	case RELEASE_POWER_KEY:
		if(test_bit(KEY_POWER, kpd_input_dev->keybit)){
			printk("[AUTOTEST] RELEASE POWER KEY!!\n");
			input_report_key(kpd_input_dev, KEY_POWER, 0);
			input_sync(kpd_input_dev);
		}else{
			printk("[AUTOTEST] Not Support POWER KEY!!\n");
		}
		break;
#endif
		
	case SET_KPD_KCOL:
		kpd_auto_test_for_factorymode();//API 3 for kpd factory mode auto-test
		printk("[kpd_auto_test_for_factorymode] test performed!!\n");
		break;		
	default:
		return -EINVAL;
	}

	return 0;
}


int kpd_dev_open(struct inode *inode, struct file *file)
{
	return 0;
}

static struct file_operations kpd_dev_fops = {
	.owner		= THIS_MODULE,
	.unlocked_ioctl	= kpd_dev_ioctl,
	.open		= kpd_dev_open,
};
/*********************************************************************/
static struct miscdevice kpd_dev = {
	.minor	= MISC_DYNAMIC_MINOR,
	.name	= KPD_NAME,
	.fops	= &kpd_dev_fops,
};

static int kpd_open(struct input_dev *dev)
{
	kpd_slide_qwerty_init();//API 1 for kpd slide qwerty init settings	
	return 0;
}


static int kpd_pdrv_probe(struct platform_device *pdev)
{
	
	int i, r;
	int err = 0;
	
	kpd_ldvt_test_init();//API 2 for kpd LFVT test enviroment settings

	/* initialize and register input device (/dev/input/eventX) */
	kpd_input_dev = input_allocate_device();
	if (!kpd_input_dev)
		return -ENOMEM;

	kpd_input_dev->name = KPD_NAME;
	kpd_input_dev->id.bustype = BUS_HOST;
	kpd_input_dev->id.vendor = 0x2454;
	kpd_input_dev->id.product = 0x6500;
	kpd_input_dev->id.version = 0x0010;
	kpd_input_dev->open = kpd_open;

	//fulfill custom settings	
	kpd_memory_setting();
	
	__set_bit(EV_KEY, kpd_input_dev->evbit);

#if (KPD_PWRKEY_USE_EINT||KPD_PWRKEY_USE_PMIC)
	__set_bit(KPD_PWRKEY_MAP, kpd_input_dev->keybit);
	kpd_keymap[8] = 0;
#endif
#ifdef EXTEND_KEY_AW9523_SUPPORT
	__set_bit(KEY_0, kpd_input_dev->keybit);
       __set_bit(KEY_1, kpd_input_dev->keybit);
	_set_bit(KEY_2, kpd_input_dev->keybit);
       __set_bit(KEY_3, kpd_input_dev->keybit);
	 _set_bit(KEY_4, kpd_input_dev->keybit);
       __set_bit(KEY_5, kpd_input_dev->keybit);
	_set_bit(KEY_6, kpd_input_dev->keybit);
       __set_bit(KEY_7, kpd_input_dev->keybit);
	 _set_bit(KEY_8, kpd_input_dev->keybit);
       __set_bit(KEY_9, kpd_input_dev->keybit);
	 _set_bit(KEY_STAR, kpd_input_dev->keybit);
       __set_bit(0xe4, kpd_input_dev->keybit);
	   
	 __set_bit(KEY_F1, kpd_input_dev->keybit);
	 __set_bit(KEY_UP, kpd_input_dev->keybit);
	 __set_bit(KEY_LEFT, kpd_input_dev->keybit);
	 __set_bit(KEY_F3, kpd_input_dev->keybit);
	 __set_bit(KEY_F2, kpd_input_dev->keybit);
	 __set_bit(KEY_DOWN, kpd_input_dev->keybit);
	 __set_bit(KEY_RIGHT, kpd_input_dev->keybit);
	 __set_bit(KEY_F4, kpd_input_dev->keybit);
	 __set_bit(KEY_TAB, kpd_input_dev->keybit);
	  __set_bit(KEY_ESC, kpd_input_dev->keybit);
	   __set_bit(KEY_DELETE, kpd_input_dev->keybit);
	 __set_bit(KEY_ENTER, kpd_input_dev->keybit);
	 
	__set_bit(KEY_F12, kpd_input_dev->keybit);
        __set_bit(KEY_LEFTSHIFT, kpd_input_dev->keybit);
        __set_bit(KEY_LEFTCTRL, kpd_input_dev->keybit);

#endif

	for (i = 17; i < KPD_NUM_KEYS; i += 9)	/* only [8] works for Power key */
		kpd_keymap[i] = 0;

	for (i = 0; i < KPD_NUM_KEYS; i++) {
		if (kpd_keymap[i] != 0)
			__set_bit(kpd_keymap[i], kpd_input_dev->keybit);
	}

#if KPD_AUTOTEST
	for (i = 0; i < ARRAY_SIZE(kpd_auto_keymap); i++)
		__set_bit(kpd_auto_keymap[i], kpd_input_dev->keybit);
#endif

#if KPD_HAS_SLIDE_QWERTY
	__set_bit(EV_SW, kpd_input_dev->evbit);
	__set_bit(SW_LID, kpd_input_dev->swbit);
#endif

#ifdef KPD_PMIC_RSTKEY_MAP
	__set_bit(KPD_PMIC_RSTKEY_MAP, kpd_input_dev->keybit);
#endif

	kpd_input_dev->dev.parent = &pdev->dev;
	r = input_register_device(kpd_input_dev);
	if (r) {
		printk(KPD_SAY "register input device failed (%d)\n", r);
		input_free_device(kpd_input_dev);
		return r;
	}

	/* register device (/dev/mt6575-kpd) */
	kpd_dev.parent = &pdev->dev;
	r = misc_register(&kpd_dev);
	if (r) {
		printk(KPD_SAY "register device failed (%d)\n", r);
		input_unregister_device(kpd_input_dev);
		return r;
	}

	/* register IRQ and EINT */
	kpd_set_debounce(KPD_KEY_DEBOUNCE);
	r = request_irq(MT_KP_IRQ_ID, kpd_irq_handler, IRQF_TRIGGER_FALLING, KPD_NAME, NULL);
	if (r) {
		printk(KPD_SAY "register IRQ failed (%d)\n", r);
		misc_deregister(&kpd_dev);
		input_unregister_device(kpd_input_dev);
		return r;
	}
  #if defined(EXTEND_KEY_AW9523_SUPPORT)
        //mt_set_gpio_mode(AW9523_EINT_GPIO, GPIO_MODE_03);
       // mt_set_gpio_dir(AW9523_EINT_GPIO, GPIO_DIR_IN);
      //  mt_set_gpio_pull_enable(AW9523_EINT_GPIO, GPIO_PULL_DISABLE); //To disable GPIO PULL.

	 mt_set_gpio_mode(GPIO_MHALL_EINT_PIN, GPIO_MHALL_EINT_PIN_M_EINT);
     mt_set_gpio_dir(GPIO_MHALL_EINT_PIN, GPIO_DIR_IN);
     mt_set_gpio_pull_enable(GPIO_MHALL_EINT_PIN, GPIO_PULL_ENABLE);
     mt_set_gpio_pull_select(GPIO_MHALL_EINT_PIN, GPIO_PULL_UP);
	 
   pmic_config_interface(0x0532, 0x6, 0x7, 5); 
    pmic_config_interface(0x050c, 0x1, 0x1, 15);
   Set_P0_X_AND_P1_Y();
        aw9523_init();
	//AW9523_delay_1us(2000); 
	//aw9523_test();
       printk("aw9523_i2c_read_reg_0x10=0x%x\r\n",AW9523_i2c_read_reg(0x10));    	
	mt65xx_eint_set_sens(CUST_EINT_MHALL_NUM , CUST_EINT_EDGE_SENSITIVE);
	mt65xx_eint_set_hw_debounce(CUST_EINT_MHALL_NUM , CUST_EINT_MHALL_DEBOUNCE_CN);
	mt65xx_eint_registration(CUST_EINT_MHALL_NUM ,true, CUST_EINT_MHALL_POLARITY, kpd_aw9523_eint_handler, false);
	//mt_eint_unmask(CUST_EINT_MHALL_NUM );
#endif

#if KPD_PWRKEY_USE_EINT
	mt_eint_register();
#endif

#ifndef KPD_EARLY_PORTING /*add for avoid early porting build err the macro is defined in custom file*/
	long_press_reboot_function_setting();///API 4 for kpd long press reboot function setting
#endif	
	hrtimer_init(&aee_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	aee_timer.function = aee_timer_func;

#if AEE_ENABLE_5_15
    hrtimer_init(&aee_timer_5s, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    aee_timer_5s.function = aee_timer_5s_func;
#endif

	if((err = kpd_create_attr(&kpd_pdrv.driver)))
	{
		kpd_print("create attr file fail\n");
		kpd_delete_attr(&kpd_pdrv.driver);
		return err;
	}

	return 0;
}

/* should never be called */
static int kpd_pdrv_remove(struct platform_device *pdev)
{
	return 0;
}

#ifndef USE_EARLY_SUSPEND
static int kpd_pdrv_suspend(struct platform_device *pdev, pm_message_t state)
{
	kpd_suspend = true;
#ifdef MTK_KP_WAKESOURCE
	if(call_status == 2){
		kpd_print("kpd_early_suspend wake up source enable!! (%d)\n", kpd_suspend);
	}else{
		kpd_wakeup_src_setting(0);
		kpd_print("kpd_early_suspend wake up source disable!! (%d)\n", kpd_suspend);
	}
#endif		
	kpd_disable_backlight();
	kpd_print("suspend!! (%d)\n", kpd_suspend);
	return 0;
}

static int kpd_pdrv_resume(struct platform_device *pdev)
{
	kpd_suspend = false;	
#ifdef MTK_KP_WAKESOURCE
	if(call_status == 2){
		kpd_print("kpd_early_suspend wake up source enable!! (%d)\n", kpd_suspend);
	}else{
		kpd_print("kpd_early_suspend wake up source resume!! (%d)\n", kpd_suspend);
		kpd_wakeup_src_setting(1);
	}
#endif	
	kpd_print("resume!! (%d)\n", kpd_suspend);
	return 0;
}
#else
#define kpd_pdrv_suspend	NULL
#define kpd_pdrv_resume		NULL
#endif


#ifdef USE_EARLY_SUSPEND
static void kpd_early_suspend(struct early_suspend *h)
{
	kpd_suspend = true;
#ifdef MTK_KP_WAKESOURCE
	if(call_status == 2){
		kpd_print("kpd_early_suspend wake up source enable!! (%d)\n", kpd_suspend);
	}else{
		//kpd_wakeup_src_setting(0);
		kpd_print("kpd_early_suspend wake up source disable!! (%d)\n", kpd_suspend);
	}
#endif	
	kpd_disable_backlight();
	kpd_print("early suspend!! (%d)\n", kpd_suspend);
}

static void kpd_early_resume(struct early_suspend *h)
{
	kpd_suspend = false;
#ifdef MTK_KP_WAKESOURCE
	if(call_status == 2){
		kpd_print("kpd_early_resume wake up source resume!! (%d)\n", kpd_suspend);
	}else{
		kpd_print("kpd_early_resume wake up source enable!! (%d)\n", kpd_suspend);
		//kpd_wakeup_src_setting(1);
	}
#endif	
	kpd_print("early resume!! (%d)\n", kpd_suspend);
}

static struct early_suspend kpd_early_suspend_desc = {
	.level		= EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1,
	.suspend	= kpd_early_suspend,
	.resume		= kpd_early_resume,
};
#endif

#ifdef MTK_SMARTBOOK_SUPPORT
#ifdef CONFIG_HAS_SBSUSPEND
static struct sb_handler kpd_sb_handler_desc = {
	.level		= SB_LEVEL_DISABLE_KEYPAD,
	.plug_in	= sb_kpd_enable,
	.plug_out	= sb_kpd_disable,
};
#endif
#endif

static int __init kpd_mod_init(void)
{
	int r;

	r = platform_driver_register(&kpd_pdrv);
	if (r) {
		printk(KPD_SAY "register driver failed (%d)\n", r);
		return r;
	}

#ifdef USE_EARLY_SUSPEND
	register_early_suspend(&kpd_early_suspend_desc);
#endif

#ifdef MTK_SMARTBOOK_SUPPORT
#ifdef CONFIG_HAS_SBSUSPEND
	register_sb_handler(&kpd_sb_handler_desc);
#endif
#endif

	return 0;
}

/* should never be called */
static void __exit kpd_mod_exit(void)
{
}

module_init(kpd_mod_init);
module_exit(kpd_mod_exit);

module_param(kpd_show_hw_keycode, int, 0644);
module_param(kpd_show_register, int, 0644);

MODULE_AUTHOR("yucong.xiong <yucong.xiong@mediatek.com>");
MODULE_DESCRIPTION("MTK Keypad (KPD) Driver v0.4");
MODULE_LICENSE("GPL");
