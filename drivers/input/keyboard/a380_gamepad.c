/* ****************************************************************
 * @ func   : A380 Game Keypad Driver
 * @ author : maddrone@gmail.com
 * ****************************************************************/
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/tty.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/fb.h>
#include <linux/init.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/kthread.h>
#include <linux/poll.h>
#include <asm/irq.h>
#include <asm/pgtable.h>
#include <asm/system.h>
#include <asm/uaccess.h>
#include <asm/processor.h>
#include <asm/jzsoc.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/sched.h>
#include <linux/miscdevice.h>
#include <linux/proc_fs.h>



#define L009_KEY_UP     0
#define L009_KEY_DOWN   1
#define L009_KEY_LEFT   2
#define L009_KEY_RIGHT  3
#define L009_KEY_A      4
#define L009_KEY_B      5
#define L009_KEY_X      6
#define L009_KEY_Y      7
#define L009_KEY_L      8
#define L009_KEY_R      9
#define L009_KEY_START    10
#define L009_KEY_SELECT   11
#define L009_KEY_HOLD     12
#define L009_KEY_POWER    13

static unsigned int key_value;

unsigned int jz_button[] =
{
	(32 * 4 + 0),	// UP        0
	(32 * 4 + 1),   // DOWN      1
	(32 * 4 + 2),   // LEFT      2
	(32 * 4 + 3),   // RIGHT     3
	(32 * 2 + 31),  // A         4
	(32 * 3 + 16),  // B         5
	(32 * 3 + 17),  // X         6
	(32 * 4 + 11),  // Y         7
	(32 * 4 + 10),   // L         8
	(32 * 4 + 7),  // R         9
	(32 * 3 + 21),  // START     10
	(32 * 4 + 8),   // SELECT    11
	//(32 * 3 + 18),  // HOLD      12
	(125)           // POWER     13
};

static int key_open(struct inode *inode, struct file *filp)
{
	printk (KERN_ALERT "key_open\n");
	key_value = 0;
	return 0;
}

static int key_release(struct inode *inode, struct file *filp)
{
	printk(KERN_ALERT "key_release\n");
	return 0;
}

//maddrone add elan wireless pad
extern unsigned int elan_key_val;
extern unsigned long elan_state_flag;

extern unsigned int l009_gsensor_flag;
extern unsigned int l009_gsensor_read();

static ssize_t key_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
	int i;

	key_value = 0;
	for(i=0; i<ARRAY_SIZE(jz_button); i++)
	{
          if(i == 4)
          {
            //printk("%s %d __gpio_get_pin(jz_button[i]) is %x\n",__FILE__,__LINE__,__gpio_get_pin(jz_button[i]));

            if((__gpio_get_pin(jz_button[i])))  key_value |= (1 << i);
          }
          else
          {
            if(!(__gpio_get_pin(jz_button[i])))  key_value |= (1 << i);
          }
	}
        //if(key_value != 0)
         // printk("%s %d key_value is %x\n",__FILE__,__LINE__,key_value);

        if(elan_state_flag > 0)
        {
          key_value |= elan_key_val;
        }

        //if(key_value != 0)
         // printk("%s %d key_value is %x\n",__FILE__,__LINE__,key_value);

        if(l009_gsensor_flag)
        {
          key_value |= l009_gsensor_read();
        }
        //if(key_value != 0)
         // printk("%s %d key_value is %x\n",__FILE__,__LINE__,key_value);

	copy_to_user(buf, &key_value, sizeof(int));

	return sizeof(int);
}

#define 	KEYPAD_MAJOR 		252
#define 	KEYPAD_DEVICE_NAME	"keypad"

static struct file_operations keypad_fops = {
    owner:              THIS_MODULE,
    open:               key_open,
    read:               key_read,
    release:            key_release,
};

static struct miscdevice keypad_device = {
	.minor      = KEYPAD_MAJOR,
	.name       = KEYPAD_DEVICE_NAME,
	.fops       = &keypad_fops,
};


static int __init keypad_init(void)
{
	int i,ret;

	ret = misc_register(&keypad_device);
	if(ret<0){
		printk("kernel : keypad register failed!\n");
	}

	for(i=0; i<ARRAY_SIZE(jz_button); i++)
	{
          if(i != 4)
          {
		__gpio_as_input(jz_button[i]);
		__gpio_enable_pull(jz_button[i]);
          }
	}


	return 0;
}

static void __exit keypad_cleanup(void)
{
	return ;
}

module_init(keypad_init);
module_exit(keypad_cleanup);

MODULE_AUTHOR("maddrone");
MODULE_DESCRIPTION("L009 keypad driver");
MODULE_LICENSE("GPL");


