/********************************************************************** 
 akpic - Copyright (C) 2009 - Andreas Kemnade
 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 3, or (at your option)
 any later version.
             
 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied
 warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
***********************************************************************/

#if 0
#include <linux/config.h>
#endif
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/signal.h>
#include <linux/slab.h>
#include <linux/module.h>
//#include <linux/smp_lock.h>
#include <linux/usb.h>
#include <linux/interrupt.h>

#include <linux/version.h>
#include <linux/i2c.h>
//#include <linux/i2c-id.h>

#include <asm/uaccess.h>

#define DRIVER_VERSION "v0.1"
#define DRIVER_AUTHOR "Andreas Kemnade <andreas@kemnade.info>"
#define DRIVER_DESC "USB PIC programmer/I2C adaptor"
#define USBAKPIC_MINOR 180
#define MAX_DEVS 16
static const char driver_name [] = "akpic";
static int register_i2c;
static struct usb_device_id id_table [] = {
	{ USB_DEVICE(0x16d0, 0x04d9)},
	{ USB_DEVICE(0x0004, 0x0009)},
	{ USB_DEVICE(0x0004, 0x000b)},
	{ USB_DEVICE(0x0004, 0x000e)},
	{}
};

MODULE_DEVICE_TABLE (usb, id_table);


static int akpic_probe(struct usb_interface *intf,
                       const struct usb_device_id *id);
static void akpic_disconnect(struct usb_interface *interface);
static struct usb_driver akpic_driver = {
	.name = driver_name,
	.probe =   akpic_probe,
	.disconnect = akpic_disconnect,
	/* .fasync = akpic_fasync, */
	.id_table = id_table
};

typedef struct {
	char buf[256];
	char *rcvbuf;
	int inpos;
	int outpos;
	char sndbuf[4];
	struct usb_device *udev;
	struct usb_interface *other_if;
	struct urb *rx_urb;
	struct fasync_struct *fasync;
	wait_queue_head_t rcv_wait;
	// struct urb *tx_urb;
	int open_cnt;
	int is_closed;
	int subcl;
	int devid;
	struct i2c_adapter *i2c_ops;
	struct kref             kref;
} akpic_t;
static akpic_t *devtable[MAX_DEVS];
static struct i2c_adapter i2cdevtable[MAX_DEVS];

static int akpic_open(struct inode *inode, struct file *file);
static ssize_t akpic_write(struct file *file,
		       const char *user_buffer, size_t count, loff_t *ppos);
static ssize_t akpic_read(struct file *file, char *buffer,
		      size_t count, loff_t *ppos);
static int akpic_release(struct inode *inode, struct file *file);
static int akpic_ioctl(struct inode *inode, struct file *file,
		       unsigned int cmd, unsigned long arg);
static struct file_operations akpic_fops = {
	.owner = THIS_MODULE,
	.read = akpic_read,
	.write = akpic_write,
	.open = akpic_open,
	/*.ioctl = akpic_ioctl, */
	.release = akpic_release,
};

static struct usb_class_driver akpic_class = {
	.name = "akpic%d",
	.fops = &akpic_fops,
	.minor_base = USBAKPIC_MINOR,
};

static void akpic_read_bulk_callback(struct urb *urb)
{
	akpic_t *dev = (akpic_t *)urb->context;
	int cp_len = urb->actual_length;
	if (!dev)
		return;
	//printk("got %d bytes of data\n",cp_len);
	wake_up_interruptible(&dev->rcv_wait);
	switch(urb->status) {
        case 0:
		break;
        case -ENOENT:
		return;
        case -ETIME:
		dev->is_closed=1;
		printk("device disonnecting? urb rx status: %d\n",urb->status);
		return;
        default:
		printk("urb rx status: %d\n",urb->status);
		goto goon;
	}
	if ((dev->inpos+cp_len) >sizeof(dev->buf)) {
		int rest=sizeof(dev->buf)-dev->inpos;
		memcpy(dev->buf+dev->inpos,urb->transfer_buffer, rest);
		memcpy(dev->buf,urb->transfer_buffer+rest,cp_len-rest);
		memcpy(dev->buf,urb->transfer_buffer+rest,cp_len-rest);
		dev->inpos=cp_len-rest;
	} else {
		memcpy(dev->buf+dev->inpos, urb->transfer_buffer, cp_len);
		dev->inpos+=cp_len;
	}
goon:
	
	usb_fill_bulk_urb(dev->rx_urb, dev->udev,
			  usb_rcvbulkpipe(dev->udev, 3),
			  dev->rcvbuf, 64, akpic_read_bulk_callback, dev);
	
	usb_submit_urb(dev->rx_urb, GFP_ATOMIC);
	return;

}


static void akpic_write_bulk_callback(struct urb *urb)
{
	akpic_t *akpic = (akpic_t *)urb->context;
	//printk("akpic: write intr status: %d\n",urb->status);
	usb_free_coherent(urb->dev, urb->transfer_buffer_length,
			urb->transfer_buffer, urb->transfer_dma);
	
}
static void akpic_delete(struct kref *kref)
{
	akpic_t *dev = container_of(kref, akpic_t, kref);
	printk("akpic delete_dev\n");
	usb_put_dev(dev->udev);
	kfree(dev->rcvbuf);
	kfree(dev);
}

static akpic_t *akpic_refpic(akpic_t *dev, int num)
{
	int res;
	//lock_kernel();
	if (!dev)
		dev=devtable[num];
	if (dev) {
		kref_get(&dev->kref);
		if (! dev->open_cnt) {
			dev->open_cnt++;
			usb_fill_bulk_urb(dev->rx_urb, dev->udev,
					  usb_rcvbulkpipe(dev->udev, 3),
					  dev->rcvbuf, 64, akpic_read_bulk_callback, dev);
			if ((res = usb_submit_urb(dev->rx_urb, GFP_KERNEL))) {
				printk("submit rx urb failed\n");
			}
		} else {
			dev->open_cnt++;
		}
	}
	//unlock_kernel();
	return dev; 
}

static void akpic_unrefpic(akpic_t *dev)
{
	//lock_kernel();
	dev->open_cnt--;
	if (!dev->open_cnt) {
		//unlock_kernel();
		usb_kill_urb(dev->rx_urb);
	} else {
		//unlock_kernel();
	}
	kref_put(&dev->kref, akpic_delete);
}

static int akpic_open(struct inode *inode, struct file *file)
{
	akpic_t *dev;
	struct usb_interface *interface;
	int subminor;
	int res;
	int retval = 0;
	subminor = iminor(inode);
	interface = usb_find_interface(&akpic_driver, subminor);
	if (!interface) {
		retval = -ENODEV;
		goto exit;
	}
	dev = usb_get_intfdata(interface);
	if (!dev) {
		retval = -ENODEV;
		goto exit;
	}
	file->private_data = dev;
	akpic_refpic(dev,dev->devid);
exit:
	return retval;
}

static int akpic_release(struct inode *inode, struct file *file)
{
	akpic_t *dev;
	dev = (akpic_t *)file->private_data;
	if (dev == NULL)
		return -ENODEV;
	printk("akpic: closing dev\n");
	//lock_kernel();
	dev->open_cnt--;
	if (!dev->open_cnt) {
		//unlock_kernel();
		usb_kill_urb(dev->rx_urb);
	} else {
		//unlock_kernel();
	}
	kref_put(&dev->kref, akpic_delete);
	return 0;
}

static ssize_t akpic_read(struct file *file, char __user *buffer,
		      size_t count, loff_t *ppos)
{
	akpic_t *dev = (akpic_t *)file->private_data;
	int i,retval = 0;
	int outpos;
	int inpos; 
	DECLARE_WAITQUEUE(wait,current);
	if ((file->f_flags&O_NONBLOCK)&&(dev->inpos==dev->outpos))
		return -EAGAIN;
	outpos=dev->outpos;
	add_wait_queue(&dev->rcv_wait,&wait);
	while((dev->inpos == dev->outpos)&&(!dev->is_closed)) {
		set_current_state(TASK_INTERRUPTIBLE); 
		if (dev->inpos != dev->outpos)
			break;
		if (dev->is_closed)
			break;
		schedule();
		if (signal_pending(current)) {
			retval=-EINTR;
			printk("interrupted in akpic_read %d %d\n",dev->inpos,dev->outpos);
			break;
		}
	}
	current->state=TASK_RUNNING;
	remove_wait_queue(&dev->rcv_wait,&wait);
	if (retval<0)
		return retval;
	inpos=dev->inpos;
	if (inpos == sizeof(dev->buf))
		inpos=0;
	if (dev->outpos > inpos) {
		if ((sizeof(dev->buf)-dev->outpos) < count) {
			count = sizeof(dev->buf)-dev->outpos;
			// printk("akpic: copying %d bytes till end of buffer\n", count);
			dev->outpos = 0;
		} else {
			dev->outpos += count;
		}
	} else {
		int indiff = inpos - dev->outpos;
		if (indiff < count) {
			count = inpos - dev->outpos;
		}
		dev->outpos += count;
		// printk("akpic: copying %d bytes\n", count);
	}
	if (dev->outpos >= sizeof(dev->buf)) {
		dev->outpos = 0;
	}
	if (!retval) {
		if (copy_to_user(buffer, dev->buf + outpos, count)) {
			retval = -EFAULT;
		} else {
			retval = count;
		}
	}
	return retval;
}
static ssize_t akpic_write_data(akpic_t *dev,u8 *ubuf, size_t count)
{
	int retval = 0;
	struct urb *urb = NULL;
	char *buf = NULL;
	urb = usb_alloc_urb(0, GFP_KERNEL);
	if (!urb) {
		retval = -ENOMEM;
		goto error;
	}
	buf = usb_alloc_coherent(dev->udev, count, GFP_KERNEL,
			         &urb->transfer_dma);
	if (!buf) {
		retval = -ENOMEM;
		goto error;
	}
	memcpy(buf,ubuf,count);
	usb_fill_bulk_urb(urb, dev->udev, usb_sndbulkpipe(dev->udev, 3), buf, count,
			  akpic_write_bulk_callback, dev);
	urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
	retval = usb_submit_urb(urb, GFP_KERNEL);
	if (retval) {
		dev_err(&dev->udev->dev,"failed writing data, error %d", retval);
		goto error;
	}
	usb_free_urb(urb);
exit:
	return count;
error:
	usb_free_coherent(dev->udev, count, buf, urb->transfer_dma);
	usb_free_urb(urb);
	return retval;
}
static ssize_t akpic_write(struct file *file,
		       const char __user *user_buffer, size_t count, loff_t *ppos)
{
	akpic_t *dev = (akpic_t *)file->private_data;
	int retval = 0;
	struct urb *urb = NULL;
	char *buf = NULL;
	// printk("akpic: writing %d bytes\n", count);
	if (count == 0)
		goto exit;
	if (count >64) {
		count=64;
	}

	urb = usb_alloc_urb(0, GFP_KERNEL);
	if (!urb) {
		retval = -ENOMEM;
		goto error;
	}
	buf = usb_alloc_coherent(dev->udev, count, GFP_KERNEL,
			         &urb->transfer_dma);
	if (!buf) {
		retval = -ENOMEM;
		goto error;
	}
	if (copy_from_user(buf, user_buffer, count)) {
		retval = -EFAULT;
		goto error;
	}
	usb_fill_bulk_urb(urb, dev->udev, usb_sndbulkpipe(dev->udev, 3), buf, count,
			  akpic_write_bulk_callback, dev);
	urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
	retval = usb_submit_urb(urb, GFP_KERNEL);
	if (retval) {
		dev_err(&dev->udev->dev,"failed to write, error %d", retval);
		goto error;
	}
	usb_free_urb(urb);
exit:
	return count;
error:
	usb_free_coherent(dev->udev, count, buf, urb->transfer_dma);
	usb_free_urb(urb);
	return retval;
}

static int akpic_ioctl(struct inode *inode, struct file *file,
		       unsigned int cmd, unsigned long arg)
{
	akpic_t *dev = (akpic_t *)file->private_data;
	if (dev == NULL) {
		return -ENODEV;
	}
	return -ENOTTY;
}

static int akpic_write_i2c_byte(akpic_t *dev,u8 chr, int write_start, int write_stop)
{
	u8 buf[5]={0x9f,0x98,0,0xb1,0x9e}; 
	int i;
	int act_len=0;
	int retval=0;
        int slen=sizeof(buf);
	DECLARE_WAITQUEUE(wait,current);
        if (dev->subcl == 0xca) {
		buf[2]=chr;
	} else {
		for(i=0;i<8;i++) {
			buf[1]=buf[1]<<1;
			if (chr&(1<<i)) {
				buf[1]|=1;
			}
		}
        }
	
	dev->inpos=dev->outpos=0;
        // if ((!write_stop) || (dev->subcl != 0xca))
		slen--;
        if (write_start && (dev->subcl == 0xca))
		akpic_write_data(dev,buf,slen);
	else
		akpic_write_data(dev,buf+1,slen-1);
	i=0;
	add_wait_queue(&dev->rcv_wait,&wait);
	while((dev->inpos == dev->outpos) && (!dev->is_closed)) {
		set_current_state(TASK_INTERRUPTIBLE); 
		if (dev->inpos != dev->outpos)
    		      break;
		if (dev->is_closed)
			break;
		schedule();
       		if (signal_pending(current))  {
			printk("interrupt in akpic_write_i2c\n");
			retval=-EINTR;
			break;
        	}
	}
	current->state=TASK_RUNNING;
	remove_wait_queue(&dev->rcv_wait,&wait);
	if (retval<0)
		return retval;
	return dev->buf[0];
	
	
}

/* max 64/4=16 bytes */
static int akpic_read_i2c_bytes(akpic_t *dev, u8 *b,int count,int ack, int write_stop)
{
	/*u8 buf[]={0x00,0xb8,0x00,0x04,0x91,0x00}; */
	u8 buf[]={0xb8,0x91,0x00};
	u8 buf2[64];
	int act_len=0;
	int retval=0;
	unsigned char r;
	int i;
	DECLARE_WAITQUEUE(wait,current);
	buf2[0]=0;
	for(i=0;i<count;i++) {
		memcpy(buf2+1+i*sizeof(buf),buf,3); 
	}
	if (!i)
		return 0;
	if (ack)
		buf2[i*sizeof(buf)]=0xff;
	act_len = count*sizeof(buf)+1;
#if 0
        if ((dev->subcl == 0xca) && (write_stop)) {
		buf2[act_len] = 0x9e;
		act_len++;
	} 
#endif
	dev->inpos=dev->outpos=0;
	akpic_write_data(dev,buf2,act_len);
	i=0;
	add_wait_queue(&dev->rcv_wait,&wait);
	while((dev->inpos < count)&(!dev->is_closed)) {
		set_current_state(TASK_INTERRUPTIBLE); 
		if (dev->inpos==count)
			break;
		if (dev->is_closed)
			break;
		schedule();
		if (signal_pending(current)) {
			retval=-EINTR;
			printk("interrupt in akpic_read_i2c %d %d\n",dev->inpos,dev->outpos);
			break;
		}

	}
	current->state=TASK_RUNNING;
	remove_wait_queue(&dev->rcv_wait,&wait);
	if (retval<0)
		return retval;
	while(count>0) {
		r=dev->buf[dev->outpos];
		*b=0;
                if (dev->subcl == 0xca) {
			*b=r;
		} else {
			for(i=0;i<8;i++) {
				*b=(*b)<<1; 
				if (r&(1<<i)) {
					*b|=1;
				}
			}
		}
		count--;
		b++; 
		dev->outpos++;
	} 
	// printk("i2c data read: %d\n",r);
	return 0; 
}

static int akpic_i2c_xfer(struct i2c_adapter *adap,
                          struct i2c_msg *msgs, int num)
{
	static u8 start_msg[]={0x00,0x04,0x87,0x04,0x87,0x04,0x86,0x04,0x84};
	static u8 stop_msg[]={0x04,0x84,0x04,0x86}; 
	int pnum=(int)(adap->algo_data);
	int act_len=0;
	int i;
	int ret=0;
	akpic_t *dev=akpic_refpic(NULL,pnum);
	//printk("i2c_xfer\n");
	if (!dev)
		return -EREMOTEIO;
	if (dev->is_closed) {
		akpic_unrefpic(dev);
		return -EREMOTEIO;
	}
	//printk("i2c_xfer dev_avail\n");
	while(num) {
		if (dev->subcl != 0xca) 
			akpic_write_data(dev,start_msg,sizeof(start_msg));
		if (msgs->flags&I2C_M_RD) {
			ret=akpic_write_i2c_byte(dev,msgs->addr*2+1,1,msgs->len == 0);
			//printk("written rd addr %02x:%d\n",msgs->addr,ret);
			if (ret) {
				ret=-EREMOTEIO;
				goto fail;
			}
			for(i=0;i<msgs->len;i+=16) {
				int l;
				l=msgs->len-i;
				if (l>16)
					l=16;
				ret=akpic_read_i2c_bytes(dev,msgs->buf+i,l,(l+i)==msgs->len,(l+i)==msgs->len);
				if (ret) {
					ret=-EIO;
					goto fail;
				}
			}
		} else {
			ret=akpic_write_i2c_byte(dev,msgs->addr*2,1,msgs->len == 0);
			//printk("written addr %02x:%d\n",msgs->addr,ret);
			if (ret) {
				ret=-EREMOTEIO;
				goto fail;
			}
			for(i=0;i<msgs->len;i++) {
				ret=akpic_write_i2c_byte(dev,msgs->buf[i],0,(1+i)==msgs->len);
				if (ret) {
					ret=-EREMOTEIO;
					goto fail;
				}
			}
		} 
		num--;
		msgs++;
	}
fail:
	if (dev->subcl != 0xca) 
		akpic_write_data(dev,stop_msg,sizeof(stop_msg));
	else
		akpic_write_data(dev,"\x9e",1);
	//printk("i2c_xfer finished\n");
	akpic_unrefpic(dev);
	//printk("i2c_xfer unrefd\n");
	return ret;
   
}

static u32 akpic_i2c_func(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL ;
}

static void i2c_akpic_add_bus(struct i2c_adapter *ada,
                              akpic_t *dev, int add_devices)
{
	static const struct i2c_algorithm akpic_i2c_algo= {
		.master_xfer=akpic_i2c_xfer,
		.functionality=akpic_i2c_func,
	};
	static struct i2c_board_info akpic_bus_info = {
       		 I2C_BOARD_INFO("bmp085", 0x77),
	};

	struct i2c_adapter *adap=&i2cdevtable[dev->devid];
	*adap=*ada;
	adap->algo=&akpic_i2c_algo;
	adap->timeout=100;
	adap->algo_data=(void *)dev->devid; 
	dev->i2c_ops=adap;
	akpic_write_data(dev,"\xfb",1);
	i2c_add_adapter(adap); 
	if (add_devices) {
		i2c_new_device(adap, &akpic_bus_info);	
	}
}

static int akpic_probe(struct usb_interface *intf,
                       const struct usb_device_id *id)

{
	struct i2c_adapter akpic_i2c_ops={
		.name="akpic-i2c",
		.owner=THIS_MODULE,
		.retries=2,
		.class=0,
	};
	struct usb_device *udev;
	struct usb_interface *intfs;
	int retval = -ENOMEM;
	int i;
	akpic_t *dev;
	printk("akpic_probe\n");
	udev = usb_get_dev(interface_to_usbdev(intf));
	intfs = usb_ifnum_to_if(udev,2);
	retval=-ENODEV;
	if ((!intfs) || (intfs != intf)) {
		printk("wrong iface\n");
		return retval;
	}
	dev = kmalloc(sizeof(akpic_t), GFP_KERNEL);
	if (!dev) {
		return -ENOMEM;
	}
	memset(dev,0, sizeof(*dev));
	dev->udev = udev;
	dev->rcvbuf = kmalloc(64, GFP_KERNEL);
	init_waitqueue_head(&dev->rcv_wait);


	kref_init(&dev->kref);
	usb_driver_claim_interface(&akpic_driver, intfs, NULL);
	usb_set_intfdata(intf, dev);
	printk("registering dev\n");
	retval = usb_register_dev(intf, &akpic_class);
	if (retval) {
		dev_err(&dev->udev->dev,"Not able to get a minor for this device.");
		usb_set_intfdata(intf, NULL);
		goto error;
	}
	printk("allocatint rx_urb\n");
	if ((dev->rx_urb = usb_alloc_urb(0, GFP_KERNEL)) == NULL) {
		goto error;
	}
	set_current_state(TASK_INTERRUPTIBLE);
	schedule_timeout(HZ*3);
	printk("akpic device now at akpic-%d\n", intf->minor);
	for(i=0;i<MAX_DEVS;i++) {
		if (!devtable[i]) {
			dev->devid=i;
			dev->subcl=0;
                        if (intf->cur_altsetting)
				dev->subcl = intf->cur_altsetting->desc.bInterfaceSubClass;
			devtable[i]=dev;
			if ((register_i2c)||(dev->subcl==0xc1)||(dev->subcl==0xca)) {
				i2c_akpic_add_bus(&akpic_i2c_ops,dev,!register_i2c);
			}
			break;
		}
	}
	return 0;
error:
	if (dev->rx_urb) {
		usb_free_urb(dev->rx_urb);
	}
	if (dev) {
		kref_put(&dev->kref, akpic_delete);
	}
	return retval;
}

static void akpic_disconnect(struct usb_interface *interface)
{
	akpic_t *dev;
	int minor = interface->minor;
	//lock_kernel();
	dev = usb_get_intfdata(interface);
	if (dev) {
		usb_set_intfdata(interface, NULL);
		usb_deregister_dev(interface, &akpic_class);
		devtable[dev->devid]=NULL;
		dev->is_closed=1;
	//	unlock_kernel();
		wake_up_interruptible(&dev->rcv_wait);
		if (dev->other_if) {
			usb_driver_release_interface(&akpic_driver, dev->other_if);
		}
		if (dev->i2c_ops) {
			printk("i2c_del_adapter\n");
			i2c_del_adapter(dev->i2c_ops);
			//kfree(dev->i2c_ops);
		}
		kref_put(&dev->kref, akpic_delete);
	} else {
	//	unlock_kernel();
	}
	printk("USB akpic %d now disconnected\n",minor);
}


static int __init usb_akpic_init(void)
{
	printk(DRIVER_DESC " " DRIVER_VERSION "\n");
	return usb_register(&akpic_driver);
}

static void __exit usb_akpic_exit(void)
{
	usb_deregister(&akpic_driver);
}


module_init(usb_akpic_init);
module_exit(usb_akpic_exit);

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
module_param(register_i2c,int,0);
MODULE_LICENSE("GPL");

/*
 * Overrides for Emacs so that we follow Linus's tabbing style.
 * Emacs will notice this stuff at the end of the file and automatically
 * adjust the settings for this buffer only.  This must remain at the end
 * of the file.
 * ---------------------------------------------------------------------------
 * Local variables:
 * c-file-style: "linux"
 * End:
 */

