#if 0
#include <linux/config.h>
#endif
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/signal.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/usb.h>
#include <linux/workqueue.h>
#include <linux/power_supply.h>
#include <linux/version.h>
#include <asm/uaccess.h>

#define DRIVER_VERSION "v0.1"
#define DRIVER_AUTHOR "Andreas Kemnade <andreas@kemnade.info>"
#define DRIVER_DESC "Bike controller power"
static const char driver_name [] = "akbikepower";
static  struct device *psp_dev;
static	struct power_supply_desc *psp_desc;
static	struct power_supply *psp_glob;
static	int vinuv;
static	int vctluv;
static	int vin,vctl;
static  int tachoperiod;
static  int do_reset;
static  unsigned char powerswitchcmd;

static struct usb_device_id id_table [] = {
	{ USB_DEVICE_AND_INTERFACE_INFO(0x16d0, 0x04d9,0xff,0xc3,0)},
	{}
};

MODULE_DEVICE_TABLE (usb, id_table);


static int akbike_probe(struct usb_interface *intf,
			const struct usb_device_id *id);
static void akbike_disconnect(struct usb_interface *interface);
static struct usb_driver akbike_driver = {
	.name = driver_name,
	.probe =   akbike_probe,
	.disconnect = akbike_disconnect,
	.id_table = id_table
};

static enum power_supply_property bikepower_props[]={
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_PRESENT,
};

typedef struct {
	char buf[8];
	char *rcvbuf;
	int inpos;
	int outpos;
	char sndbuf[8];
	enum {
		INIT_ADVIN=0,
		READ_VIN,
		READ_VIN2,
		INIT_ADVCTL,
		READ_VCTL1,
		READ_VCTL2,
	} work_state;
	struct usb_device *udev;
	struct urb *rx_urb;
	int open_cnt;
	int is_closed;
        int devver;
	int epin,epout;
	struct delayed_work work;
	struct kref kref;
} akbike_t;

static unsigned int cache_time = 2000;
module_param(cache_time, uint, 0644);
MODULE_PARM_DESC(cache_time,"cache time in ms");

static void akbike_read_bulk_callback(struct urb *urb)
{
	akbike_t *dev = (akbike_t *)urb->context;
	int cp_len = urb->actual_length;
	u8 *tb;
	u32 t,t2;
	u64 t64;
	if (!dev)
		return;
	if (dev->is_closed)
		return;
	switch(urb->status) {
	case 0:
		break;
	case -ENOENT:
		vin=0;
		vinuv=0;
		vctl=0;
		vctluv=0;
		return;
	default:
		vin=0;
		vinuv=0;
		vctl=0;
		vctluv=0;
		dev_dbg(&dev->udev->dev,"urb rx status: %d\n",urb->status);
	schedule_delayed_work(&dev->work,HZ/10);
	return;
	}
	dev_dbg(&dev->udev->dev,"got %d bytes\n",cp_len);
	tb=urb->transfer_buffer;
	t=tb[1]*256+tb[2];
        if (dev->devver > 2) {
           t2=tb[3]*256+tb[4];
           if (vin != 0) {
             vin=vin+t;
           } else {
             vin=t*2;
           }
           t64=vin;
           t64=t64*(33*(20+1)*100*1000)/131072;
           vinuv=(int)t64;
           vin/=2;
           if (vctl != 0) {
             vctl += t2;
           } else {
             vctl=t2*2;
           }
	   t64=vctl;
           t64=t64*33*2*100*1000/131072;
           vctluv=(int)t64;
           vctl/=2;
           t = (tb[6]*256+tb[7])*256+tb[8];
           if (t > 65536) { /* skip errornous measurements giving insane high speeds */
             tachoperiod=t*2;
           }
	   schedule_delayed_work(&dev->work,cache_time*HZ/1000);
	   return;
        } else {
	switch(dev->work_state) {
	case READ_VIN:
		vin=t;
		dev->work_state=READ_VIN2;
		break;
	case READ_VIN2:
		vin+=t;
		t64=vin;
		/* vinuv=dev->vin*3.3/131071*(47+5.6)/5.6*pow(1,6) */
		/* 56*131071); */
                if (dev->devver == 1)  {
		  t64=t64*(33*(470+56)*100*1000)/131072;
		  vinuv=(int)t64;
		  vinuv/=56;
                } else {
                }
		dev->work_state=INIT_ADVCTL;
		break;
	case READ_VCTL1:
		vctl=t;
		dev->work_state=READ_VCTL2;
		break;
	case READ_VCTL2:
		vctl+=t;
		t64=vctl;
                if (dev->devver==1) {
		  t=4294967295U/vctl;
		
		// t64=(u64)33*100L*1000L*131071L/t64;
		// dev->vctluv=(int)t64;
		  vctluv=t*101;
                } else {
                  t64=t64*33*2*100*1000/131072;
                  vctluv=(int)t64;
                }
		dev->work_state=INIT_ADVIN;
		break;
	default:
		break;
	}
	schedule_delayed_work(&dev->work,HZ/10);
	return;
        }
	
}


static void akbike_delete(struct kref *kref)
{
	akbike_t *dev = container_of(kref, akbike_t, kref);
	dev_dbg(&dev->udev->dev,"akbike delete_dev");
	usb_put_dev(dev->udev);
	kfree(dev->rcvbuf);
	kfree(dev);
}


static void akbike_write_bulk_callback(struct urb *urb)

{
	/* akbike_t *dev = (akbike_t *)urb->context; */
	usb_free_coherent(urb->dev, urb->transfer_buffer_length,
			urb->transfer_buffer, urb->transfer_dma);
	
}
int akbike_write(akbike_t *dev, char *bufsrc, int count)
{
	int retval = 0;
	struct urb *urb = NULL;
	char *buf = NULL;
	urb = usb_alloc_urb(0, GFP_KERNEL);
	if (count >8)
		count=8;
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
	memcpy(buf,bufsrc,count);
	usb_fill_bulk_urb(urb, dev->udev, usb_sndbulkpipe(dev->udev, dev->epout), buf, count,
			  akbike_write_bulk_callback, dev);
	urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
	retval = usb_submit_urb(urb, GFP_KERNEL);
	if (retval) {
		dev_err(&dev->udev->dev,"akbike: failed sending data, error %d", retval);
		goto error;
	}
	usb_free_urb(urb);
	return count;
error:
	usb_free_coherent(dev->udev, count, buf, urb->transfer_dma);
	usb_free_urb(urb);
	return retval;
	
}

static ssize_t set_powerswitch(struct device *ddev,
			    struct device_attribute *attr,
			    const char *buf,size_t count) {
   int s, onoff;
   if (2 != sscanf(buf, "%d %d",&s, &onoff)) {
     return -EINVAL;
   }
   if ((s < 0) || (s >3) || ((onoff != 1) && (onoff != 0)))  {
     return -EINVAL;
   }
   powerswitchcmd = (s << 1) + 1 - onoff + 0x42;
   return count;
}

/* this causes a reset of the microcontroller on the bike board */
static ssize_t do_usb_reset(struct device *ddev,
			    struct device_attribute *attr,
			    const char *buf,size_t count)
{
        do_reset=1;
        return count;
}
static DEVICE_ATTR(reset, S_IWUSR, NULL, do_usb_reset);
static DEVICE_ATTR(power_switches, S_IWUSR, NULL, set_powerswitch);

static ssize_t show_vctrl_voltage(struct device *ddev,
				  struct device_attribute *attr,
				  char *buf)
{
/*	akbike_t *dev = dev_get_drvdata(ddev); */
	return sprintf(buf,"%u\n",vctluv);
}
static ssize_t show_tacho_period(struct device *ddev,
			         struct device_attribute *attr,
			         char *buf)
{
/*	akbike_t *dev = dev_get_drvdata(ddev); */
        return sprintf(buf,"%u\n",tachoperiod);
}

static DEVICE_ATTR(vctrl_voltage, S_IRUGO, show_vctrl_voltage, NULL);
static DEVICE_ATTR(tacho_period, S_IRUGO, show_tacho_period, NULL);

static struct attribute *bikepower_sysfs_entries[] = {
	&dev_attr_vctrl_voltage.attr,
	&dev_attr_reset.attr,
	&dev_attr_power_switches.attr,
        &dev_attr_tacho_period.attr,
	NULL
};

static struct attribute_group bikepower_attr_group = {
	.name = NULL,
	.attrs = bikepower_sysfs_entries
};

static int bikepower_get_property(struct power_supply *psp,
				  enum power_supply_property prop,
				  union power_supply_propval *val)
{
	switch(prop) {
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval=(vinuv>0);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval=vinuv;
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval=vinuv>4800000;
		break;
	default:
		return -EINVAL;
		break;
	}
	return 0;
}

static void bikepower_work(struct work_struct *work)
{
	akbike_t *dev=container_of(work,akbike_t,work.work);
	if (dev->is_closed)
		return;
	dev_dbg(&dev->udev->dev,"work state: %d\n",dev->work_state);
        if (dev->devver > 2) {
		usb_fill_bulk_urb(dev->rx_urb, dev->udev,
				  usb_rcvbulkpipe(dev->udev, dev->epin),
				  dev->rcvbuf, 16, akbike_read_bulk_callback, dev);
		usb_submit_urb(dev->rx_urb, GFP_ATOMIC);
	  akbike_write(dev,"\x30",1);
        } else {
	switch(dev->work_state) {
	case INIT_ADVIN:
		akbike_write(dev,"\x61",1);
		dev->work_state=READ_VIN;
		schedule_delayed_work(&dev->work,cache_time*HZ/1000);
		break;
	case READ_VIN:
	case READ_VIN2:
		usb_fill_bulk_urb(dev->rx_urb, dev->udev,
				  usb_rcvbulkpipe(dev->udev, dev->epin),
				  dev->rcvbuf, 8, akbike_read_bulk_callback, dev);
		usb_submit_urb(dev->rx_urb, GFP_ATOMIC);
		akbike_write(dev,"\x41",1);
		break;
	case INIT_ADVCTL:
		akbike_write(dev,"\x64",1);
		schedule_delayed_work(&dev->work,10);
		dev->work_state=READ_VCTL1;
		break;
	case READ_VCTL1:
	case READ_VCTL2:
		usb_fill_bulk_urb(dev->rx_urb, dev->udev,
		      usb_rcvbulkpipe(dev->udev, 1),
				  dev->rcvbuf, 8, akbike_read_bulk_callback, dev);
		usb_submit_urb(dev->rx_urb, GFP_ATOMIC);
		akbike_write(dev,"\x41",1);
		break;
	}
        }
        if (do_reset) {
          do_reset=0; 
	  akbike_write(dev,"e",1);
        } else if (powerswitchcmd) {
          akbike_write(dev,&powerswitchcmd,1);
          powerswitchcmd=0; 
        } 
}


static int akbike_probe(struct usb_interface *intf,
			const struct usb_device_id *id)
	
{
	struct usb_device *udev = usb_get_dev(interface_to_usbdev(intf));
	int retval = -ENOMEM;
	int epin=-1;
	int epout=-1;
	int i;
	struct usb_host_endpoint *ep;
	akbike_t *dev;
	if (!intf->cur_altsetting)
		return -ENODEV;
	ep=intf->cur_altsetting->endpoint;
	for(i=0;i<intf->cur_altsetting->desc.bNumEndpoints;i++) {
		if (usb_endpoint_dir_in(&ep[i].desc)) {
			epin=usb_endpoint_num(&ep[i].desc);
		} else {
			epout=usb_endpoint_num(&ep[i].desc);
		}
	}
	if ((epin<0)||(epout<0))
		return -ENODEV;
	dev = kmalloc(sizeof(akbike_t), GFP_KERNEL);
	if (!dev) {
		return -ENOMEM;
	}
	memset(dev,0, sizeof(*dev));
	dev->udev = udev;
	dev->epout = epout;
	dev->epin = epin;
        dev->devver = __le16_to_cpu(interface_to_usbdev(intf)->descriptor.bcdDevice);
	dev->rcvbuf = kmalloc(64, GFP_KERNEL);
	return 0;
error:
	if (dev->rx_urb) {
		usb_free_urb(dev->rx_urb);
	}
	if (dev) {
		kref_put(&dev->kref, akbike_delete);
	}
	return retval;
}

static void akbike_disconnect(struct usb_interface *interface)
{
	akbike_t *dev;
	/* lock_kernel(); */
	dev = usb_get_intfdata(interface);
	if (dev) {
		dev->is_closed=1;
		usb_set_intfdata(interface, NULL);
		cancel_delayed_work(&dev->work);
		/* unlock_kernel(); */
		cancel_delayed_work_sync(&dev->work);
		
		usb_kill_urb(dev->rx_urb);
	       
		dev_set_drvdata(psp_dev,NULL);
		kref_put(&dev->kref, akbike_delete);
	} else {
		/* unlock_kernel(); */
	}
	dev_notice(&interface->dev,"USB akbike power supply now disconnected\n");
	vinuv=vin=vctl=vctluv=0;
}


static int __init usb_akbike_init(void)
{
	int retval;
	printk(DRIVER_DESC " " DRIVER_VERSION "\n");
        struct power_supply_config psy_cfg = { .drv_data = NULL, }; /* fill it again */

	psp_dev=root_device_register("bikepower");
	/*kzalloc(sizeof(struct device),GFP_KERNEL); 
	  device_initialize(psp_dev); */
	psp_desc=kzalloc(sizeof(struct power_supply_desc),GFP_KERNEL);
	psp_desc->name = "bike";  
	psp_desc->type = POWER_SUPPLY_TYPE_MAINS;
	psp_desc->properties = bikepower_props;
	psp_desc->num_properties = ARRAY_SIZE(bikepower_props);
	psp_desc->get_property = bikepower_get_property;
	psp_glob = power_supply_register(psp_dev,psp_desc,&psy_cfg);
	if (NULL == psp_glob) {
		dev_err(psp_dev,"Not able to register powerdev.");
		kfree(psp_desc);
                psp_desc = NULL;
	}
	
	if (sysfs_create_group(&psp_dev->kobj,
			       &bikepower_attr_group)) {
		dev_err(psp_dev,"failed to create sysfs entries");
	}
	retval=usb_register(&akbike_driver);
	return retval;
	
}

static void __exit usb_akbike_exit(void)
{
	usb_deregister(&akbike_driver);
	if (psp_glob) {
		power_supply_unregister(psp_glob);
		kfree(psp_desc);
	}
	if (psp_dev) {
		sysfs_remove_group(&psp_dev->kobj, &bikepower_attr_group);
		root_device_unregister(psp_dev);
	}
}


module_init(usb_akbike_init);
module_exit(usb_akbike_exit);

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
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
