#include <linux/sched.h>
#include <linux/init.h>
#include <linux/signal.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/usb.h>
#include <linux/version.h>
#include <asm/uaccess.h>

#define DRIVER_VERSION "v0.1"
#define DRIVER_AUTHOR "Andreas Kemnade <andreas@kemnade.info>"
#define DRIVER_DESC "Bike data streaming (currently speed information)"
#define USBAKBIKE_MINOR 170
#define USBAKBIKECTRL_MINOR 190
static const char driver_name [] = "akbike";

static struct usb_device_id id_table [] = {
	{ USB_DEVICE_AND_INTERFACE_INFO(0x16d0, 0x04d9,0xff,0xc2,0)},
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

typedef struct {
	char buf[256];
	char *rcvbuf;
	int inpos;
	int outpos;
	char sndbuf[8];
	int intfnum;
	struct usb_device *udev;
	struct urb *rx_urb;
	struct urb *tx_urb;
	wait_queue_head_t rcv_wait;
	int epin;
	int open_cnt;
	int is_closed;
	struct kref             kref;
} akbike_t;

static int akbike_open(struct inode *inode, struct file *file);
static int akbike_write(struct file *file,
			const char __user *user_buffer, size_t count, loff_t *ppos);
static int akbike_read(struct file *file, char __user *buffer,
		       size_t count, loff_t *ppos);
static int akbike_release(struct inode *inode, struct file *file);
static int akbike_ioctl(struct inode *inode, struct file *file,
			unsigned int cmd, unsigned long arg);
static struct file_operations akbike_fops = {
	.owner = THIS_MODULE,
	.read = akbike_read,
	.write = akbike_write,
	.open = akbike_open,
	.release = akbike_release,
};

static struct usb_class_driver akbike_class = {
	.name = "akbike%d",
	.fops = &akbike_fops,
	.minor_base = USBAKBIKE_MINOR,
};

static void akbike_read_bulk_callback(struct urb *urb)
{
	akbike_t *dev = (akbike_t *)urb->context;
	int cp_len = urb->actual_length;
	if (!dev)
		return;
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
	printk("got %d bytes\n",cp_len);
	if ((dev->inpos+cp_len) >=sizeof(dev->buf)) {
		int rest=sizeof(dev->buf)-dev->inpos;
		memcpy(dev->buf+dev->inpos,urb->transfer_buffer, rest);
		memcpy(dev->buf,urb->transfer_buffer+rest,cp_len-rest);
		dev->inpos=cp_len-rest;
	} else {
		//printk("akpic copy 4");
		memcpy(dev->buf+dev->inpos, urb->transfer_buffer, cp_len);
		//printk("akpic copy 5");
		dev->inpos+=cp_len;
	}
	
goon:
	
	usb_fill_bulk_urb(dev->rx_urb, dev->udev,
			  usb_rcvbulkpipe(dev->udev, dev->epin),
			  dev->rcvbuf, 64, akbike_read_bulk_callback, dev);
	
	usb_submit_urb(dev->rx_urb, GFP_ATOMIC);
	return;
	
}


static void akbike_delete(struct kref *kref)
{
	akbike_t *dev = container_of(kref, akbike_t, kref);
	printk("akbike delete_dev");
	usb_put_dev(dev->udev);
	kfree(dev->rcvbuf);
	kfree(dev);
}

static int akbike_open(struct inode *inode, struct file *file)
{
	akbike_t *dev;
	struct usb_interface *interface;
	int subminor;
	int res;
	int retval = 0;
	subminor = iminor(inode);
	interface = usb_find_interface(&akbike_driver, subminor);
	if (!interface) {
		retval = -ENODEV;
		goto exit;
	}
	dev = usb_get_intfdata(interface);
	if (!dev) {
		retval = -ENODEV;
		goto exit;
	}
	kref_get(&dev->kref);
	file->private_data = dev;
	//lock_kernel();
	if (! dev->open_cnt) {
		dev->open_cnt++;
		usb_fill_bulk_urb(dev->rx_urb, dev->udev,
				  usb_rcvbulkpipe(dev->udev, dev->epin),
				  dev->rcvbuf, 64, akbike_read_bulk_callback, dev);
		if ((res = usb_submit_urb(dev->rx_urb, GFP_KERNEL))) {
			printk("submit rx urb failed\n");
		}
	} else {
		dev->open_cnt++;
	}
	//unlock_kernel();
exit:
	return retval;
}

static int akbike_release(struct inode *inode, struct file *file)
{
	akbike_t *dev;
	dev = (akbike_t *)file->private_data;
	if (dev == NULL)
		return -ENODEV;
	printk("akbike: closing dev\n");
	//lock_kernel();
	dev->open_cnt--;
	if (!dev->open_cnt) {
	//	unlock_kernel();
		usb_kill_urb(dev->rx_urb);
	} else {
	//	unlock_kernel();
	}
	kref_put(&dev->kref, akbike_delete);
	return 0;
}

static int akbike_read(struct file *file, char __user *buffer,
		       size_t count, loff_t *ppos)
{
	akbike_t *dev = (akbike_t *)file->private_data;
	int retval = 0;
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
			printk("interrupted in akbike_read %d %d\n",dev->inpos,dev->outpos);
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

static int akbike_write(struct file *file,
			const char __user *user_buffer, size_t count, loff_t *ppos)
{
	return -EIO; 
}

static int akbike_ioctl(struct inode *inode, struct file *file,
			unsigned int cmd, unsigned long arg)
{
	akbike_t *dev = (akbike_t *)file->private_data;
	if (dev == NULL) {
		return -ENODEV;
	}
	return -ENOTTY;
}


static int akbike_probe(struct usb_interface *intf,
			const struct usb_device_id *id)

{
	struct usb_device *udev = usb_get_dev(interface_to_usbdev(intf));
	int retval = -ENOMEM;
	akbike_t *dev;
	struct usb_host_endpoint *ep;
	int epin=-1;
	int i;
	if (!intf->cur_altsetting)
		return -ENODEV;
	ep=intf->cur_altsetting->endpoint;
	for(i=0;i<intf->cur_altsetting->desc.bNumEndpoints;i++) {
		if (usb_endpoint_dir_in(&ep[i].desc)) {
			epin=usb_endpoint_num(&ep[i].desc);
		}
	}
	if (epin<0)
		return -ENODEV;
	dev = kmalloc(sizeof(akbike_t), GFP_KERNEL);
	if (!dev) {
		return -ENOMEM;
	}
	memset(dev,0, sizeof(*dev));
	dev->udev = udev;
	dev->epin = epin;
	dev->rcvbuf = kmalloc(64, GFP_KERNEL);
	
	if (!dev->rcvbuf) {
		kfree(dev);
		return -ENOMEM;
	}
	init_waitqueue_head(&dev->rcv_wait);
	kref_init(&dev->kref);
	usb_driver_claim_interface(&akbike_driver, intf, NULL);
	
	usb_set_intfdata(intf, dev);
	retval = usb_register_dev(intf, &akbike_class);
	if (retval) {
		dev_err(&dev->udev->dev,"Not able to get a minor for this device.");
		usb_set_intfdata(intf, NULL);
		goto error;
	}
	
	if ((dev->rx_urb = usb_alloc_urb(0, GFP_KERNEL)) == NULL) {
		goto error;
	}
	printk("akbike device now at akbike-%d\n", intf->minor);
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
	int minor = interface->minor;
	//lock_kernel();
	dev = usb_get_intfdata(interface);
	if (dev) {
		usb_set_intfdata(interface, NULL);
		usb_deregister_dev(interface, &akbike_class);
		dev->is_closed=1;
	//	unlock_kernel();
		wake_up_interruptible(&dev->rcv_wait);
		kref_put(&dev->kref, akbike_delete);
	} else {
	//	unlock_kernel();
	}
	printk("USB akbike %d now disconnected\n",minor);
}


static int __init usb_akbike_init(void)
{
	printk(DRIVER_DESC " " DRIVER_VERSION "\n");
	return usb_register(&akbike_driver);
}

static void __exit usb_akbike_exit(void)
{
	usb_deregister(&akbike_driver);
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
