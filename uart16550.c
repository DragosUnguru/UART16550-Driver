#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/miscdevice.h>
#include <linux/kfifo.h>
#include <asm/ioctl.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>

#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/wait.h>

#include "uart16550.h"

#define DEFAULT_MAJOR       42
#define MODULE_NAME         "uart16550"
#define COM1				0
#define COM2				1

#define COM1_BASE_PORT		0x3f8
#define COM2_BASE_PORT		0x2f8
#define NUM_ADDRESSES		8

#define IRQ_COM1			4
#define IRQ_COM2			3

#define LSR_OFFSET			5
#define MCR_OFFSET			4
#define LCR_OFFSET			3
#define ISR_OFFSET			2
#define FCR_OFFSET			2
#define IER_OFFSET			1

#ifndef BUFSIZ
#define BUFSIZ				128
#endif

MODULE_DESCRIPTION("Driver UART");
MODULE_AUTHOR("Burcea Vlad + Dragos Unguru <3");
MODULE_LICENSE("GPL");

int major = DEFAULT_MAJOR, option = OPTION_BOTH;
int num_minors = MAX_NUMBER_DEVICES, minor;

const int BASE_PORTS[2] = { COM1_BASE_PORT, COM2_BASE_PORT };
const int IRQS[2] = { IRQ_COM1, IRQ_COM2 };

module_param(major, int, 0660);
MODULE_PARM_DESC(major, "The major");
module_param(option, int, 0660);
MODULE_PARM_DESC(myshort, "The option");

struct uart_device_data {
	struct cdev cdev;
	DECLARE_KFIFO(read_buffer, u8, BUFSIZ);
	DECLARE_KFIFO(write_buffer, u8, BUFSIZ);
	wait_queue_head_t wq_read, wq_write;
} devs[MAX_NUMBER_DEVICES];

irqreturn_t uart_interrupt_handle(int irq_no, void *dev_id)
{
	struct uart_device_data *dev;
	int baseport, lsr, isr;
	u8 ch;

	dev = (struct uart_device_data *) dev_id;
	if (irq_no == IRQ_COM1)
		baseport = BASE_PORTS[COM1];
	else if (irq_no == IRQ_COM2)
		baseport = BASE_PORTS[COM2];
	else
		return IRQ_HANDLED;

	isr = inb(baseport + ISR_OFFSET);

	if (isr & 0x4) {
		lsr = baseport + LSR_OFFSET;

		// Disable read interrupts
		outb(inb(baseport + IER_OFFSET) & ~0x01, baseport + IER_OFFSET);

		while ((inb(lsr) & 1) && !kfifo_is_full(&dev->read_buffer)) {
			ch = inb(baseport);
			kfifo_in(&dev->read_buffer, &ch, sizeof(ch));
		}

		wake_up(&dev->wq_read);
	}

	if (isr & 0x2) {
		// Disable write interrupts
		outb(inb(baseport + IER_OFFSET) & ~0x02, baseport + IER_OFFSET);

		while (!kfifo_is_empty(&dev->write_buffer)) {
			kfifo_out(&dev->write_buffer, &ch, sizeof(ch));
			outb(ch, baseport);
		}

		wake_up(&dev->wq_write);
	}

	return IRQ_HANDLED;
}

static ssize_t uart_cdev_read(struct file *file, char __user *user_buffer,
				size_t size, loff_t *offset)
{
	struct uart_device_data *data;
	size_t to_read = size;
	int err, ier, read_bytes;

	data = (struct uart_device_data *) file->private_data;

	if (!size)
		return 0;

	err = wait_event_interruptible(data->wq_read,
			!kfifo_is_empty(&data->read_buffer));
	if (err) {
		pr_info("[READ] FAULT: wait_event_interruptible");
		return -ERESTARTSYS;
	}

	if (kfifo_len(&data->read_buffer) < to_read)
		to_read = kfifo_len(&data->read_buffer);

	err = kfifo_to_user(&data->read_buffer, user_buffer,
			to_read, &read_bytes);
	if (err) {
		pr_info("[READ] FAULT: kfifo_to_user");
		return -EFAULT;
	}

	// Enable read interrupt
	ier = BASE_PORTS[MINOR(data->cdev.dev)] + IER_OFFSET;
	outb(inb(ier) | 1, ier);

	return read_bytes;
}

static ssize_t uart_cdev_write(struct file *file,
				   const char __user *user_buffer, size_t size,
				   loff_t *offset)
{
	struct uart_device_data *data;
	size_t to_write = size;
	int err, ier, written_bytes;

	data = (struct uart_device_data *) file->private_data;

	if (!size)
		return 0;

	err = kfifo_from_user(&data->write_buffer, user_buffer,
			to_write, &written_bytes);
	if (err) {
		pr_info("[WRITE] FAULT: kfifo_in_locked");
		return -EFAULT;
	}

	// Enable write interrupts
	ier = BASE_PORTS[MINOR(data->cdev.dev)] + IER_OFFSET;
	outb(inb(ier) | 2, ier);

	err = wait_event_interruptible(data->wq_write,
		!kfifo_is_full(&data->write_buffer));

	return written_bytes;
}

static long uart_cdev_ioctl(struct file *file,
	unsigned int cmd, unsigned long arg)
{
	struct uart16550_line_info info;
	struct uart_device_data *dev;

	dev = (struct uart_device_data*) file->private_data;

	if (cmd == UART16550_IOCTL_SET_LINE) {
		if (copy_from_user(&info, (const void *) arg, sizeof(info)))
			return -EFAULT;

		outb(0x80 | info.len | info.stop | info.par,
			BASE_PORTS[MINOR(dev->cdev.dev)] + LCR_OFFSET);
		outb((u8) info.baud, BASE_PORTS[MINOR(dev->cdev.dev)]);
		outb(0x0, BASE_PORTS[MINOR(dev->cdev.dev)] + LCR_OFFSET);

		return 0;
	}

	return -EINVAL;
}

static int uart_cdev_release(struct inode *inode, struct file *file)
{
	return 0;
}

static int uart_cdev_open(struct inode *inode, struct file *file)
{
	file->private_data = container_of(inode->i_cdev,
			struct uart_device_data, cdev);
	return 0;
}

static const struct file_operations uart_fops = {
	.owner = THIS_MODULE,
	.open = uart_cdev_open,
	.release = uart_cdev_release,
	.read = uart_cdev_read,
	.write = uart_cdev_write,
	.unlocked_ioctl = uart_cdev_ioctl,
};

int uart_cdev_init(void)
{
	int err;
	int i;

	if (option == OPTION_COM1)
		num_minors = 1;
	else if (option == OPTION_COM2) {
		num_minors = 1;
		minor = 1;
	} else if (option != OPTION_BOTH)
		return -EFAULT;

	err = register_chrdev_region(MKDEV(major, minor), num_minors,
					MODULE_NAME);
	if (err) {
		pr_info("register_chrdev_region");
		return err;
	}

	for (i = minor; i < minor + num_minors; i++) {
		init_waitqueue_head(&devs[i].wq_read);
		init_waitqueue_head(&devs[i].wq_write);

		INIT_KFIFO(devs[i].read_buffer);
		INIT_KFIFO(devs[i].write_buffer);

		cdev_init(&devs[i].cdev, &uart_fops);
		err = cdev_add(&devs[i].cdev, MKDEV(DEFAULT_MAJOR, i), 1);

		if (err)
			goto unregister;

		if (!request_region(BASE_PORTS[i], NUM_ADDRESSES, MODULE_NAME))
			goto unregister;

		err = request_irq(IRQS[i], uart_interrupt_handle, IRQF_SHARED,
				  MODULE_NAME, &devs[i].cdev);
		if (err)
			goto unregister;

		outb(0xc7, BASE_PORTS[i] + FCR_OFFSET);

		// Enable interrupts
		outb(0x03, BASE_PORTS[i] + IER_OFFSET);
		// FIFO enable
		outb(0x01, BASE_PORTS[i] + FCR_OFFSET);
	}

	return 0;

unregister:
	for (i = minor; i < minor + num_minors; i++)
		cdev_del(&devs[i].cdev);

	unregister_chrdev_region(MKDEV(DEFAULT_MAJOR, minor), num_minors);
	return -EFAULT;
}

static void uart_cdev_exit(void)
{
	int i;

	for (i = minor; i < minor + num_minors; i++) {
		outb(0, BASE_PORTS[i] + IER_OFFSET);
		outb(0, BASE_PORTS[i] + MCR_OFFSET);
		free_irq(IRQS[i], &devs[i]);
		release_region(BASE_PORTS[i], NUM_ADDRESSES);
		cdev_del(&devs[i].cdev);
	}

	unregister_chrdev_region(MKDEV(DEFAULT_MAJOR, minor), num_minors);
}


module_init(uart_cdev_init);
module_exit(uart_cdev_exit);