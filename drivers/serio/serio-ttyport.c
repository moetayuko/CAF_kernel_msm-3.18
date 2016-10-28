/*
 * Input device TTY line discipline
 *
 * Copyright (c) 1999-2002 Vojtech Pavlik
 *
 * This is a module that converts a tty line into a much simpler
 * 'serial io port' abstraction that the input device drivers use.
 */

/*
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#include <asm/uaccess.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/serio.h>
#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <linux/of.h>
#include <linux/list.h>

MODULE_AUTHOR("Vojtech Pavlik <vojtech@ucw.cz>");
MODULE_DESCRIPTION("Input device TTY line discipline");
MODULE_LICENSE("GPL");

#define SERPORT_BUSY	1
#define SERPORT_ACTIVE	2
#define SERPORT_DEAD	3

struct serport {
	struct serio serio;
	struct tty_port *port;
	struct tty_struct *tty;
	struct tty_driver *tty_drv;
	int tty_idx;
	spinlock_t lock;
	unsigned long flags;
	struct list_head serport_list;
};

static LIST_HEAD(serio_tty_ports);

/*
 * Callback functions from the serio code.
 */

static int serport_serio_write_buf(struct serio *serio, const unsigned char *data, size_t len)
{
	struct serport *serport = serio->port_data;
	struct tty_struct *tty = serport->tty;

	set_bit(TTY_DO_WRITE_WAKEUP, &tty->flags);
	return serport->tty->ops->write(serport->tty, data, len);
}

static void serport_serio_write_flush(struct serio *serio)
{
	struct serport *serport = serio->port_data;
	struct tty_struct *tty = serport->tty;

	tty_driver_flush_buffer(tty);
}

static int serport_serio_open(struct serio *serio)
{
	struct serport *serport = serio->port_data;
	unsigned long flags;

	serport->tty = tty_init_dev(serport->tty_drv, serport->tty_idx);

	if (serio_buffered_mode_enabled(serio))
		serport->tty->receive_room = 65536;

	if (serport->tty->ops->open)
		serport->tty->ops->open(serport->tty, NULL);
	else
		tty_port_open(serport->port, serport->tty, NULL);

	set_bit(TTY_DO_WRITE_WAKEUP, &serport->tty->flags);

	spin_lock_irqsave(&serport->lock, flags);
	set_bit(SERPORT_ACTIVE, &serport->flags);
	spin_unlock_irqrestore(&serport->lock, flags);

	return 0;
}

static void serport_serio_close(struct serio *serio)
{
	struct serport *serport = serio->port_data;
	struct tty_struct *tty = serport->tty;
	unsigned long flags;

	spin_lock_irqsave(&serport->lock, flags);

	if (tty->ops->close)
		tty->ops->close(tty, NULL);

	tty_release_struct(tty, serport->tty_idx);

	clear_bit(SERPORT_ACTIVE, &serport->flags);
	spin_unlock_irqrestore(&serport->lock, flags);
}

static unsigned int serport_serio_set_baudrate(struct serio *serio, unsigned int speed)
{
	struct serport *serport = serio->port_data;
	struct tty_struct *tty = serport->tty;
	struct ktermios ktermios;

	ktermios = tty->termios;
	ktermios.c_cflag &= ~CBAUD;
	tty_termios_encode_baud_rate(&ktermios, speed, speed);

	/* tty_set_termios() return not checked as it is always 0 */
	tty_set_termios(tty, &ktermios);
	return speed;
}

static void serport_serio_set_flow_control(struct serio *serio, bool enable)
{
	struct serport *serport = serio->port_data;
	struct tty_struct *tty = serport->tty;
	struct ktermios ktermios;
	int status;
	unsigned int set = 0;
	unsigned int clear = 0;

	if (enable) {
		/* Disable hardware flow control */
		ktermios = tty->termios;
		ktermios.c_cflag &= ~CRTSCTS;
		status = tty_set_termios(tty, &ktermios);
		dev_dbg(&serio->dev, "Disabling hardware flow control: %s",
			status ? "failed" : "success");

		/* Clear RTS to prevent the device from sending */
		/* Most UARTs need OUT2 to enable interrupts */
		status = tty->driver->ops->tiocmget(tty);
		dev_dbg(&serio->dev, "Current tiocm 0x%x", status);

		set &= ~(TIOCM_OUT2 | TIOCM_RTS);
		clear = ~set;
		set &= TIOCM_DTR | TIOCM_RTS | TIOCM_OUT1 |
		       TIOCM_OUT2 | TIOCM_LOOP;
		clear &= TIOCM_DTR | TIOCM_RTS | TIOCM_OUT1 |
			 TIOCM_OUT2 | TIOCM_LOOP;
		status = tty->driver->ops->tiocmset(tty, set, clear);
		dev_dbg(&serio->dev, "Clearing RTS: %s", status ? "failed" : "success");
	} else {
		/* Set RTS to allow the device to send again */
		status = tty->driver->ops->tiocmget(tty);
		dev_dbg(&serio->dev, "Current tiocm 0x%x", status);

		set |= (TIOCM_OUT2 | TIOCM_RTS);
		clear = ~set;
		set &= TIOCM_DTR | TIOCM_RTS | TIOCM_OUT1 |
		       TIOCM_OUT2 | TIOCM_LOOP;
		clear &= TIOCM_DTR | TIOCM_RTS | TIOCM_OUT1 |
			 TIOCM_OUT2 | TIOCM_LOOP;
		status = tty->driver->ops->tiocmset(tty, set, clear);
		dev_dbg(&serio->dev, "Setting RTS: %s", status ? "failed" : "success");

		/* Re-enable hardware flow control */
		ktermios = tty->termios;
		ktermios.c_cflag |= CRTSCTS;
		status = tty_set_termios(tty, &ktermios);
		dev_dbg(&serio->dev, "Enabling hardware flow control: %s",
			status ? "failed" : "success");
	}

}

static int serport_receive_buf(struct tty_port *port, const unsigned char *cp,
				const unsigned char *fp, size_t count)
{
	struct serport *serport = port->client_data;
	unsigned long flags;
	unsigned int ch_flags = 0;
	int i = 0;

	spin_lock_irqsave(&serport->lock, flags);

	if (!test_bit(SERPORT_ACTIVE, &serport->flags))
		goto out;

	if (serio_buffered_mode_enabled(&serport->serio)) {
		serio_receive_buf(&serport->serio, cp, count);
		i = count;
	} else {
		for (i = 0; i < count; i++) {
			if (fp) {
				switch (fp[i]) {
				case TTY_FRAME:
					ch_flags = SERIO_FRAME;
					break;

				case TTY_PARITY:
					ch_flags = SERIO_PARITY;
					break;

				default:
					ch_flags = 0;
					break;
				}
			}

			serio_interrupt(&serport->serio, cp[i], ch_flags);
		}
	}
out:
	spin_unlock_irqrestore(&serport->lock, flags);
	return i;
}

static void serport_write_wakeup(struct tty_port *port)
{
	struct serport *serport = port->client_data;
	unsigned long flags;

	clear_bit(TTY_DO_WRITE_WAKEUP, &port->tty->flags);

	spin_lock_irqsave(&serport->lock, flags);
	if (test_bit(SERPORT_ACTIVE, &serport->flags))
		serio_drv_write_wakeup(&serport->serio);
	spin_unlock_irqrestore(&serport->lock, flags);
}

static const struct tty_port_client_operations client_ops = {
	.receive_buf = serport_receive_buf,
	.write_wakeup = serport_write_wakeup,
};

static int serio_tty_port_register(struct tty_port *port, struct tty_driver *drv, int idx)
{
	struct device_node *node, *found = NULL;
	struct device *parent = port->dev;
	struct serio *serio;
	struct serport *serport;

	if (!parent || !parent->of_node)
		return -ENODEV;

	for_each_available_child_of_node(parent->of_node, node) {
		if (!of_get_property(node, "compatible", NULL))
			continue;
		if (found) {
			dev_err(parent, "Multiple connected children found - none registered");
			return -ENODEV;
		}
		found = node;
	}
	if (!found)
		return -ENODEV;

	serport = kzalloc(sizeof(struct serport), GFP_KERNEL);
	if (!serport)
		return -ENOMEM;

	serio = &serport->serio;

	spin_lock_init(&serport->lock);

	serport->port = port;
	serport->tty_idx = idx;
	serport->tty_drv = drv;

	port->client_ops = &client_ops;
	port->client_data = serport;

	strlcpy(serio->name, "Serial port", sizeof(serio->name));
	snprintf(serio->phys, sizeof(serio->phys), "%s/serio0", drv->name);
	serio->id.type = SERIO_RS232;
	serio->write_buf = serport_serio_write_buf;
	serio->write_flush = serport_serio_write_flush;
	serio->open = serport_serio_open;
	serio->close = serport_serio_close;
	serio->set_baudrate = serport_serio_set_baudrate;
	serio->set_flow_control = serport_serio_set_flow_control;
	serio->port_data = serport;
	serio->dev.parent = parent;

	list_add(&serport->serport_list, &serio_tty_ports);

	serio_register_port(serio);
	printk(KERN_INFO "serio: Serial port %s\n", drv->name);

	return 0;
}

static void serio_tty_port_unregister(struct tty_port *port)
{
	struct serport *serport = port->client_data;

	if (!serport)
		return;

	list_del(&serport->serport_list);

	serio_unregister_port(&serport->serio);
	port->client_ops = NULL;
	port->client_data = NULL;
	kfree(serport);
}

int serio_tty_port_init(void)
{
	tty_port_get_registered_ports(serio_tty_port_register);

	return 0;
}
module_init(serio_tty_port_init);

void serio_tty_port_exit(void)
{
	struct serport *serport, *tmp;

	list_for_each_entry_safe(serport, tmp, &serio_tty_ports, serport_list)
		serio_tty_port_unregister(serport->port);
}
module_exit(serio_tty_port_exit);
