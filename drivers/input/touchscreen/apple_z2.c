// SPDX-License-Identifier: GPL-2.0
/*
 * Apple Z2 touchscreen driver
 *
 * Copyright (C) The Asahi Linux Contributors
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/gpio/consumer.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/input/touchscreen.h>
#include <linux/interrupt.h>
#include <linux/ktime.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>
#include <linux/unaligned.h>

#define APPLE_Z2_NUM_FINGERS_OFFSET      16
#define APPLE_Z2_FINGERS_OFFSET          24
#define APPLE_Z2_TOUCH_STARTED           3
#define APPLE_Z2_TOUCH_MOVED             4
#define APPLE_Z2_CMD_READ_INTERRUPT_DATA 0xEB
#define APPLE_Z2_REPLY_INTERRUPT_DATA    0xE1
#define APPLE_Z2_HBPP_CMD_BLOB           0x3001
#define APPLE_Z2_FW_MAGIC                0x5746325A
#define LOAD_COMMAND_INIT_PAYLOAD        0
#define LOAD_COMMAND_SEND_BLOB           1
#define LOAD_COMMAND_SEND_CALIBRATION    2
#define LOAD_COMMAND_SEND_PROX_CAL       3
#define LOAD_COMMAND_DELAY               4
#define CAL_PROP_NAME                    "apple,z2-cal-blob"
#define PROX_CAL_PROP_NAME               "apple,z2-prox-cal-blob"

/*
 * N1 (iPhone 6): same Z2 protocol, but the HBPP bootloader talks big-endian
 * 16-bit words over an 8-bit-only SPI link, DATA packets carry an extra sync
 * prefix and every packet is acknowledged with a status word.
 */
#define APPLE_N1_HBPP_SYNC               0x18e1
#define APPLE_N1_HBPP_ATN_ACK            0x1aa1
#define APPLE_N1_HBPP_WAKEUP             0x19c1
#define APPLE_N1_HBPP_MEM_WRITE          0x1e33
#define APPLE_N1_HBPP_CALIBRATE          0x1f01
#define APPLE_N1_ACK_DATA_OK             0x4bc1
#define APPLE_N1_ACK_WRITE_OK            0x4ad1
#define APPLE_N1_CAL_CHUNK               0x3f0
#define APPLE_N1_REPORT_CONTACTS         0x44
#define APPLE_N1_REPORT_CONTACTS_IMAGE   0x45
#define APPLE_N1_BUF_SIZE                4000
/* iOS keeps the N1 unpowered for at least this long before powering it again */
#define APPLE_N1_MIN_OFF_MS              200

static bool close_poweroff = true;
module_param(close_poweroff, bool, 0644);
MODULE_PARM_DESC(close_poweroff,
		 "Power the controller down while the input device is closed or inhibited (default: true)");

static bool off_cs_low = true;
module_param(off_cs_low, bool, 0644);
MODULE_PARM_DESC(off_cs_low,
		 "N1: hold the GPIO chip select asserted (low) while unpowered, as iOS does (default: true)");

struct apple_z2 {
	struct spi_device *spidev;
	struct gpio_desc *reset_gpio;
	struct input_dev *input_dev;
	struct completion boot_irq;
	bool booted;
	int index_parity;
	struct touchscreen_properties props;
	const char *fw_name;
	u8 *tx_buf;
	u8 *rx_buf;
	bool n1;
	bool powered;
	bool ack_after_delay;
	struct regulator *vdd;
	struct regulator *vdd_ana;
	struct clk *clk;
	/* Serialises probe, open/close, suspend/resume and remove */
	struct mutex lock;
	bool running;
	bool resume_running;
	ktime_t off_time;
};

struct apple_z2_finger {
	u8 finger;
	u8 state;
	__le16 unknown2;
	__le16 abs_x;
	__le16 abs_y;
	__le16 rel_x;
	__le16 rel_y;
	__le16 tool_major;
	__le16 tool_minor;
	__le16 orientation;
	__le16 touch_major;
	__le16 touch_minor;
	__le16 unused[2];
	__le16 pressure;
	__le16 multi;
} __packed;

struct apple_z2_hbpp_blob_hdr {
	__le16 cmd;
	__le16 len;
	__le32 addr;
	__le16 checksum;
};

struct apple_z2_fw_hdr {
	__le32 magic;
	__le32 version;
};

struct apple_z2_read_interrupt_cmd {
	u8 cmd;
	u8 counter;
	u8 unused[12];
	__le16 checksum;
};

static void apple_z2_parse_touches(struct apple_z2 *z2,
				   const u8 *msg, size_t msg_len)
{
	int i;
	int nfingers;
	int slot;
	int slot_valid;
	struct apple_z2_finger *fingers;

	if (msg_len <= APPLE_Z2_NUM_FINGERS_OFFSET)
		return;
	nfingers = msg[APPLE_Z2_NUM_FINGERS_OFFSET];
	fingers = (struct apple_z2_finger *)(msg + APPLE_Z2_FINGERS_OFFSET);
	for (i = 0; i < nfingers; i++) {
		slot = input_mt_get_slot_by_key(z2->input_dev, fingers[i].finger);
		if (slot < 0) {
			dev_warn(&z2->spidev->dev, "unable to get slot for finger\n");
			continue;
		}
		slot_valid = fingers[i].state == APPLE_Z2_TOUCH_STARTED ||
			     fingers[i].state == APPLE_Z2_TOUCH_MOVED;
		input_mt_slot(z2->input_dev, slot);
		if (!input_mt_report_slot_state(z2->input_dev, MT_TOOL_FINGER, slot_valid))
			continue;
		touchscreen_report_pos(z2->input_dev, &z2->props,
				       le16_to_cpu(fingers[i].abs_x),
				       le16_to_cpu(fingers[i].abs_y),
				       true);
		input_report_abs(z2->input_dev, ABS_MT_WIDTH_MAJOR,
				 le16_to_cpu(fingers[i].tool_major));
		input_report_abs(z2->input_dev, ABS_MT_WIDTH_MINOR,
				 le16_to_cpu(fingers[i].tool_minor));
		input_report_abs(z2->input_dev, ABS_MT_ORIENTATION,
				 le16_to_cpu(fingers[i].orientation));
		input_report_abs(z2->input_dev, ABS_MT_TOUCH_MAJOR,
				 le16_to_cpu(fingers[i].touch_major));
		input_report_abs(z2->input_dev, ABS_MT_TOUCH_MINOR,
				 le16_to_cpu(fingers[i].touch_minor));
	}
	input_mt_sync_frame(z2->input_dev);
	input_sync(z2->input_dev);
}

/*
 * N1 contact reports: report ID, header length at +2, image length at +14,
 * contact count at +16 and record size at +17. Coordinates are signed.
 */
static void apple_n1_parse_touches(struct apple_z2 *z2, const u8 *msg, size_t msg_len)
{
	const struct apple_z2_finger *finger;
	size_t off, rec_size;
	int i, nfingers, slot;
	bool valid;

	if (msg_len < APPLE_Z2_FINGERS_OFFSET ||
	    (msg[0] != APPLE_N1_REPORT_CONTACTS && msg[0] != APPLE_N1_REPORT_CONTACTS_IMAGE))
		return;

	nfingers = msg[APPLE_Z2_NUM_FINGERS_OFFSET];
	rec_size = msg[APPLE_Z2_NUM_FINGERS_OFFSET + 1];
	off = msg[2] + get_unaligned_le16(msg + 14);
	if (rec_size < sizeof(*finger))
		return;

	for (i = 0; i < nfingers && off + rec_size <= msg_len; i++, off += rec_size) {
		finger = (const struct apple_z2_finger *)(msg + off);
		slot = input_mt_get_slot_by_key(z2->input_dev, finger->finger);
		if (slot < 0) {
			dev_warn(&z2->spidev->dev, "unable to get slot for finger\n");
			continue;
		}
		valid = finger->state == APPLE_Z2_TOUCH_STARTED ||
			finger->state == APPLE_Z2_TOUCH_MOVED;
		input_mt_slot(z2->input_dev, slot);
		if (!input_mt_report_slot_state(z2->input_dev, MT_TOOL_FINGER, valid))
			continue;
		touchscreen_report_pos(z2->input_dev, &z2->props,
				       max_t(s16, le16_to_cpu(finger->abs_x), 0),
				       max_t(s16, le16_to_cpu(finger->abs_y), 0),
				       true);
		input_report_abs(z2->input_dev, ABS_MT_TOUCH_MAJOR,
				 le16_to_cpu(finger->tool_major));
		input_report_abs(z2->input_dev, ABS_MT_TOUCH_MINOR,
				 le16_to_cpu(finger->tool_minor));
		input_report_abs(z2->input_dev, ABS_MT_ORIENTATION,
				 (s16)le16_to_cpu(finger->orientation));
		input_report_abs(z2->input_dev, ABS_MT_PRESSURE,
				 le16_to_cpu(finger->pressure));
	}
	input_mt_sync_frame(z2->input_dev);
	input_sync(z2->input_dev);
}

static u16 apple_n1_sum(const u8 *buf, size_t len)
{
	u16 sum = 0;

	while (len--)
		sum += *buf++;
	return sum;
}

/*
 * The length reply and the data phase are both checksummed, and the counter
 * only advances after a good frame.
 */
static int apple_n1_read_packet(struct apple_z2 *z2)
{
	u8 *tx = z2->tx_buf, *rx = z2->rx_buf;
	u8 counter = z2->index_parity + 1;
	struct spi_transfer xfer = { .tx_buf = tx, .rx_buf = rx, .len = 16 };
	size_t len, payload_len;
	int error;

	memset(tx, 0, 16);
	tx[0] = APPLE_Z2_CMD_READ_INTERRUPT_DATA;
	tx[1] = counter;
	put_unaligned_le16(apple_n1_sum(tx, 14), tx + 14);
	error = spi_sync_transfer(z2->spidev, &xfer, 1);
	if (error)
		return error;

	if ((rx[0] & 0xf0) != 0xe0 || get_unaligned_le16(rx + 14) != apple_n1_sum(rx, 14))
		return 0;
	len = get_unaligned_le16(rx + 1);
	if (!len)
		return 0;
	if (len + 5 > APPLE_N1_BUF_SIZE)
		return -EMSGSIZE;

	memset(tx, 0, len + 5);
	tx[0] = APPLE_Z2_CMD_READ_INTERRUPT_DATA;
	tx[1] = counter;
	tx[2] = 1;
	put_unaligned_le16(apple_n1_sum(tx, 3), tx + len + 3);
	xfer.len = len + 5;
	error = spi_sync_transfer(z2->spidev, &xfer, 1);
	if (error)
		return error;

	payload_len = get_unaligned_le16(rx + 2);
	if (apple_n1_sum(rx, 5) & 0xff || payload_len < 2 || payload_len > len ||
	    get_unaligned_le16(rx + 5 + payload_len - 2) !=
	    apple_n1_sum(rx + 5, payload_len - 2)) {
		dev_dbg(&z2->spidev->dev, "bad frame\n");
		return 0;
	}

	z2->index_parity = !z2->index_parity;
	apple_n1_parse_touches(z2, rx + 5, payload_len - 2);
	return 0;
}

static int apple_z2_read_packet(struct apple_z2 *z2)
{
	struct apple_z2_read_interrupt_cmd *len_cmd = (void *)z2->tx_buf;
	struct spi_transfer xfer;
	int error;
	size_t pkt_len;

	if (z2->n1)
		return apple_n1_read_packet(z2);

	memset(&xfer, 0, sizeof(xfer));
	len_cmd->cmd = APPLE_Z2_CMD_READ_INTERRUPT_DATA;
	len_cmd->counter = z2->index_parity + 1;
	len_cmd->checksum =
		cpu_to_le16(APPLE_Z2_CMD_READ_INTERRUPT_DATA + len_cmd->counter);
	z2->index_parity = !z2->index_parity;
	xfer.tx_buf = z2->tx_buf;
	xfer.rx_buf = z2->rx_buf;
	xfer.len = sizeof(*len_cmd);

	error = spi_sync_transfer(z2->spidev, &xfer, 1);
	if (error)
		return error;

	if (z2->rx_buf[0] != APPLE_Z2_REPLY_INTERRUPT_DATA)
		return 0;

	pkt_len = (get_unaligned_le16(z2->rx_buf + 1) + 8) & 0xfffffffc;

	error = spi_read(z2->spidev, z2->rx_buf, pkt_len);
	if (error)
		return error;

	apple_z2_parse_touches(z2, z2->rx_buf + 5, pkt_len - 5);

	return 0;
}

static irqreturn_t apple_z2_irq(int irq, void *data)
{
	struct apple_z2 *z2 = data;

	if (unlikely(!z2->booted))
		complete(&z2->boot_irq);
	else
		apple_z2_read_packet(z2);

	return IRQ_HANDLED;
}

/* Build an HBPP data blob, caller is responsible for freeing the blob data. */
/*
 * The header struct carries two bytes of tail padding, which the Z2 tolerates.
 * The N1 does not, so it passes the exact header length instead.
 */
static u8 *apple_z2_build_blob(u32 address, const u8 *data, size_t len, size_t hdr_len,
			       size_t *size)
{
	struct apple_z2_hbpp_blob_hdr *hdr;
	size_t padded = round_up(len, 4);
	size_t blob_size;
	u16 checksum_hdr;
	u32 checksum;
	u8 *blob_data;
	size_t i;

	blob_size = hdr_len + padded + sizeof(__le32);
	blob_data = kzalloc(blob_size, GFP_KERNEL);
	if (!blob_data)
		return ERR_PTR(-ENOMEM);

	hdr = (struct apple_z2_hbpp_blob_hdr *)blob_data;
	hdr->cmd = cpu_to_le16(APPLE_Z2_HBPP_CMD_BLOB);
	hdr->len = cpu_to_le16(padded / 4);
	hdr->addr = cpu_to_le32(address);

	checksum_hdr = 0;
	for (i = 2; i < 8; i++)
		checksum_hdr += blob_data[i];
	hdr->checksum = cpu_to_le16(checksum_hdr);

	memcpy(blob_data + hdr_len, data, len);
	checksum = 0;
	for (i = 0; i < len; i++)
		checksum += data[i];
	put_unaligned_le32(checksum, blob_data + hdr_len + padded);

	*size = blob_size;
	return blob_data;
}

/* Read a calibration property, caller is responsible for freeing the data. */
static u8 *apple_z2_read_cal(struct apple_z2 *z2, const char *prop, size_t *len)
{
	int cal_size, error;

	if (!device_property_present(&z2->spidev->dev, prop))
		return NULL;

	cal_size = device_property_count_u8(&z2->spidev->dev, prop);
	if (cal_size < 0)
		return ERR_PTR(cal_size);

	u8 *cal_data __free(kfree) = kzalloc(cal_size, GFP_KERNEL);
	if (!cal_data)
		return ERR_PTR(-ENOMEM);

	error = device_property_read_u8_array(&z2->spidev->dev, prop, cal_data, cal_size);
	if (error)
		return ERR_PTR(error);

	*len = cal_size;
	return no_free_ptr(cal_data);
}

/* Build calibration blob, caller is responsible for freeing the blob data. */
static const u8 *apple_z2_build_cal_blob(struct apple_z2 *z2,
					 u32 address, size_t *size)
{
	size_t cal_size;

	const u8 *cal_data __free(kfree) = apple_z2_read_cal(z2, CAL_PROP_NAME, &cal_size);
	if (IS_ERR_OR_NULL(cal_data))
		return cal_data;

	return apple_z2_build_blob(address, cal_data, cal_size,
				   sizeof(struct apple_z2_hbpp_blob_hdr), size);
}

static int apple_n1_ack(struct apple_z2 *z2, u16 *status)
{
	struct spi_transfer xfer = { .tx_buf = z2->tx_buf, .rx_buf = z2->rx_buf, .len = 2 };
	int error;

	put_unaligned_be16(APPLE_N1_HBPP_ATN_ACK, z2->tx_buf);
	error = spi_sync_transfer(z2->spidev, &xfer, 1);
	if (!error)
		*status = get_unaligned_be16(z2->rx_buf);
	return error;
}

/*
 * Blobs are stored as little-endian 16-bit words like for the Z2, but the N1
 * link is 8-bit, so swap each word. DATA packets get the sync prefix and are
 * acknowledged, as are memory writes; the calibration request is acknowledged
 * after the delay that follows it.
 */
static int apple_n1_send_blob(struct apple_z2 *z2, const u8 *data, u32 size, bool init)
{
	u16 cmd = size >= 2 ? get_unaligned_le16(data) : 0;
	u16 expected = 0, status;
	size_t off = 0, i;
	int error;

	u8 *buf __free(kfree) = kmalloc(size + 2, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	if (init) {
		memcpy(buf, data, size);
	} else {
		if (size % 2)
			return -EINVAL;
		if (cmd == APPLE_Z2_HBPP_CMD_BLOB) {
			put_unaligned_be16(APPLE_N1_HBPP_SYNC, buf);
			off = 2;
			expected = APPLE_N1_ACK_DATA_OK;
		} else if (cmd == APPLE_N1_HBPP_MEM_WRITE) {
			expected = APPLE_N1_ACK_WRITE_OK;
		} else if (cmd == APPLE_N1_HBPP_CALIBRATE) {
			z2->ack_after_delay = true;
		}
		for (i = 0; i < size; i += 2) {
			buf[off + i] = data[i + 1];
			buf[off + i + 1] = data[i];
		}
	}

	struct spi_transfer xfer = { .tx_buf = buf, .rx_buf = NULL, .len = size + off };

	error = spi_sync_transfer(z2->spidev, &xfer, 1);
	if (error || !expected)
		return error;

	error = apple_n1_ack(z2, &status);
	if (error)
		return error;
	if (status != expected) {
		dev_err(&z2->spidev->dev, "command %04x: status %04x, expected %04x\n",
			cmd, status, expected);
		return -EIO;
	}
	return 0;
}

/* The N1 takes calibration data in chunks of at most APPLE_N1_CAL_CHUNK bytes. */
static int apple_n1_send_cal(struct apple_z2 *z2, const char *prop, u32 address)
{
	size_t cal_size, off, chunk, size;
	int error;

	const u8 *cal_data __free(kfree) = apple_z2_read_cal(z2, prop, &cal_size);
	if (IS_ERR(cal_data))
		return PTR_ERR(cal_data);
	if (!cal_data) {
		dev_warn(&z2->spidev->dev, "no %s, touch may be inaccurate\n", prop);
		return 0;
	}

	for (off = 0; off < cal_size; off += chunk) {
		chunk = min_t(size_t, cal_size - off, APPLE_N1_CAL_CHUNK);

		const u8 *blob __free(kfree) =
			apple_z2_build_blob(address + off, cal_data + off, chunk,
					    offsetofend(struct apple_z2_hbpp_blob_hdr, checksum),
					    &size);
		if (IS_ERR(blob))
			return PTR_ERR(blob);

		error = apple_n1_send_blob(z2, blob, size, false);
		if (error)
			return error;
	}
	return 0;
}

static int apple_z2_send_firmware_blob(struct apple_z2 *z2, const u8 *data,
				       u32 size, bool init)
{
	struct spi_message msg;
	struct spi_transfer blob_xfer, ack_xfer;
	int error;

	if (z2->n1)
		return apple_n1_send_blob(z2, data, size, init);

	z2->tx_buf[0] = 0x1a;
	z2->tx_buf[1] = 0xa1;

	spi_message_init(&msg);
	memset(&blob_xfer, 0, sizeof(blob_xfer));
	memset(&ack_xfer, 0, sizeof(ack_xfer));

	blob_xfer.tx_buf = data;
	blob_xfer.len = size;
	blob_xfer.bits_per_word = init ? 8 : 16;
	spi_message_add_tail(&blob_xfer, &msg);

	ack_xfer.tx_buf = z2->tx_buf;
	ack_xfer.len = 2;
	spi_message_add_tail(&ack_xfer, &msg);

	reinit_completion(&z2->boot_irq);
	error = spi_sync(z2->spidev, &msg);
	if (error)
		return error;

	/* Irq only happens sometimes, but the thing boots reliably nonetheless */
	wait_for_completion_timeout(&z2->boot_irq, msecs_to_jiffies(20));

	return 0;
}

static int apple_z2_upload_firmware(struct apple_z2 *z2)
{
	const struct apple_z2_fw_hdr *fw_hdr;
	size_t fw_idx = sizeof(struct apple_z2_fw_hdr);
	int error;
	u32 load_cmd;
	u32 address;
	bool init;
	size_t size;

	const struct firmware *fw __free(firmware) = NULL;
	error = request_firmware(&fw, z2->fw_name, &z2->spidev->dev);
	if (error) {
		dev_err(&z2->spidev->dev, "unable to load firmware\n");
		return error;
	}

	fw_hdr = (const struct apple_z2_fw_hdr *)fw->data;
	if (le32_to_cpu(fw_hdr->magic) != APPLE_Z2_FW_MAGIC || le32_to_cpu(fw_hdr->version) != 1) {
		dev_err(&z2->spidev->dev, "invalid firmware header\n");
		return -EINVAL;
	}

	/*
	 * This will interrupt the upload half-way if the file is malformed
	 * As the device has no non-volatile storage to corrupt, and gets reset
	 * on boot anyway, this is fine.
	 */
	while (fw_idx < fw->size) {
		if (fw->size - fw_idx < 8) {
			dev_err(&z2->spidev->dev, "firmware malformed\n");
			return -EINVAL;
		}

		load_cmd = le32_to_cpup((__force __le32 *)(fw->data + fw_idx));
		fw_idx += sizeof(u32);
		if (load_cmd == LOAD_COMMAND_INIT_PAYLOAD || load_cmd == LOAD_COMMAND_SEND_BLOB) {
			size = le32_to_cpup((__force __le32 *)(fw->data + fw_idx));
			fw_idx += sizeof(u32);
			if (fw->size - fw_idx < size) {
				dev_err(&z2->spidev->dev, "firmware malformed\n");
				return -EINVAL;
			}
			init = load_cmd == LOAD_COMMAND_INIT_PAYLOAD;
			error = apple_z2_send_firmware_blob(z2, fw->data + fw_idx,
							    size, init);
			if (error)
				return error;
			fw_idx += size;
		} else if (load_cmd == LOAD_COMMAND_SEND_CALIBRATION && z2->n1) {
			address = le32_to_cpup((__force __le32 *)(fw->data + fw_idx));
			fw_idx += sizeof(u32);
			error = apple_n1_send_cal(z2, CAL_PROP_NAME, address);
			if (error)
				return error;
		} else if (load_cmd == LOAD_COMMAND_SEND_CALIBRATION) {
			address = le32_to_cpup((__force __le32 *)(fw->data + fw_idx));
			fw_idx += sizeof(u32);

			const u8 *data __free(kfree) =
				apple_z2_build_cal_blob(z2, address, &size);
			if (IS_ERR(data))
				return PTR_ERR(data);

			if (data) {
				error = apple_z2_send_firmware_blob(z2, data, size, false);
				if (error)
					return error;
			}
		} else if (load_cmd == LOAD_COMMAND_SEND_PROX_CAL && z2->n1) {
			address = le32_to_cpup((__force __le32 *)(fw->data + fw_idx));
			fw_idx += sizeof(u32);
			error = apple_n1_send_cal(z2, PROX_CAL_PROP_NAME, address);
			if (error)
				return error;
		} else if (load_cmd == LOAD_COMMAND_DELAY) {
			msleep(le32_to_cpup((__force __le32 *)(fw->data + fw_idx)));
			fw_idx += sizeof(u32);
			if (z2->ack_after_delay) {
				u16 status;

				z2->ack_after_delay = false;
				error = apple_n1_ack(z2, &status);
				if (error)
					return error;
			}
		} else {
			dev_err(&z2->spidev->dev, "firmware malformed\n");
			return -EINVAL;
		}
		fw_idx = round_up(fw_idx, 4);
	}

	z2->booted = true;
	apple_z2_read_packet(z2);
	return 0;
}

static const u16 apple_n1_probe_replies[] = {
	0x18e1, 0x1aa1, 0x1f01, 0x4879, 0x4969, 0x4ad1, 0x4bc1,
};

/* ATN_ACK followed by sync words: keeps SCLK busy, the reply tells if the bootloader runs */
static int apple_n1_probe_bootloader(struct apple_z2 *z2)
{
	struct spi_transfer xfer = { .tx_buf = z2->tx_buf, .rx_buf = z2->rx_buf, .len = 16 };
	int error, i;
	u16 reply;

	put_unaligned_be16(APPLE_N1_HBPP_ATN_ACK, z2->tx_buf);
	for (i = 2; i < 16; i += 2)
		put_unaligned_be16(APPLE_N1_HBPP_SYNC, z2->tx_buf + i);
	error = spi_sync_transfer(z2->spidev, &xfer, 1);
	if (error)
		return error;

	reply = get_unaligned_be16(z2->rx_buf);
	for (i = 0; i < ARRAY_SIZE(apple_n1_probe_replies); i++)
		if (reply == apple_n1_probe_replies[i])
			return 0;
	return -ENODEV;
}

/*
 * iOS drives the chip select low before it cuts the supplies and only raises
 * it again once they are back, so the unpowered controller is not fed through
 * its CS pin. The line is an active-low GPIO owned by the SPI core, which only
 * touches it during transfers, and there are none while the N1 is off.
 */
static void apple_n1_set_cs_asserted(struct apple_z2 *z2, bool asserted)
{
	struct gpio_desc *cs = spi_get_csgpiod(z2->spidev, 0);

	if (cs && (off_cs_low || !asserted))
		gpiod_set_value_cansleep(cs, asserted);
}

static void apple_n1_power_off(struct apple_z2 *z2)
{
	if (!z2->powered)
		return;

	gpiod_set_value_cansleep(z2->reset_gpio, 1);
	usleep_range(1000, 2000);
	apple_n1_set_cs_asserted(z2, true);
	clk_disable_unprepare(z2->clk);
	if (z2->vdd)
		regulator_disable(z2->vdd);
	if (z2->vdd_ana)
		regulator_disable(z2->vdd_ana);
	z2->powered = false;
	z2->off_time = ktime_get();
}

static int apple_n1_power_on(struct apple_z2 *z2)
{
	struct spi_transfer xfer = { .tx_buf = z2->tx_buf, .rx_buf = z2->rx_buf, .len = 2 };
	s64 off_ms;
	int error;

	if (z2->off_time) {
		off_ms = ktime_ms_delta(ktime_get(), z2->off_time);
		if (off_ms >= 0 && off_ms < APPLE_N1_MIN_OFF_MS)
			msleep(APPLE_N1_MIN_OFF_MS - off_ms);
	}

	if (z2->vdd_ana) {
		error = regulator_enable(z2->vdd_ana);
		if (error)
			return error;
	}
	if (z2->vdd) {
		error = regulator_enable(z2->vdd);
		if (error)
			goto err_ana;
	}
	usleep_range(5000, 6000);
	apple_n1_set_cs_asserted(z2, false);

	error = clk_prepare_enable(z2->clk);
	if (error)
		goto err_vdd;
	z2->powered = true;
	usleep_range(15000, 16000);

	/* Wake the controller while it is still held in reset */
	put_unaligned_be16(APPLE_N1_HBPP_WAKEUP, z2->tx_buf);
	error = spi_sync_transfer(z2->spidev, &xfer, 1);
	if (error)
		goto err_off;
	usleep_range(10000, 11000);

	error = apple_n1_probe_bootloader(z2);
	if (error && error != -ENODEV)
		goto err_off;
	gpiod_set_value_cansleep(z2->reset_gpio, 0);
	msleep(30);

	error = apple_n1_probe_bootloader(z2);
	if (error) {
		dev_err(&z2->spidev->dev, "bootloader not responding\n");
		goto err_off;
	}
	return 0;

err_off:
	apple_n1_power_off(z2);
	return error;
err_vdd:
	apple_n1_set_cs_asserted(z2, true);
	if (z2->vdd)
		regulator_disable(z2->vdd);
err_ana:
	if (z2->vdd_ana)
		regulator_disable(z2->vdd_ana);
	z2->off_time = ktime_get();
	return error;
}

static int apple_n1_boot(struct apple_z2 *z2)
{
	int error;

	error = apple_n1_power_on(z2);
	if (error)
		return error;

	error = apple_z2_upload_firmware(z2);
	if (error) {
		apple_n1_power_off(z2);
		return error;
	}

	enable_irq(z2->spidev->irq);
	return 0;
}

/* Bring the controller up; the IRQ is enabled only if this succeeds */
static int apple_z2_boot(struct apple_z2 *z2)
{
	int error;

	/* A freshly booted controller expects the same counter as at probe */
	z2->index_parity = 0;

	if (z2->n1)
		return apple_n1_boot(z2);

	reinit_completion(&z2->boot_irq);
	enable_irq(z2->spidev->irq);
	gpiod_set_value(z2->reset_gpio, 0);
	if (!wait_for_completion_timeout(&z2->boot_irq, msecs_to_jiffies(20)))
		error = -ETIMEDOUT;
	else
		error = apple_z2_upload_firmware(z2);
	if (error) {
		gpiod_set_value(z2->reset_gpio, 1);
		disable_irq(z2->spidev->irq);
		return error;
	}

	return 0;
}

/* Boot with one retry. Must be called with z2->lock held. */
static int apple_z2_start(struct apple_z2 *z2)
{
	struct device *dev = &z2->spidev->dev;
	int error;

	lockdep_assert_held(&z2->lock);

	if (z2->running)
		return 0;

	error = apple_z2_boot(z2);
	if (error) {
		dev_warn(dev, "bring-up failed (%d), retrying\n", error);
		/* The N1 path waits out its minimum off time by itself */
		usleep_range(5000, 10000);
		error = apple_z2_boot(z2);
		if (error) {
			dev_err(dev, "bring-up failed again (%d), controller left off\n", error);
			return error;
		}
		dev_info(dev, "bring-up succeeded on retry\n");
	}

	z2->running = true;
	return 0;
}

/* IRQ off first, then the controller. Must be called with z2->lock held. */
static void apple_z2_stop(struct apple_z2 *z2)
{
	lockdep_assert_held(&z2->lock);

	if (!z2->running)
		return;

	disable_irq(z2->spidev->irq);
	if (z2->n1)
		apple_n1_power_off(z2);
	else
		gpiod_direction_output(z2->reset_gpio, 1);
	z2->booted = false;
	z2->running = false;
}

static int apple_z2_open(struct input_dev *input_dev)
{
	struct apple_z2 *z2 = input_get_drvdata(input_dev);

	guard(mutex)(&z2->lock);
	return apple_z2_start(z2);
}

static void apple_z2_close(struct input_dev *input_dev)
{
	struct apple_z2 *z2 = input_get_drvdata(input_dev);

	if (!close_poweroff)
		return;

	guard(mutex)(&z2->lock);
	apple_z2_stop(z2);
}

static int apple_z2_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct apple_z2 *z2;
	int error;

	z2 = devm_kzalloc(dev, sizeof(*z2), GFP_KERNEL);
	if (!z2)
		return -ENOMEM;

	z2->n1 = (uintptr_t)device_get_match_data(dev);
	z2->tx_buf = devm_kzalloc(dev, z2->n1 ? APPLE_N1_BUF_SIZE :
				  sizeof(struct apple_z2_read_interrupt_cmd), GFP_KERNEL);
	if (!z2->tx_buf)
		return -ENOMEM;
	/* 4096 will end up being rounded up to 8192 due to devres header */
	z2->rx_buf = devm_kzalloc(dev, 4000, GFP_KERNEL);
	if (!z2->rx_buf)
		return -ENOMEM;

	z2->spidev = spi;
	init_completion(&z2->boot_irq);
	spi_set_drvdata(spi, z2);
	error = devm_mutex_init(dev, &z2->lock);
	if (error)
		return error;

	/* Reset the device on boot */
	z2->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(z2->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(z2->reset_gpio), "unable to get reset\n");

	if (z2->n1) {
		z2->vdd = devm_regulator_get_optional(dev, "vdd");
		if (IS_ERR(z2->vdd)) {
			if (PTR_ERR(z2->vdd) != -ENODEV)
				return dev_err_probe(dev, PTR_ERR(z2->vdd), "unable to get vdd\n");
			z2->vdd = NULL;
		}
		z2->vdd_ana = devm_regulator_get_optional(dev, "vdd-ana");
		if (IS_ERR(z2->vdd_ana)) {
			if (PTR_ERR(z2->vdd_ana) != -ENODEV)
				return dev_err_probe(dev, PTR_ERR(z2->vdd_ana),
						     "unable to get vdd-ana\n");
			z2->vdd_ana = NULL;
		}
		z2->clk = devm_clk_get_optional(dev, NULL);
		if (IS_ERR(z2->clk))
			return dev_err_probe(dev, PTR_ERR(z2->clk), "unable to get clock\n");
	}

	error = devm_request_threaded_irq(dev, z2->spidev->irq, NULL, apple_z2_irq,
					  IRQF_ONESHOT | IRQF_NO_AUTOEN,
					  "apple-z2-irq", z2);
	if (error)
		return dev_err_probe(dev, error, "unable to request irq\n");

	error = device_property_read_string(dev, "firmware-name", &z2->fw_name);
	if (error)
		return dev_err_probe(dev, error, "unable to get firmware name\n");

	z2->input_dev = devm_input_allocate_device(dev);
	if (!z2->input_dev)
		return -ENOMEM;

	z2->input_dev->name = (char *)spi_get_device_id(spi)->driver_data;
	z2->input_dev->phys = "apple_z2";
	z2->input_dev->id.bustype = BUS_SPI;
	z2->input_dev->open = apple_z2_open;
	z2->input_dev->close = apple_z2_close;
	input_set_drvdata(z2->input_dev, z2);

	/* Allocate the axes before setting from DT */
	input_set_abs_params(z2->input_dev, ABS_MT_POSITION_X, 0, 0, 0, 0);
	input_set_abs_params(z2->input_dev, ABS_MT_POSITION_Y, 0, 0, 0, 0);
	touchscreen_parse_properties(z2->input_dev, true, &z2->props);
	input_abs_set_res(z2->input_dev, ABS_MT_POSITION_X, 100);
	input_abs_set_res(z2->input_dev, ABS_MT_POSITION_Y, 100);
	input_set_abs_params(z2->input_dev, ABS_MT_WIDTH_MAJOR, 0, 65535, 0, 0);
	input_set_abs_params(z2->input_dev, ABS_MT_WIDTH_MINOR, 0, 65535, 0, 0);
	input_set_abs_params(z2->input_dev, ABS_MT_TOUCH_MAJOR, 0, 65535, 0, 0);
	input_set_abs_params(z2->input_dev, ABS_MT_TOUCH_MINOR, 0, 65535, 0, 0);
	input_set_abs_params(z2->input_dev, ABS_MT_ORIENTATION, -32768, 32767, 0, 0);
	if (z2->n1)
		input_set_abs_params(z2->input_dev, ABS_MT_PRESSURE, 0, 65535, 0, 0);

	error = input_mt_init_slots(z2->input_dev, 256, INPUT_MT_DIRECT);
	if (error)
		return dev_err_probe(dev, error, "unable to initialize multitouch slots\n");

	error = input_register_device(z2->input_dev);
	if (error)
		return dev_err_probe(dev, error, "unable to register input device\n");

	/*
	 * Boot now, so a broken controller fails the probe. If nobody opens the
	 * device, the first close() or inhibit powers it down again.
	 */
	usleep_range(5000, 10000);
	guard(mutex)(&z2->lock);
	return apple_z2_start(z2);
}

static void apple_z2_shutdown(struct spi_device *spi)
{
	struct apple_z2 *z2 = spi_get_drvdata(spi);

	guard(mutex)(&z2->lock);
	apple_z2_stop(z2);
}

static int apple_z2_suspend(struct device *dev)
{
	struct apple_z2 *z2 = spi_get_drvdata(to_spi_device(dev));

	guard(mutex)(&z2->lock);
	z2->resume_running = z2->running;
	apple_z2_stop(z2);

	return 0;
}

/* A closed or inhibited device stays off across suspend */
static int apple_z2_resume(struct device *dev)
{
	struct apple_z2 *z2 = spi_get_drvdata(to_spi_device(dev));

	guard(mutex)(&z2->lock);
	if (!z2->resume_running)
		return 0;

	return apple_z2_start(z2);
}

static DEFINE_SIMPLE_DEV_PM_OPS(apple_z2_pm, apple_z2_suspend, apple_z2_resume);

static const struct of_device_id apple_z2_of_match[] = {
	{ .compatible = "apple,j293-touchbar" },
	{ .compatible = "apple,j493-touchbar" },
	{ .compatible = "apple,n61-touchscreen", .data = (void *)true },
	{}
};
MODULE_DEVICE_TABLE(of, apple_z2_of_match);

static struct spi_device_id apple_z2_of_id[] = {
	{ .name = "j293-touchbar", .driver_data = (kernel_ulong_t)"MacBookPro17,1 Touch Bar" },
	{ .name = "j493-touchbar", .driver_data = (kernel_ulong_t)"Mac14,7 Touch Bar" },
	{ .name = "n61-touchscreen", .driver_data = (kernel_ulong_t)"iPhone 6 Touchscreen" },
	{}
};
MODULE_DEVICE_TABLE(spi, apple_z2_of_id);

static struct spi_driver apple_z2_driver = {
	.driver = {
		.name	= "apple-z2",
		.pm	= pm_sleep_ptr(&apple_z2_pm),
		.of_match_table = apple_z2_of_match,
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
	},
	.id_table = apple_z2_of_id,
	.probe    = apple_z2_probe,
	.remove   = apple_z2_shutdown,
};

module_spi_driver(apple_z2_driver);

MODULE_LICENSE("GPL");
MODULE_FIRMWARE("apple/dfrmtfw-*.bin");
MODULE_DESCRIPTION("Apple Z2 touchscreens driver");
