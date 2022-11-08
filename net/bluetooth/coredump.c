// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2022 Google Corporation
 */

#include <linux/devcoredump.h>

#include <net/bluetooth/bluetooth.h>
#include <net/bluetooth/hci_core.h>

enum hci_devcoredump_pkt_type {
	HCI_DEVCOREDUMP_PKT_INIT,
	HCI_DEVCOREDUMP_PKT_SKB,
	HCI_DEVCOREDUMP_PKT_PATTERN,
	HCI_DEVCOREDUMP_PKT_COMPLETE,
	HCI_DEVCOREDUMP_PKT_ABORT,
};

struct hci_devcoredump_skb_cb {
	u16 pkt_type;
};

struct hci_devcoredump_skb_pattern {
	u8 pattern;
	u32 len;
} __packed;

#define hci_dmp_cb(skb)	((struct hci_devcoredump_skb_cb *)((skb)->cb))

#define MAX_DEVCOREDUMP_HDR_SIZE	512	/* bytes */

static int hci_devcoredump_update_hdr_state(char *buf, size_t size, int state)
{
	if (!buf)
		return 0;

	return snprintf(buf, size, "Bluetooth devcoredump\nState: %d\n", state);
}

/* Call with hci_dev_lock only. */
static int hci_devcoredump_update_state(struct hci_dev *hdev, int state)
{
	hdev->dump.state = state;

	return hci_devcoredump_update_hdr_state(hdev->dump.head,
						hdev->dump.alloc_size, state);
}

static int hci_devcoredump_mkheader(struct hci_dev *hdev, char *buf,
				    size_t buf_size)
{
	char *ptr = buf;
	size_t rem = buf_size;
	size_t read = 0;

	read = hci_devcoredump_update_hdr_state(ptr, rem, HCI_DEVCOREDUMP_IDLE);
	read += 1; /* update_hdr_state adds \0 at the end upon state rewrite */
	rem -= read;
	ptr += read;

	if (hdev->dump.dmp_hdr) {
		/* dmp_hdr() should return number of bytes written */
		read = hdev->dump.dmp_hdr(hdev, ptr, rem);
		rem -= read;
		ptr += read;
	}

	read = snprintf(ptr, rem, "--- Start dump ---\n");
	rem -= read;
	ptr += read;

	return buf_size - rem;
}

/* Do not call with hci_dev_lock since this calls driver code. */
static void hci_devcoredump_notify(struct hci_dev *hdev, int state)
{
	if (hdev->dump.notify_change)
		hdev->dump.notify_change(hdev, state);
}

/* Call with hci_dev_lock only. */
void hci_devcoredump_reset(struct hci_dev *hdev)
{
	hdev->dump.head = NULL;
	hdev->dump.tail = NULL;
	hdev->dump.alloc_size = 0;

	hci_devcoredump_update_state(hdev, HCI_DEVCOREDUMP_IDLE);

	cancel_delayed_work(&hdev->dump.dump_timeout);
	skb_queue_purge(&hdev->dump.dump_q);
}

/* Call with hci_dev_lock only. */
static void hci_devcoredump_free(struct hci_dev *hdev)
{
	if (hdev->dump.head)
		vfree(hdev->dump.head);

	hci_devcoredump_reset(hdev);
}

/* Call with hci_dev_lock only. */
static int hci_devcoredump_alloc(struct hci_dev *hdev, u32 size)
{
	hdev->dump.head = vmalloc(size);
	if (!hdev->dump.head)
		return -ENOMEM;

	hdev->dump.alloc_size = size;
	hdev->dump.tail = hdev->dump.head;
	hdev->dump.end = hdev->dump.head + size;

	hci_devcoredump_update_state(hdev, HCI_DEVCOREDUMP_IDLE);

	return 0;
}

/* Call with hci_dev_lock only. */
static bool hci_devcoredump_copy(struct hci_dev *hdev, char *buf, u32 size)
{
	if (hdev->dump.tail + size > hdev->dump.end)
		return false;

	memcpy(hdev->dump.tail, buf, size);
	hdev->dump.tail += size;

	return true;
}

/* Call with hci_dev_lock only. */
static bool hci_devcoredump_memset(struct hci_dev *hdev, u8 pattern, u32 len)
{
	if (hdev->dump.tail + len > hdev->dump.end)
		return false;

	memset(hdev->dump.tail, pattern, len);
	hdev->dump.tail += len;

	return true;
}

/* Call with hci_dev_lock only. */
static int hci_devcoredump_prepare(struct hci_dev *hdev, u32 dump_size)
{
	char *dump_hdr;
	int dump_hdr_size;
	u32 size;
	int err = 0;

	dump_hdr = vmalloc(MAX_DEVCOREDUMP_HDR_SIZE);
	if (!dump_hdr) {
		err = -ENOMEM;
		goto hdr_free;
	}

	dump_hdr_size = hci_devcoredump_mkheader(hdev, dump_hdr,
						 MAX_DEVCOREDUMP_HDR_SIZE);
	size = dump_hdr_size + dump_size;

	if (hci_devcoredump_alloc(hdev, size)) {
		err = -ENOMEM;
		goto hdr_free;
	}

	/* Insert the device header */
	if (!hci_devcoredump_copy(hdev, dump_hdr, dump_hdr_size)) {
		bt_dev_err(hdev, "Failed to insert header");
		hci_devcoredump_free(hdev);

		err = -ENOMEM;
		goto hdr_free;
	}

hdr_free:
	if (dump_hdr)
		vfree(dump_hdr);

	return err;
}

/* Bluetooth devcoredump state machine.
 *
 * Devcoredump states:
 *
 *      HCI_DEVCOREDUMP_IDLE: The default state.
 *
 *      HCI_DEVCOREDUMP_ACTIVE: A devcoredump will be in this state once it has
 *              been initialized using hci_devcoredump_init(). Once active, the
 *              driver can append data using hci_devcoredump_append() or insert
 *              a pattern using hci_devcoredump_append_pattern().
 *
 *      HCI_DEVCOREDUMP_DONE: Once the dump collection is complete, the drive
 *              can signal the completion using hci_devcoredump_complete(). A
 *              devcoredump is generated indicating the completion event and
 *              then the state machine is reset to the default state.
 *
 *      HCI_DEVCOREDUMP_ABORT: The driver can cancel ongoing dump collection in
 *              case of any error using hci_devcoredump_abort(). A devcoredump
 *              is still generated with the available data indicating the abort
 *              event and then the state machine is reset to the default state.
 *
 *      HCI_DEVCOREDUMP_TIMEOUT: A timeout timer for HCI_DEVCOREDUMP_TIMEOUT sec
 *              is started during devcoredump initialization. Once the timeout
 *              occurs, the driver is notified, a devcoredump is generated with
 *              the available data indicating the timeout event and then the
 *              state machine is reset to the default state.
 *
 * The driver must register using hci_devcoredump_register() before using the
 * hci devcoredump APIs.
 */
void hci_devcoredump_rx(struct work_struct *work)
{
	struct hci_dev *hdev = container_of(work, struct hci_dev, dump.dump_rx);
	struct sk_buff *skb;
	struct hci_devcoredump_skb_pattern *pattern;
	u32 dump_size;
	int start_state;

#define DBG_UNEXPECTED_STATE() \
		bt_dev_dbg(hdev, \
			   "Unexpected packet (%d) for state (%d). ", \
			   hci_dmp_cb(skb)->pkt_type, hdev->dump.state)

	while ((skb = skb_dequeue(&hdev->dump.dump_q))) {
		hci_dev_lock(hdev);
		start_state = hdev->dump.state;

		switch (hci_dmp_cb(skb)->pkt_type) {
		case HCI_DEVCOREDUMP_PKT_INIT:
			if (hdev->dump.state != HCI_DEVCOREDUMP_IDLE) {
				DBG_UNEXPECTED_STATE();
				goto loop_continue;
			}

			if (skb->len != sizeof(dump_size)) {
				bt_dev_dbg(hdev, "Invalid dump init pkt");
				goto loop_continue;
			}

			dump_size = *((u32 *)skb->data);
			if (!dump_size) {
				bt_dev_err(hdev, "Zero size dump init pkt");
				goto loop_continue;
			}

			if (hci_devcoredump_prepare(hdev, dump_size)) {
				bt_dev_err(hdev, "Failed to prepare for dump");
				goto loop_continue;
			}

			hci_devcoredump_update_state(hdev,
						     HCI_DEVCOREDUMP_ACTIVE);
			queue_delayed_work(hdev->workqueue,
					   &hdev->dump.dump_timeout,
					   DEVCOREDUMP_TIMEOUT);
			break;

		case HCI_DEVCOREDUMP_PKT_SKB:
			if (hdev->dump.state != HCI_DEVCOREDUMP_ACTIVE) {
				DBG_UNEXPECTED_STATE();
				goto loop_continue;
			}

			if (!hci_devcoredump_copy(hdev, skb->data, skb->len))
				bt_dev_dbg(hdev, "Failed to insert skb");
			break;

		case HCI_DEVCOREDUMP_PKT_PATTERN:
			if (hdev->dump.state != HCI_DEVCOREDUMP_ACTIVE) {
				DBG_UNEXPECTED_STATE();
				goto loop_continue;
			}

			if (skb->len != sizeof(*pattern)) {
				bt_dev_dbg(hdev, "Invalid pattern skb");
				goto loop_continue;
			}

			pattern = (void *)skb->data;

			if (!hci_devcoredump_memset(hdev, pattern->pattern,
						    pattern->len))
				bt_dev_dbg(hdev, "Failed to set pattern");
			break;

		case HCI_DEVCOREDUMP_PKT_COMPLETE:
			if (hdev->dump.state != HCI_DEVCOREDUMP_ACTIVE) {
				DBG_UNEXPECTED_STATE();
				goto loop_continue;
			}

			hci_devcoredump_update_state(hdev,
						     HCI_DEVCOREDUMP_DONE);
			dump_size = hdev->dump.tail - hdev->dump.head;

			bt_dev_info(hdev,
				    "Devcoredump complete with size %u "
				    "(expect %zu)",
				    dump_size, hdev->dump.alloc_size);

			dev_coredumpv(&hdev->dev, hdev->dump.head, dump_size,
				      GFP_KERNEL);
			break;

		case HCI_DEVCOREDUMP_PKT_ABORT:
			if (hdev->dump.state != HCI_DEVCOREDUMP_ACTIVE) {
				DBG_UNEXPECTED_STATE();
				goto loop_continue;
			}

			hci_devcoredump_update_state(hdev,
						     HCI_DEVCOREDUMP_ABORT);
			dump_size = hdev->dump.tail - hdev->dump.head;

			bt_dev_info(hdev,
				    "Devcoredump aborted with size %u "
				    "(expect %zu)",
				    dump_size, hdev->dump.alloc_size);

			/* Emit a devcoredump with the available data */
			dev_coredumpv(&hdev->dev, hdev->dump.head, dump_size,
				      GFP_KERNEL);
			break;

		default:
			bt_dev_dbg(hdev,
				   "Unknown packet (%d) for state (%d). ",
				   hci_dmp_cb(skb)->pkt_type, hdev->dump.state);
			break;
		}

loop_continue:
		kfree_skb(skb);
		hci_dev_unlock(hdev);

		if (start_state != hdev->dump.state)
			hci_devcoredump_notify(hdev, hdev->dump.state);

		hci_dev_lock(hdev);
		if (hdev->dump.state == HCI_DEVCOREDUMP_DONE ||
		    hdev->dump.state == HCI_DEVCOREDUMP_ABORT)
			hci_devcoredump_reset(hdev);
		hci_dev_unlock(hdev);
	}
}
EXPORT_SYMBOL(hci_devcoredump_rx);

void hci_devcoredump_timeout(struct work_struct *work)
{
	struct hci_dev *hdev = container_of(work, struct hci_dev,
					    dump.dump_timeout.work);
	u32 dump_size;

	hci_devcoredump_notify(hdev, HCI_DEVCOREDUMP_TIMEOUT);

	hci_dev_lock(hdev);

	cancel_work_sync(&hdev->dump.dump_rx);

	hci_devcoredump_update_state(hdev, HCI_DEVCOREDUMP_TIMEOUT);
	dump_size = hdev->dump.tail - hdev->dump.head;
	bt_dev_info(hdev, "Devcoredump timeout with size %u (expect %zu)",
		    dump_size, hdev->dump.alloc_size);

	/* Emit a devcoredump with the available data */
	dev_coredumpv(&hdev->dev, hdev->dump.head, dump_size, GFP_KERNEL);

	hci_devcoredump_reset(hdev);

	hci_dev_unlock(hdev);
}
EXPORT_SYMBOL(hci_devcoredump_timeout);

int hci_devcoredump_register(struct hci_dev *hdev, coredump_t coredump,
			     dmp_hdr_t dmp_hdr, notify_change_t notify_change)
{
	/* Driver must implement coredump() and dmp_hdr() functions for
	 * bluetooth devcoredump. The coredump() should trigger a coredump
	 * event on the controller when the device's coredump sysfs entry is
	 * written to. The dmp_hdr() should create a dump header to identify
	 * the controller/fw/driver info.
	 */
	if (!coredump || !dmp_hdr)
		return -EINVAL;

	hci_dev_lock(hdev);
	hdev->dump.coredump = coredump;
	hdev->dump.dmp_hdr = dmp_hdr;
	hdev->dump.notify_change = notify_change;
	hdev->dump.supported = true;
	hci_dev_unlock(hdev);

	return 0;
}
EXPORT_SYMBOL(hci_devcoredump_register);

static inline bool hci_devcoredump_enabled(struct hci_dev *hdev)
{
	/* The 'supported' flag is true when the driver registers with the HCI
	 * devcoredump API, whereas, the 'enabled' is controlled via a sysfs
	 * entry. For drivers like btusb which supports multiple vendor drivers,
	 * it is possible that the vendor driver does not support but the
	 * interface is provided by the base btusb driver. So, check both.
	 */
	if (hdev->dump.supported && hdev->dump.enabled)
		return hdev->dump.enabled(hdev);

	return false;
}

int hci_devcoredump_init(struct hci_dev *hdev, u32 dmp_size)
{
	struct sk_buff *skb = NULL;

	if (!hci_devcoredump_enabled(hdev))
		return -EOPNOTSUPP;

	skb = alloc_skb(sizeof(dmp_size), GFP_ATOMIC);
	if (!skb) {
		bt_dev_err(hdev, "Failed to allocate devcoredump init");
		return -ENOMEM;
	}

	hci_dmp_cb(skb)->pkt_type = HCI_DEVCOREDUMP_PKT_INIT;
	skb_put_data(skb, &dmp_size, sizeof(dmp_size));

	skb_queue_tail(&hdev->dump.dump_q, skb);
	queue_work(hdev->workqueue, &hdev->dump.dump_rx);

	return 0;
}
EXPORT_SYMBOL(hci_devcoredump_init);

int hci_devcoredump_append(struct hci_dev *hdev, struct sk_buff *skb)
{
	if (!skb)
		return -ENOMEM;

	if (!hci_devcoredump_enabled(hdev)) {
		kfree_skb(skb);
		return -EOPNOTSUPP;
	}

	hci_dmp_cb(skb)->pkt_type = HCI_DEVCOREDUMP_PKT_SKB;

	skb_queue_tail(&hdev->dump.dump_q, skb);
	queue_work(hdev->workqueue, &hdev->dump.dump_rx);

	return 0;
}
EXPORT_SYMBOL(hci_devcoredump_append);

int hci_devcoredump_append_pattern(struct hci_dev *hdev, u8 pattern, u32 len)
{
	struct hci_devcoredump_skb_pattern p;
	struct sk_buff *skb = NULL;

	if (!hci_devcoredump_enabled(hdev))
		return -EOPNOTSUPP;

	skb = alloc_skb(sizeof(p), GFP_ATOMIC);
	if (!skb) {
		bt_dev_err(hdev, "Failed to allocate devcoredump pattern");
		return -ENOMEM;
	}

	p.pattern = pattern;
	p.len = len;

	hci_dmp_cb(skb)->pkt_type = HCI_DEVCOREDUMP_PKT_PATTERN;
	skb_put_data(skb, &p, sizeof(p));

	skb_queue_tail(&hdev->dump.dump_q, skb);
	queue_work(hdev->workqueue, &hdev->dump.dump_rx);

	return 0;
}
EXPORT_SYMBOL(hci_devcoredump_append_pattern);

int hci_devcoredump_complete(struct hci_dev *hdev)
{
	struct sk_buff *skb = NULL;

	if (!hci_devcoredump_enabled(hdev))
		return -EOPNOTSUPP;

	skb = alloc_skb(0, GFP_ATOMIC);
	if (!skb) {
		bt_dev_err(hdev, "Failed to allocate devcoredump complete");
		return -ENOMEM;
	}

	hci_dmp_cb(skb)->pkt_type = HCI_DEVCOREDUMP_PKT_COMPLETE;

	skb_queue_tail(&hdev->dump.dump_q, skb);
	queue_work(hdev->workqueue, &hdev->dump.dump_rx);

	return 0;
}
EXPORT_SYMBOL(hci_devcoredump_complete);

int hci_devcoredump_abort(struct hci_dev *hdev)
{
	struct sk_buff *skb = NULL;

	if (!hci_devcoredump_enabled(hdev))
		return -EOPNOTSUPP;

	skb = alloc_skb(0, GFP_ATOMIC);
	if (!skb) {
		bt_dev_err(hdev, "Failed to allocate devcoredump abort");
		return -ENOMEM;
	}

	hci_dmp_cb(skb)->pkt_type = HCI_DEVCOREDUMP_PKT_ABORT;

	skb_queue_tail(&hdev->dump.dump_q, skb);
	queue_work(hdev->workqueue, &hdev->dump.dump_rx);

	return 0;
}
EXPORT_SYMBOL(hci_devcoredump_abort);
