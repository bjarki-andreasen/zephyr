/*
 * Copyright (c) 2018 Intel Corporation.
 * Copyright (c) 2021 Nordic Semiconductor ASA.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/pm/device.h>
#include <zephyr/pm/device_runtime.h>
#include <zephyr/sys/__assert.h>

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(pm_device, CONFIG_PM_DEVICE_LOG_LEVEL);

#ifdef CONFIG_PM_DEVICE_POWER_DOMAIN
#define PM_DOMAIN(_pm) \
	(_pm)->domain
#else
#define PM_DOMAIN(_pm) NULL
#endif

#define EVENT_STATE_ACTIVE	BIT(PM_DEVICE_STATE_ACTIVE)
#define EVENT_STATE_SUSPENDED	BIT(PM_DEVICE_STATE_SUSPENDED)

#define EVENT_MASK		(EVENT_STATE_ACTIVE | EVENT_STATE_SUSPENDED)

/**
 * @brief Suspend a device
 *
 * @note Asynchronous operations are not supported when in pre-kernel mode. In
 * this case, the async flag will be always forced to be false, and so the
 * function will be blocking.
 *
 * @funcprops \pre_kernel_ok
 *
 * @param dev Device instance.
 * @param async Perform operation asynchronously.
 * @param delay Period to delay the asynchronous operation.
 *
 * @retval 0 If device has been suspended or queued for suspend.
 * @retval -EALREADY If device is already suspended (can only happen if get/put
 * calls are unbalanced).
 * @retval -EBUSY If the device is busy.
 * @retval -errno Other negative errno, result of the action callback.
 */
int runtime_suspend(const struct device *dev, bool async, k_timeout_t delay)
{
	int ret = 0;
	struct pm_device *pm = dev->pm;

	if (k_is_pre_kernel()) {
		async = false;
	} else {
		ret = k_sem_take(&pm->lock, k_is_in_isr() ? K_NO_WAIT : K_FOREVER);
		if (ret < 0) {
			return -EBUSY;
		}
	}

	if (pm->base.usage == 0U) {
		LOG_WRN("Unbalanced suspend");
		ret = -EALREADY;
		goto unlock;
	}

	pm->base.usage--;
	if (pm->base.usage > 0U) {
		goto unlock;
	}

	if (async) {
		/* queue suspend */
		pm->base.state = PM_DEVICE_STATE_SUSPENDING;
		(void)k_work_schedule(&pm->work, delay);
	} else {
		/* suspend now */
		ret = pm->base.action_cb(pm->dev, PM_DEVICE_ACTION_SUSPEND);
		if (ret < 0) {
			pm->base.usage++;
			goto unlock;
		}

		pm->base.state = PM_DEVICE_STATE_SUSPENDED;
	}

unlock:
	if (!k_is_pre_kernel()) {
		k_sem_give(&pm->lock);
	}

	return ret;
}

static void runtime_suspend_work(struct k_work *work)
{
	int ret;
	struct k_work_delayable *dwork = k_work_delayable_from_work(work);
	struct pm_device *pm = CONTAINER_OF(dwork, struct pm_device, work);

	ret = pm->base.action_cb(pm->dev, PM_DEVICE_ACTION_SUSPEND);

	(void)k_sem_take(&pm->lock, K_FOREVER);
	if (ret < 0) {
		pm->base.usage++;
		pm->base.state = PM_DEVICE_STATE_ACTIVE;
	} else {
		pm->base.state = PM_DEVICE_STATE_SUSPENDED;
	}
	k_event_set(&pm->event, BIT(pm->base.state));
	k_sem_give(&pm->lock);

	/*
	 * On async put, we have to suspend the domain when the device
	 * finishes its operation
	 */
	if ((ret == 0) &&
	    atomic_test_bit(&pm->base.flags, PM_DEVICE_FLAG_PD_CLAIMED)) {
		(void)pm_device_runtime_put(PM_DOMAIN(&pm->base));
	}

	__ASSERT(ret == 0, "Could not suspend device (%d)", ret);
}

static int get_sync_locked(const struct device *dev)
{
	int ret;
	struct pm_device_isr *pm = dev->pm_isr;
	uint32_t flags = pm->base.flags;

	if (pm->base.usage == 0) {
		if (flags & BIT(PM_DEVICE_FLAG_PD_CLAIMED)) {
			const struct device *domain = PM_DOMAIN(&pm->base);

			if (domain->pm_base->flags & BIT(PM_DEVICE_FLAG_ISR_SAFE)) {
				ret = pm_device_runtime_get(domain);
				if (ret < 0) {
					return ret;
				}
			} else {
				return -EWOULDBLOCK;
			}
		}

		ret = pm->base.action_cb(dev, PM_DEVICE_ACTION_RESUME);
		if (ret < 0) {
			return ret;
		}
		pm->base.state = PM_DEVICE_STATE_ACTIVE;
	} else {
		ret = 0;
	}

	pm->base.usage++;

	return ret;
}

int pm_device_runtime_get(const struct device *dev)
{
	int ret = 0;
	struct pm_device *pm = dev->pm;

	if (pm == NULL) {
		return 0;
	}

	SYS_PORT_TRACING_FUNC_ENTER(pm, device_runtime_get, dev);

	if (atomic_test_bit(&dev->pm_base->flags, PM_DEVICE_FLAG_ISR_SAFE)) {
		struct pm_device_isr *pm_sync = dev->pm_isr;
		k_spinlock_key_t k = k_spin_lock(&pm_sync->lock);

		ret = get_sync_locked(dev);
		k_spin_unlock(&pm_sync->lock, k);
		goto end;
	}

	if (!k_is_pre_kernel()) {
		ret = k_sem_take(&pm->lock, k_is_in_isr() ? K_NO_WAIT : K_FOREVER);
		if (ret < 0) {
			return -EWOULDBLOCK;
		}
	}

	if (k_is_in_isr() && (pm->base.state == PM_DEVICE_STATE_SUSPENDING)) {
		ret = -EWOULDBLOCK;
		goto unlock;
	}

	/*
	 * If the device is under a power domain, the domain has to be get
	 * first.
	 */
	const struct device *domain = PM_DOMAIN(&pm->base);

	if (domain != NULL) {
		ret = pm_device_runtime_get(domain);
		if (ret != 0) {
			goto unlock;
		}
		/* Check if powering up this device failed */
		if (atomic_test_bit(&pm->base.flags, PM_DEVICE_FLAG_TURN_ON_FAILED)) {
			(void)pm_device_runtime_put(domain);
			ret = -EAGAIN;
			goto unlock;
		}
		/* Power domain successfully claimed */
		atomic_set_bit(&pm->base.flags, PM_DEVICE_FLAG_PD_CLAIMED);
	}

	pm->base.usage++;

	/*
	 * Check if the device has a pending suspend operation (not started
	 * yet) and cancel it. This way we avoid unnecessary operations because
	 * the device is actually active.
	 */
	if ((pm->base.state == PM_DEVICE_STATE_SUSPENDING) &&
		((k_work_cancel_delayable(&pm->work) & K_WORK_RUNNING) == 0)) {
		pm->base.state = PM_DEVICE_STATE_ACTIVE;
		goto unlock;
	}

	if (!k_is_pre_kernel()) {
		/*
		 * If the device is already suspending there is
		 * nothing else we can do but wait until it finishes.
		 */
		while (pm->base.state == PM_DEVICE_STATE_SUSPENDING) {
			k_event_clear(&pm->event, EVENT_MASK);
			k_sem_give(&pm->lock);

			k_event_wait(&pm->event, EVENT_MASK, false, K_FOREVER);

			(void)k_sem_take(&pm->lock, K_FOREVER);
		}
	}

	if (pm->base.usage > 1U) {
		goto unlock;
	}

	ret = pm->base.action_cb(pm->dev, PM_DEVICE_ACTION_RESUME);
	if (ret < 0) {
		pm->base.usage--;
		goto unlock;
	}

	pm->base.state = PM_DEVICE_STATE_ACTIVE;

unlock:
	if (!k_is_pre_kernel()) {
		k_sem_give(&pm->lock);
	}

end:
	SYS_PORT_TRACING_FUNC_EXIT(pm, device_runtime_get, dev, ret);

	return ret;
}

static int put_sync_locked(const struct device *dev)
{
	int ret;
	struct pm_device_isr *pm = dev->pm_isr;
	uint32_t flags = pm->base.flags;

	if (pm->base.usage == 0U) {
		return -EALREADY;
	}

	pm->base.usage--;
	if (pm->base.usage == 0U) {
		ret = pm->base.action_cb(dev, PM_DEVICE_ACTION_SUSPEND);
		if (ret < 0) {
			return ret;
		}
		pm->base.state = PM_DEVICE_STATE_SUSPENDED;

		if (flags & BIT(PM_DEVICE_FLAG_PD_CLAIMED)) {
			const struct device *domain = PM_DOMAIN(&pm->base);

			if (domain->pm_base->flags & BIT(PM_DEVICE_FLAG_ISR_SAFE)) {
				ret = put_sync_locked(domain);
			} else {
				ret = -EWOULDBLOCK;
			}
		}
	} else {
		ret = 0;
	}

	return ret;
}

int pm_device_runtime_put(const struct device *dev)
{
	int ret;

	if (dev->pm_base == NULL) {
		return 0;
	}

	SYS_PORT_TRACING_FUNC_ENTER(pm, device_runtime_put, dev);

	if (atomic_test_bit(&dev->pm_base->flags, PM_DEVICE_FLAG_ISR_SAFE)) {
		struct pm_device_isr *pm_sync = dev->pm_isr;
		k_spinlock_key_t k = k_spin_lock(&pm_sync->lock);

		ret = put_sync_locked(dev);

		k_spin_unlock(&pm_sync->lock, k);
	} else {
		ret = runtime_suspend(dev, false, K_NO_WAIT);

		/*
		 * Now put the domain
		 */
		if ((ret == 0) &&
		    atomic_test_bit(&dev->pm_base->flags, PM_DEVICE_FLAG_PD_CLAIMED)) {
			ret = pm_device_runtime_put(PM_DOMAIN(dev->pm_base));
		}
	}
	SYS_PORT_TRACING_FUNC_EXIT(pm, device_runtime_put, dev, ret);

	return ret;
}

int pm_device_runtime_put_async(const struct device *dev, k_timeout_t delay)
{
	int ret;

	if (dev->pm_base == NULL) {
		return 0;
	}

	SYS_PORT_TRACING_FUNC_ENTER(pm, device_runtime_put_async, dev, delay);
	if (atomic_test_bit(&dev->pm_base->flags, PM_DEVICE_FLAG_ISR_SAFE)) {
		struct pm_device_isr *pm_sync = dev->pm_isr;
		k_spinlock_key_t k = k_spin_lock(&pm_sync->lock);

		ret = put_sync_locked(dev);

		k_spin_unlock(&pm_sync->lock, k);
	} else {
		ret = runtime_suspend(dev, true, delay);
	}
	SYS_PORT_TRACING_FUNC_EXIT(pm, device_runtime_put_async, dev, delay, ret);

	return ret;
}

int pm_device_runtime_usage(const struct device *dev)
{
	return dev->pm_base->usage;
}

void pm_device_runtime_init(const struct device *dev)
{
	struct pm_device *pm = dev->pm;

	pm->dev = dev;
	k_work_init_delayable(&pm->work, runtime_suspend_work);
}
