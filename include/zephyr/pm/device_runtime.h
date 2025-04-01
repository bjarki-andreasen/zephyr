/*
 * Copyright (c) 2015 Intel Corporation.
 * Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_PM_DEVICE_RUNTIME_H_
#define ZEPHYR_INCLUDE_PM_DEVICE_RUNTIME_H_

#include <zephyr/device.h>
#include <zephyr/kernel.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Device Runtime Power Management API
 * @defgroup subsys_pm_device_runtime Device Runtime
 * @ingroup subsys_pm
 * @{
 */

#if defined(CONFIG_PM_DEVICE_RUNTIME) || defined(__DOXYGEN__)
/**
 * @brief Resume a device based on usage count.
 *
 * This function will resume the device if the device is suspended (usage count
 * equal to 0). In case of a resume failure, usage count and device state will
 * be left unchanged. In all other cases, usage count will be incremented.
 *
 * If the device is still being suspended as a result of calling
 * pm_device_runtime_put_async(), this function will wait for the operation to
 * finish to then resume the device.
 *
 * @note It is safe to use this function in contexts where blocking is not
 * allowed, e.g. ISR, provided the device PM implementation does not block.
 *
 * @funcprops \pre_kernel_ok
 *
 * @param dev Device instance.
 *
 * @retval 0 If it succeeds. In case device runtime PM is not enabled or not
 * available this function will be a no-op and will also return 0.
 * @retval -EWOUDBLOCK If call would block but it is not allowed (e.g. in ISR).
 * @retval -errno Other negative errno, result of the PM action callback.
 */
int pm_device_runtime_get(const struct device *dev);

/**
 * @brief Suspend a device based on usage count.
 *
 * This function will suspend the device if the device is no longer required
 * (usage count equal to 0). In case of suspend failure, usage count and device
 * state will be left unchanged. In all other cases, usage count will be
 * decremented (down to 0).
 *
 * @funcprops \pre_kernel_ok
 *
 * @param dev Device instance.
 *
 * @retval 0 If it succeeds. In case device runtime PM is not enabled or not
 * available this function will be a no-op and will also return 0.
 * @retval -EALREADY If device is already suspended (can only happen if get/put
 * calls are unbalanced).
 * @retval -errno Other negative errno, result of the action callback.
 *
 * @see pm_device_runtime_put_async()
 */
int pm_device_runtime_put(const struct device *dev);

/**
 * @brief Suspend a device based on usage count (asynchronously).
 *
 * This function will schedule the device suspension if the device is no longer
 * required (usage count equal to 0). In all other cases, usage count will be
 * decremented (down to 0).
 *
 * @note Asynchronous operations are not supported when in pre-kernel mode. In
 * this case, the function will be blocking (equivalent to
 * pm_device_runtime_put()).
 *
 * @funcprops \pre_kernel_ok, \async, \isr_ok
 *
 * @param dev Device instance.
 * @param delay Minimum amount of time before triggering the action.
 *
 * @retval 0 If it succeeds. In case device runtime PM is not enabled or not
 * available this function will be a no-op and will also return 0.
 * @retval -EBUSY If the device is busy.
 * @retval -EALREADY If device is already suspended (can only happen if get/put
 * calls are unbalanced).
 *
 * @see pm_device_runtime_put()
 */
int pm_device_runtime_put_async(const struct device *dev, k_timeout_t delay);

/**
 * @brief Return the current device usage counter.
 *
 * @param dev Device instance.
 *
 * @returns the current usage counter.
 * @retval -ENOTSUP If the device is not using runtime PM.
 * @retval -ENOSYS If the runtime PM is not enabled at all.
 */
int pm_device_runtime_usage(const struct device *dev);

/**
 * @brief Initialize PM device runtime context.
 *
 * @param dev Device instance.
 */
void pm_device_runtime_init(const struct device *dev);

#else

static inline int pm_device_runtime_get(const struct device *dev)
{
	ARG_UNUSED(dev);
	return 0;
}

static inline int pm_device_runtime_put(const struct device *dev)
{
	ARG_UNUSED(dev);
	return 0;
}

static inline int pm_device_runtime_put_async(const struct device *dev,
		k_timeout_t delay)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(delay);
	return 0;
}

static inline int pm_device_runtime_usage(const struct device *dev)
{
	ARG_UNUSED(dev);
	return -ENOSYS;
}

static inline void pm_device_runtime_init(const struct device *dev)
{
	ARG_UNUSED(dev);
}

#endif

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_PM_DEVICE_RUNTIME_H_ */
