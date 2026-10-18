/*
 * Copyright (c) 2022 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <soc_secure.h>
#include <errno.h>

#if !defined(CONFIG_TFM_USE_NS_APP)

#include "tfm_platform_api.h"
#include "tfm_ioctl_api.h"

int soc_secure_mem_read(void *dst, void *src, size_t len)
{
	enum tfm_platform_err_t status;
	uint32_t result;

	status = tfm_platform_mem_read(dst, (uintptr_t)src, len, &result);

	switch (status) {
	case TFM_PLATFORM_ERR_INVALID_PARAM:
		return -EINVAL;
	case TFM_PLATFORM_ERR_NOT_SUPPORTED:
		return -ENOTSUP;
	case TFM_PLATFORM_ERR_SUCCESS:
		if (result == 0) {
			return 0;
		}
		/* Fallthrough */
	default:
		return -EPERM;
	}
}

#else

int soc_secure_mem_read(void *dst, void *src, size_t len)
{
	return -EINVAL;
}

#endif /* !CONFIG_TFM_USE_NS_APP */
