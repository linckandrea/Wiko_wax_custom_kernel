/*
 * Copyright (c) 2012-2014, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef _NVSHM_IPC_H
#define _NVSHM_IPC_H

/**
 * Register IPC for handle
 *
 * @param struct nvshm_handle
 * @return 0 if ok
 */
extern int nvshm_register_ipc(struct nvshm_handle *handle);

/**
 * Unregister IPC for handle
 *
 * @param struct nvshm_handle
 * @return 0 if ok
 */
extern int nvshm_unregister_ipc(struct nvshm_handle *handle);

/**
 * Generate an IPC interrupt with given mailbox content
 *
 * @param struct _nvshm_priv_handle
 * @return 0 if ok
 */
extern int nvshm_generate_ipc(struct nvshm_handle *handle);

/**
 * Trigger internal recovery on unrecoverable error
 *
 * @param none
 * @return none
 */
extern void nvshm_trigger_recovery(void);
#endif /* _NVHSM_IPC_H */
