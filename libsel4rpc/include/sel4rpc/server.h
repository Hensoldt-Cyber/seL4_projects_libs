/*
 * Copyright 2019, Data61
 * Commonwealth Scientific and Industrial Research Organisation (CSIRO)
 * ABN 41 687 119 230.
 *
 * This software may be distributed and modified according to the terms of
 * the BSD 2-Clause license. Note that NO WARRANTY is provided.
 * See "LICENSE_BSD2.txt" for details.
 *
 * @TAG(DATA61_BSD)
 */

#pragma once

#include <simple/simple.h>
#include <vka/object.h>
#include <vka/vka.h>
#include <rpc.pb.h>

#define SEL4RPC_MSG_MAGIC (0xcafed00d)

struct sel4rpc_env;

typedef int (*sel4rpc_handler_t)(struct sel4rpc_env *env, void *data, RpcMessage *rpcMsg);
typedef struct sel4rpc_env {
    vka_t *vka;

    vka_object_t *reply;

    sel4rpc_handler_t handler;
    void *data;

    simple_t *simple;
} sel4rpc_server_env_t;

int sel4rpc_server_init(sel4rpc_server_env_t *env, vka_t *vka,
                        sel4rpc_handler_t handler_func, void *data, vka_object_t *reply, simple_t *simple);
int sel4rpc_server_recv(sel4rpc_server_env_t *env);
int sel4rpc_server_reply(sel4rpc_server_env_t *env, int caps, int errorCode);
int sel4rpc_default_handler(sel4rpc_server_env_t *env, UNUSED void *data, RpcMessage *rpcMsg);

