/*
 * Copyright 2019, Data61, CSIRO (ABN 41 687 119 230)
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "vm.h"


/* The ARM GIC architecture defines 16 SGIs (0 - 7 is recommended for non-secure
 * state, 8 - 15 for secure state), 16 PPIs (interrupt 16 - 31) and 988 SPIs
 * (32 - 1019). The interrupt IDs 1020 - 1023 are used for special purposes.
 * GICv3.1 is not implemented here, it supports 64 additional PPIs (interrupt
 * 1056 - 1119) and 1024 SPIs (interrupt 4096 – 5119). LPIs starting at
 * interrupt 8192 are also not implemented here.
 */
#define NUM_SGI_VIRQS           16   // vCPU local SGI interrupts
#define NUM_PPI_VIRQS           16   // vCPU local PPI interrupts
#define NUM_VCPU_LOCAL_VIRQS    (NUM_SGI_VIRQS + NUM_PPI_VIRQS)

/* Usually, VMs do not use all SPIs. To reduce the memory footprint, our vGIC
 * implementation manages the SPIs in a fixed size slot list. 200 entries have
 * been good trade-off that is sufficient for most systems.
 */
#define NUM_SLOTS_SPI_VIRQ      200

struct virq_handle {
    int virq;
    irq_ack_fn_t ack;
    void *token;
};

typedef struct virq_handle *virq_handle_t;

static inline void virq_ack(vm_vcpu_t *vcpu, struct virq_handle *irq)
{
    irq->ack(vcpu, irq->virq, irq->token);
}

/* TODO: A typical number of list registers supported by GIC is four, but not
 * always. One particular way to probe the number of registers is to inject a
 * dummy IRQ with seL4_ARM_VCPU_InjectIRQ(), using LR index high enough to be
 * not supported by any target; the kernel will reply with the supported range
 * of LR indexes.
 */
#define NUM_LIST_REGS 4
/* This is a rather arbitrary number, increase if needed. */
#define MAX_IRQ_QUEUE_LEN 64
#define IRQ_QUEUE_NEXT(_i) (((_i) + 1) & (MAX_IRQ_QUEUE_LEN - 1))

static_assert((MAX_IRQ_QUEUE_LEN & (MAX_IRQ_QUEUE_LEN - 1)) == 0,
              "IRQ ring buffer size must be power of two");

struct irq_queue {
    struct virq_handle *irqs[MAX_IRQ_QUEUE_LEN]; /* circular buffer */
    size_t head;
    size_t tail;
};

/* vCPU specific interrupt context */
typedef struct vgic_vcpu {
    /* Mirrors the GIC's vCPU list registers */
    virq_handle_t lr_shadow[NUM_LIST_REGS];
    /* Queue for IRQs that don't fit in the GIC's vCPU list registers */
    struct irq_queue irq_queue;
    /*  vCPU local interrupts (SGI, PPI) */
    virq_handle_t local_virqs[NUM_VCPU_LOCAL_VIRQS];
} vgic_vcpu_t;

/* GIC global interrupt context */
typedef struct vgic {
    /* virtual distributor registers */
    struct gic_dist_map *dist;
    /* registered global interrupts (SPI) */
    virq_handle_t vspis[NUM_SLOTS_SPI_VIRQ];
    /* vCPU specific interrupt context */
    vgic_vcpu_t vgic_vcpu[CONFIG_MAX_NUM_NODES];
} vgic_t;

static inline vgic_vcpu_t *get_vgic_vcpu(vgic_t *vgic, int vcpu_id)
{
    assert(vgic);
    assert((vcpu_id >= 0) && (vcpu_id < ARRAY_SIZE(vgic->vgic_vcpu)));
    return &(vgic->vgic_vcpu[vcpu_id]);
}

static inline struct virq_handle *virq_get_sgi_ppi(vgic_t *vgic, vm_vcpu_t *vcpu, int virq)
{
    vgic_vcpu_t *vgic_vcpu = get_vgic_vcpu(vgic, vcpu->vcpu_id);
    assert(vgic_vcpu);
    assert((virq >= 0) && (virq < ARRAY_SIZE(vgic_vcpu->local_virqs)));
    return vgic_vcpu->local_virqs[virq];
}

static inline struct virq_handle *virq_find_spi_irq_data(struct vgic *vgic, int virq)
{
    for (int i = 0; i < ARRAY_SIZE(vgic->vspis); i++) {
        if (vgic->vspis[i] && vgic->vspis[i]->virq == virq) {
            return vgic->vspis[i];
        }
    }
    return NULL;
}

static inline struct virq_handle *virq_find_irq_data(struct vgic *vgic, vm_vcpu_t *vcpu, int virq)
{
    if (virq < NUM_VCPU_LOCAL_VIRQS)  {
        return virq_get_sgi_ppi(vgic, vcpu, virq);
    }
    return virq_find_spi_irq_data(vgic, virq);
}

static inline int virq_register(vm_vcpu_t *vcpu, vgic_t *vgic, int irq,
                                irq_ack_fn_t ack_fn, void *token)
{
    assert(vgic);
    assert(vcpu);
    assert(irq >= 0);

    vgic_vcpu_t *vgic_vcpu = get_vgic_vcpu(vgic, vcpu->vcpu_id);
    assert(vgic_vcpu);

    /* Find a slot for the virq to be created */
    struct virq_handle **slot = NULL;
    if (irq < ARRAY_SIZE(vgic_vcpu->local_virqs)) {
        /* Add a local interrupt (SGI, PPI). */
        slot = &vgic_vcpu->local_virqs[irq];
        if (NULL != *slot) {
            ZF_LOGE("IRQ %d already registered as SGI/PPI on VCPU %u",
                    irq, vcpu->vcpu_id);
            return -1;
        }
    } else {
        /* Find a free slot to register a global interrupt (SPI). */
        for (int i = 0; i < ARRAY_SIZE(vgic->vspis); i++) {
            slot = &vgic->vspis[i];
            if (NULL == *slot) {
                break;
            }
        }
        if (NULL == slot) {
            ZF_LOGE("NUM_SLOTS_SPI_VIRQ exceeded, can't register IRQ %d as SPI",
                    irq);
            return -1;
        }
    }

    assert(NULL != slot); /* There must be a valid slot here. */

    /* Allocate and initialize new virq. */
    struct virq_handle *virq = (typeof(virq))calloc(1, sizeof(*virq));
    if (!virq) {
        ZF_LOGE("virq allocation failed for IRQ %d", irq);
        return -1;
    }
    *virq = (typeof(*virq)) {
        .virq  = irq,
        .token = cookie,
        .ack   = ack_fn
    };
    /* Add virq to slot. */
    *slot = virq;

    return 0;
}

static inline void virq_init(virq_handle_t virq, int irq, irq_ack_fn_t ack_fn, void *token)
{
    virq->virq = irq;
    virq->token = token;
    virq->ack = ack_fn;
}

static inline int vgic_irq_enqueue(vgic_t *vgic, vm_vcpu_t *vcpu, struct virq_handle *irq)
{
    vgic_vcpu_t *vgic_vcpu = get_vgic_vcpu(vgic, vcpu->vcpu_id);
    assert(vgic_vcpu);
    struct irq_queue *q = &vgic_vcpu->irq_queue;

    if (unlikely(IRQ_QUEUE_NEXT(q->tail) == q->head)) {
        return -1;
    }

    q->irqs[q->tail] = irq;
    q->tail = IRQ_QUEUE_NEXT(q->tail);

    return 0;
}

static inline struct virq_handle *vgic_irq_dequeue(vgic_t *vgic, vm_vcpu_t *vcpu)
{
    vgic_vcpu_t *vgic_vcpu = get_vgic_vcpu(vgic, vcpu->vcpu_id);
    assert(vgic_vcpu);
    struct irq_queue *q = &vgic_vcpu->irq_queue;
    struct virq_handle *virq = NULL;

    if (q->head != q->tail) {
        virq = q->irqs[q->head];
        q->head = IRQ_QUEUE_NEXT(q->head);
    }

    return virq;
}

static inline int vgic_find_empty_list_reg(vgic_t *vgic, vm_vcpu_t *vcpu)
{
    vgic_vcpu_t *vgic_vcpu = get_vgic_vcpu(vgic, vcpu->vcpu_id);
    assert(vgic_vcpu);
    for (int i = 0; i < ARRAY_SIZE(vgic_vcpu->lr_shadow); i++) {
        if (vgic_vcpu->lr_shadow[i] == NULL) {
            return i;
        }
    }

    return -1;
}

static inline int vgic_vcpu_load_list_reg(vgic_t *vgic, vm_vcpu_t *vcpu, int idx, int group, struct virq_handle *irq)
{
    vgic_vcpu_t *vgic_vcpu = get_vgic_vcpu(vgic, vcpu->vcpu_id);
    assert(vgic_vcpu);
    assert((idx >= 0) && (idx < ARRAY_SIZE(vgic_vcpu->lr_shadow)));

    int err = seL4_ARM_VCPU_InjectIRQ(vcpu->vcpu.cptr, irq->virq, 0, group, idx);
    if (err) {
        ZF_LOGF("Failure loading vGIC list register (error %d)", err);
        return err;
    }

    vgic_vcpu->lr_shadow[idx] = irq;

    return 0;
}
