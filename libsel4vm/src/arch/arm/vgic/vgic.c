/*
 * Copyright 2019, Data61, CSIRO (ABN 41 687 119 230)
 * Copyright 2019, DornerWorks
 * Copyright 2022, HENSOLDT Cyber
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */
/*
 * This component controls and maintains the GIC for the VM.
 * IRQs must be registered at init time with vm_virq_new(...)
 * This function creates and registers an IRQ data structure which will be used for IRQ maintenance
 * b) ENABLING: When the VM enables the IRQ, it checks the pending flag for the VM.
 *   - If the IRQ is not pending, we either
 *        1) have not received an IRQ so it is still enabled in seL4
 *        2) have received an IRQ, but ignored it because the VM had disabled it.
 *     In either case, we simply ACK the IRQ with seL4. In case 1), the IRQ will come straight through,
       in case 2), we have ACKed an IRQ that was not yet pending anyway.
 *   - If the IRQ is already pending, we can assume that the VM has yet to ACK the IRQ and take no further
 *     action.
 *   Transitions: b->c
 * c) PIRQ: When an IRQ is received from seL4, seL4 disables the IRQ and sends an async message. When the VMM
 *    receives the message.
 *   - If the IRQ is enabled, we set the pending flag in the VM and inject the appropriate IRQ
 *     leading to state d)
 *   - If the IRQ is disabled, the VMM takes no further action, leading to state b)
 *   Transitions: (enabled)? c->d :  c->b
 * d) When the VM acknowledges the IRQ, an exception is raised and delivered to the VMM. When the VMM
 *    receives the exception, it clears the pending flag and acks the IRQ with seL4, leading back to state c)
 *    Transition: d->c
 * g) When/if the VM disables the IRQ, we may still have an IRQ resident in the GIC. We allow
 *    this IRQ to be delivered to the VM, but subsequent IRQs will not be delivered as seen by state c)
 *    Transitions g->c
 *
 *   NOTE: There is a big assumption that the VM will not manually manipulate our pending flags and
 *         destroy our state. The affects of this will be an IRQ that is never acknowledged and hence,
 *         will never occur again.
 */

#include "vgic.h"

#include <stdint.h>
#include <string.h>
#include <stdlib.h>

#include <utils/arith.h>
#include <vka/vka.h>
#include <vka/capops.h>

#include <sel4vm/gen_config.h>
#include <sel4vm/guest_vm.h>
#include <sel4vm/boot.h>
#include <sel4vm/guest_memory.h>
#include <sel4vm/guest_irq_controller.h>
#include <sel4vm/guest_vm_util.h>

#include "vm.h"
#include "../fault.h"

#define GIC_V2
//#define GIC_V3

#if defined(GIC_V2)
#include "vgicv2_defs.h"
#elif defined(GIC_V3)
#include "vgicv3_defs.h"
#else
#error "set GIC_V2 or GIC_V3"
#endif

//#define DEBUG_IRQ
//#define DEBUG_DIST

#ifdef DEBUG_IRQ
#define DIRQ(...) do{ printf("VDIST: "); printf(__VA_ARGS__); }while(0)
#else
#define DIRQ(...) do{}while(0)
#endif

#ifdef DEBUG_DIST
#define DDIST(...) do{ printf("VDIST: "); printf(__VA_ARGS__); }while(0)
#else
#define DDIST(...) do{}while(0)
#endif


#if defined(GIC_V2)

/* FIXME these should be defined in a way that is friendlier to extension. */
#if defined(CONFIG_PLAT_EXYNOS5)
#define GIC_PADDR   0x10480000
#elif defined(CONFIG_PLAT_TK1) || defined(CONFIG_PLAT_TX1)
#define GIC_PADDR   0x50040000
#elif defined(CONFIG_PLAT_TX2)
#define GIC_PADDR   0x03880000
#elif defined(CONFIG_PLAT_QEMU_ARM_VIRT)
#define GIC_PADDR   0x8000000
#elif defined(CONFIG_PLAT_ODROIDC2)
#define GIC_PADDR   0xc4300000
#else
#error "Unsupported platform for GIC"
#endif

#ifdef CONFIG_PLAT_QEMU_ARM_VIRT
#define GIC_DIST_PADDR       (GIC_PADDR)
#define GIC_CPU_PADDR        (GIC_PADDR + 0x00010000)
#define GIC_VCPU_CNTR_PADDR  (GIC_PADDR + 0x00030000)
#define GIC_VCPU_PADDR       (GIC_PADDR + 0x00040000)
#else
#define GIC_DIST_PADDR       (GIC_PADDR + 0x1000)
#define GIC_CPU_PADDR        (GIC_PADDR + 0x2000)
#define GIC_VCPU_CNTR_PADDR  (GIC_PADDR + 0x4000)
#define GIC_VCPU_PADDR       (GIC_PADDR + 0x6000)
#endif

#elif defined(GIC_V3)

#define GIC_DIST_PADDR      0x38800000
#define GIC_REDIST_PADDR    0x38880000

#define GIC_SGI_OFFSET      0x10000

#else
#error "set GIC_V2 or GIC_V3"
#endif


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

/* GIC Distributor register access utilities */
#define GIC_DIST_REGN(offset, reg) ((offset-reg)/sizeof(uint32_t))
#define RANGE32(a, b) a ... b + (sizeof(uint32_t)-1)

#define IRQ_IDX(irq) ((irq) / 32)
#define IRQ_BIT(irq) (1U << ((irq) % 32))

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

#if defined(GIC_V2)

typedef struct {
    uint32_t ctlr;                                      /* 0x000 */
    uint32_t typer;                                     /* 0x004 */
    uint32_t iidr;                                      /* 0x008 */

    uint32_t res1[29];                                  /* [0x00C, 0x080) */

    uint32_t irq_group0[CONFIG_MAX_NUM_NODES];          /* [0x080, 0x84) */
    uint32_t irq_group[31];                             /* [0x084, 0x100) */
    uint32_t enable_set0[CONFIG_MAX_NUM_NODES];         /* [0x100, 0x104) */
    uint32_t enable_set[31];                            /* [0x104, 0x180) */
    uint32_t enable_clr0[CONFIG_MAX_NUM_NODES];         /* [0x180, 0x184) */
    uint32_t enable_clr[31];                            /* [0x184, 0x200) */
    uint32_t pending_set0[CONFIG_MAX_NUM_NODES];        /* [0x200, 0x204) */
    uint32_t pending_set[31];                           /* [0x204, 0x280) */
    uint32_t pending_clr0[CONFIG_MAX_NUM_NODES];        /* [0x280, 0x284) */
    uint32_t pending_clr[31];                           /* [0x284, 0x300) */
    uint32_t active0[CONFIG_MAX_NUM_NODES];             /* [0x300, 0x304) */
    uint32_t active[31];                                /* [0x300, 0x380) */
    uint32_t active_clr0[CONFIG_MAX_NUM_NODES];         /* [0x380, 0x384) */
    uint32_t active_clr[31];                            /* [0x384, 0x400) */
    uint32_t priority0[CONFIG_MAX_NUM_NODES][8];        /* [0x400, 0x420) */
    uint32_t priority[247];                             /* [0x420, 0x7FC) */
    uint32_t res3;                                      /* 0x7FC */

    uint32_t targets0[CONFIG_MAX_NUM_NODES][8];         /* [0x800, 0x820) */
    uint32_t targets[247];                              /* [0x820, 0xBFC) */
    uint32_t res4;                                      /* 0xBFC */

    uint32_t config[64];                                /* [0xC00, 0xD00) */

    uint32_t spi[32];                                   /* [0xD00, 0xD80) */
    uint32_t res5[20];                                  /* [0xD80, 0xDD0) */
    uint32_t res6;                                      /* 0xDD0 */
    uint32_t legacy_int;                                /* 0xDD4 */
    uint32_t res7[2];                                   /* [0xDD8, 0xDE0) */
    uint32_t match_d;                                   /* 0xDE0 */
    uint32_t enable_d;                                  /* 0xDE4 */
    uint32_t res8[70];                                  /* [0xDE8, 0xF00) */

    uint32_t sgir;                                      /* 0xF00 */
    uint32_t res9[3];                                   /* [0xF04, 0xF10) */

    uint32_t sgi_pending_clr[CONFIG_MAX_NUM_NODES][4];  /* [0xF10, 0xF20) */
    uint32_t sgi_pending_set[CONFIG_MAX_NUM_NODES][4];  /* [0xF20, 0xF30) */
    uint32_t res10[40];                                 /* [0xF30, 0xFC0) */

    uint32_t periph_id[12];                             /* [0xFC0, 0xFF0) */
    uint32_t component_id[4];                           /* [0xFF0, 0xFFF] */
} vgic_v2_dist_t;

#define ARM_INTR_GROUP   0

#elif defined(GIC_V3)

typedef struct {
    uint32_t ctlr;                /* 0x0000 */
    uint32_t typer;               /* 0x0004 */
    uint32_t iidr;                /* 0x0008 */
    uint32_t res1[13];            /* [0x000C, 0x0040) */
    uint32_t setspi_nsr;          /* 0x0040 */
    uint32_t res2;                /* 0x0044 */
    uint32_t clrspi_nsr;          /* 0x0048 */
    uint32_t res3;                /* 0x004C */
    uint32_t setspi_sr;           /* 0x0050 */
    uint32_t res4;                /* 0x0054 */
    uint32_t clrspi_sr;           /* 0x0058 */
    uint32_t res5[9];             /* [0x005C, 0x0080) */
    uint32_t irq_group0[CONFIG_MAX_NUM_NODES];          /* [0x080, 0x84) */
    uint32_t irq_group[31];                             /* [0x084, 0x100) */
    uint32_t enable_set0[CONFIG_MAX_NUM_NODES];         /* [0x100, 0x104) */
    uint32_t enable_set[31];                            /* [0x104, 0x180) */
    uint32_t enable_clr0[CONFIG_MAX_NUM_NODES];         /* [0x180, 0x184) */
    uint32_t enable_clr[31];                            /* [0x184, 0x200) */
    uint32_t pending_set0[CONFIG_MAX_NUM_NODES];        /* [0x200, 0x204) */
    uint32_t pending_set[31];                           /* [0x204, 0x280) */
    uint32_t pending_clr0[CONFIG_MAX_NUM_NODES];        /* [0x280, 0x284) */
    uint32_t pending_clr[31];                           /* [0x284, 0x300) */
    uint32_t active0[CONFIG_MAX_NUM_NODES];             /* [0x300, 0x304) */
    uint32_t active[31];                                /* [0x300, 0x380) */
    uint32_t active_clr0[CONFIG_MAX_NUM_NODES];         /* [0x380, 0x384) */
    uint32_t active_clr[32];                            /* [0x384, 0x400) */
    uint32_t priority0[CONFIG_MAX_NUM_NODES][8];        /* [0x400, 0x420) */
    uint32_t priority[247];                             /* [0x420, 0x7FC) */
    uint32_t res6;                  /* 0x7FC */

    uint32_t targets0[CONFIG_MAX_NUM_NODES][8];         /* [0x800, 0x820) */
    uint32_t targets[247];                              /* [0x820, 0xBFC) */
    uint32_t res7;                  /* 0xBFC */

    uint32_t config[64];            /* [0xC00, 0xD00) */
    uint32_t group_mod[64];         /* [0xD00, 0xE00) */
    uint32_t nsacr[64];             /* [0xE00, 0xF00) */
    uint32_t sgir;                  /* 0xF00 */
    uint32_t res8[3];               /* [0xF00, 0xF10) */
    uint32_t sgi_pending_clr[CONFIG_MAX_NUM_NODES][4];  /* [0xF10, 0xF20) */
    uint32_t sgi_pending_set[CONFIG_MAX_NUM_NODES][4];  /* [0xF20, 0xF30) */
    uint32_t res9[5235];            /* [0x0F30, 0x6100) */

    uint64_t irouter[960];          /* [0x6100, 0x7F00) */
    uint64_t res10[2080];           /* [0x7F00, 0xC000) */
    uint32_t estatusr;              /* 0xC000 */
    uint32_t errtestr;              /* 0xC004 */
    uint32_t res11[31];             /* [0xC008, 0xC084) */
    uint32_t spisr[30];             /* [0xC084, 0xC0FC) */
    uint32_t res12[4021];           /* [0xC0FC, 0xFFD0) */

    uint32_t pidrn[8];              /* [0xFFD0, 0xFFF0) */
    uint32_t cidrn[4];              /* [0xFFD0, 0xFFFC] */
} vgic_v3_dist_t;

/* Memory map for GIC Redistributor Registers for control and physical LPI's */
typedef struct {          /* Starting */
    uint32_t    ctlr;           /* 0x0000 */
    uint32_t    iidr;           /* 0x0004 */
    uint64_t    typer;          /* 0x008 */
    uint32_t    res0;           /* 0x0010 */
    uint32_t    waker;          /* 0x0014 */
    uint32_t    res1[21];       /* 0x0018 */
    uint64_t    propbaser;      /* 0x0070 */
    uint64_t    pendbaser;      /* 0x0078 */
    uint32_t    res2[16340];    /* 0x0080 */
    uint32_t    pidr4;          /* 0xFFD0 */
    uint32_t    pidr5;          /* 0xFFD4 */
    uint32_t    pidr6;          /* 0xFFD8 */
    uint32_t    pidr7;          /* 0xFFDC */
    uint32_t    pidr0;          /* 0xFFE0 */
    uint32_t    pidr1;          /* 0xFFE4 */
    uint32_t    pidr2;          /* 0xFFE8 */
    uint32_t    pidr3;          /* 0xFFEC */
    uint32_t    cidr0;          /* 0xFFF0 */
    uint32_t    cidr1;          /* 0xFFF4 */
    uint32_t    cidr2;          /* 0xFFF8 */
    uint32_t    cidr3;          /* 0xFFFC */
} vgic_v3_rdist_t;

/* Memory map for the GIC Redistributor Registers for the SGI and PPI's */
// struct gic_rdist_sgi_ppi_map {  /* Starting */
//     uint32_t    res0[32];       /* 0x0000 */
//     uint32_t    igroup[32];     /* 0x0080 */
//     uint32_t    isenable[32];   /* 0x0100 */
//     uint32_t    icenable[32];   /* 0x0180 */
//     uint32_t    ispend[32];     /* 0x0200 */
//     uint32_t    icpend[32];     /* 0x0280 */
//     uint32_t    isactive[32];   /* 0x0300 */
//     uint32_t    icactive[32];   /* 0x0380 */
//     uint32_t    ipriorityrn[8]; /* 0x0400 */
//     uint32_t    res1[504];      /* 0x0420 */
//     uint32_t    icfgrn_ro;      /* 0x0C00 */
//     uint32_t    icfgrn_rw;      /* 0x0C04 */
//     uint32_t    res2[62];       /* 0x0C08 */
//     uint32_t    igrpmod[64];    /* 0x0D00 */
//     uint32_t    nsac;           /* 0x0E00 */
//     uint32_t    res11[11391];   /* 0x0E04 */
//     uint32_t    miscstatsr;     /* 0xC000 */
//     uint32_t    res3[31];       /* 0xC004 */
//     uint32_t    ppisr;          /* 0xC080 */
//     uint32_t    res4[4062];     /* 0xC084 */
// };

#define GIC_MAX_DISABLE 32

/* bits in Distributor control register */
#define GIC_500_GRP0     BIT(0)
#define GIC_500_GRP1_NS  BIT(1)
#define GIC_500_GRP1_S   BIT(2)
#define GIC_500_ARE_S    BIT(4)
#define ARM_INTR_GROUP   GIC_500_GRP1_NS


#else
#error "set GIC_V2 or GIC_V3"
#endif


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

typedef struct {
    uintptr_t paddr;
    size_t size;
    vm_memory_reservation_t *vm_res;
} vm_mapping_t;

/* GIC global interrupt context */
typedef struct vgic {
    /* virtual distributor registers */
    vm_mapping_t mapped_dist;
#if defined(GIC_V2)
    vm_mapping_t mapped_cpu_if;
    vgic_v2_dist_t dist;
#elif defined(GIC_V3)
    vm_mapping_t mapped_rdist;
    vgic_v3_dist_t dist;
    vgic_v3_rdist_t rdist;
#else
#error "set GIC_V2 or GIC_V3"
#endif
    /* registered global interrupts (SPI) */
    virq_handle_t vspis[NUM_SLOTS_SPI_VIRQ];
    /* vCPU specific interrupt context */
    vgic_vcpu_t vgic_vcpu[CONFIG_MAX_NUM_NODES];
} vgic_t;

static vgic_t *get_vgic_from_vm(vm_t *vm)
{
    assert(vm);
    vgic_t *vgic = (typeof(vgic))(vm->arch.vgic_context);
    assert(vgic);
    return vgic;
}

static vgic_vcpu_t *get_vgic_vcpu(vgic_t *vgic, int vcpu_id)
{
    assert(vgic);
    assert((vcpu_id >= 0) && (vcpu_id < ARRAY_SIZE(vgic->vgic_vcpu)));
    return &(vgic->vgic_vcpu[vcpu_id]);
}

static bool gic_dist_is_enabled(vgic_t *vgic)
{
#if defined(GIC_V2)
    return (0 != vgic->dist.ctlr);
#elif defined(GIC_V3)
    /* We just care about group 1 non-secure interrupts beeing enabled. */
    return vgic->dist.ctlr & GIC_500_GRP1_NS;
#else
#error "set GIC_V2 or GIC_V3"
#endif
}

static void vgic_dist_set_ctlr(vgic_t *vgic, uint32_t data)
{
    switch (data) {
    case 0:
        DDIST("disabling gic distributor\n");
        vgic->dist.ctlr = 0;
        break;
#if defined(GIC_V2)
    case 1:
        DDIST("enabling gic distributor\n");
        vgic->dist.ctlr = 1;
        break;
#elif defined(GIC_V3)
    case (GIC_500_GRP0 | GIC_500_GRP1_NS | GIC_500_ARE_S):
        DDIST("enabling gic distributor (GRP0, GRP1_NS, ARE_S)\n");
        vgic->dist.ctlr = data;
    case (GIC_500_GRP1_NS | GIC_500_ARE_S):
        DDIST("enabling gic distributor  (GRP1_NS, ARE_S)\n");
        vgic->dist.ctlr = data;
    case GIC_500_GRP1_NS:
        DDIST("enabling gic distributor  (GRP1_NS)\n");
        vgic->dist.ctlr = data;
        break;
#else
#error "set GIC_V2 or GIC_V3"
#endif
    default:
        ZF_LOGE("Unknown dist ctlr encoding 0x%x", data);
        break;
    }
}

static struct virq_handle *virq_get_sgi_ppi(vgic_t *vgic, vm_vcpu_t *vcpu, int virq)
{
    vgic_vcpu_t *vgic_vcpu = get_vgic_vcpu(vgic, vcpu->vcpu_id);
    assert(vgic_vcpu);
    assert((virq >= 0) && (virq < ARRAY_SIZE(vgic_vcpu->local_virqs)));
    return vgic_vcpu->local_virqs[virq];
}

static struct virq_handle *virq_find_spi_irq_data(struct vgic *vgic, int virq)
{
    for (int i = 0; i < ARRAY_SIZE(vgic->vspis); i++) {
        if (vgic->vspis[i] && vgic->vspis[i]->virq == virq) {
            return vgic->vspis[i];
        }
    }
    return NULL;
}

static struct virq_handle *virq_find_irq_data(struct vgic *vgic, vm_vcpu_t *vcpu, int virq)
{
    if (virq < NUM_VCPU_LOCAL_VIRQS)  {
        return virq_get_sgi_ppi(vgic, vcpu, virq);
    }
    return virq_find_spi_irq_data(vgic, virq);
}

static int virq_spi_add(vgic_t *vgic, struct virq_handle *virq_data)
{
    for (int i = 0; i < ARRAY_SIZE(vgic->vspis); i++) {
        if (vgic->vspis[i] == NULL) {
            vgic->vspis[i] = virq_data;
            return 0;
        }
    }
    return -1;
}

static int virq_sgi_ppi_add(vm_vcpu_t *vcpu, vgic_t *vgic, struct virq_handle *virq_data)
{
    vgic_vcpu_t *vgic_vcpu = get_vgic_vcpu(vgic, vcpu->vcpu_id);
    assert(vgic_vcpu);
    int irq = virq_data->virq;
    assert((irq >= 0) && (irq < ARRAY_SIZE(vgic_vcpu->local_virqs)));
    virq_handle_t *slot = &vgic_vcpu->local_virqs[irq];
    if (*slot != NULL) {
        ZF_LOGE("IRQ %d already registered on VCPU %u", virq_data->virq, vcpu->vcpu_id);
        return -1;
    }
    *slot = virq_data;
    return 0;
}

static int virq_add(vm_vcpu_t *vcpu, vgic_t *vgic, struct virq_handle *virq_data)
{
    int virq = virq_data->virq;
    if (virq < NUM_VCPU_LOCAL_VIRQS) {
        return virq_sgi_ppi_add(vcpu, vgic, virq_data);
    }
    return virq_spi_add(vgic, virq_data);
}

static inline void virq_init(virq_handle_t virq, int irq, irq_ack_fn_t ack_fn, void *token)
{
    virq->virq = irq;
    virq->token = token;
    virq->ack = ack_fn;
}

static inline void set_sgi_ppi_pending(vgic_t *vgic, int irq, bool set_pending, int vcpu_id)
{
    if (set_pending) {
        vgic->dist.pending_set0[vcpu_id] |= IRQ_BIT(irq);
        vgic->dist.pending_clr0[vcpu_id] |= IRQ_BIT(irq);
    } else {
        vgic->dist.pending_set0[vcpu_id] &= ~IRQ_BIT(irq);
        vgic->dist.pending_clr0[vcpu_id] &= ~IRQ_BIT(irq);
    }
}

static inline void set_spi_pending(vgic_t *vgic, int irq, bool set_pending)
{
    if (set_pending) {
        vgic->dist.pending_set[IRQ_IDX(irq)] |= IRQ_BIT(irq);
        vgic->dist.pending_clr[IRQ_IDX(irq)] |= IRQ_BIT(irq);
    } else {
        vgic->dist.pending_set[IRQ_IDX(irq)] &= ~IRQ_BIT(irq);
        vgic->dist.pending_clr[IRQ_IDX(irq)] &= ~IRQ_BIT(irq);
    }
}

static inline void set_pending(vgic_t *vgic, int irq, bool set_pending, int vcpu_id)
{
    if (irq < NUM_VCPU_LOCAL_VIRQS) {
        set_sgi_ppi_pending(vgic, irq, set_pending, vcpu_id);
        return;
    }
    set_spi_pending(vgic, irq, set_pending);
}

static inline bool is_sgi_ppi_pending(vgic_t *vgic, int irq, int vcpu_id)
{
    return !!(vgic->dist.pending_set0[vcpu_id] & IRQ_BIT(irq));
}

static inline bool is_spi_pending(vgic_t *vgic, int irq)
{
    return !!(vgic->dist.pending_set[IRQ_IDX(irq)] & IRQ_BIT(irq));
}

static inline bool is_pending(vgic_t *vgic, int irq, int vcpu_id)
{
    if (irq < NUM_VCPU_LOCAL_VIRQS) {
        return is_sgi_ppi_pending(vgic, irq, vcpu_id);

    }
    return is_spi_pending(vgic, irq);
}

static inline void set_sgi_ppi_enable(vgic_t *vgic, int irq, bool set_enable, int vcpu_id)
{
    if (set_enable) {
        vgic->dist.enable_set0[vcpu_id] |= IRQ_BIT(irq);
        vgic->dist.enable_clr0[vcpu_id] |= IRQ_BIT(irq);
    } else {
        vgic->dist.enable_set0[vcpu_id] &= ~IRQ_BIT(irq);
        vgic->dist.enable_clr0[vcpu_id] &= ~IRQ_BIT(irq);
    }
}

static inline void set_spi_enable(vgic_t *vgic, int irq, bool set_enable)
{
    if (set_enable) {
        vgic->dist.enable_set[IRQ_IDX(irq)] |= IRQ_BIT(irq);
        vgic->dist.enable_clr[IRQ_IDX(irq)] |= IRQ_BIT(irq);
    } else {
        vgic->dist.enable_set[IRQ_IDX(irq)] &= ~IRQ_BIT(irq);
        vgic->dist.enable_clr[IRQ_IDX(irq)] &= ~IRQ_BIT(irq);
    }
}

static inline void set_enable(vgic_t *vgic, int irq, bool set_enable, int vcpu_id)
{
    if (irq < NUM_VCPU_LOCAL_VIRQS) {
        set_sgi_ppi_enable(vgic, irq, set_enable, vcpu_id);
        return;
    }
    set_spi_enable(vgic, irq, set_enable);
}

static inline bool is_sgi_ppi_enabled(vgic_t *vgic, int irq, int vcpu_id)
{
    return !!(vgic->dist.enable_set0[vcpu_id] & IRQ_BIT(irq));
}

static inline bool is_spi_enabled(vgic_t *vgic, int irq)
{
    return !!(vgic->dist.enable_set[IRQ_IDX(irq)] & IRQ_BIT(irq));
}

static inline bool is_enabled(vgic_t *vgic, int irq, int vcpu_id)
{
    if (irq < NUM_VCPU_LOCAL_VIRQS) {
        return is_sgi_ppi_enabled(vgic, irq, vcpu_id);
    }
    return is_spi_enabled(vgic, irq);
}

static inline bool is_sgi_ppi_active(vgic_t *vgic, int irq, int vcpu_id)
{
    return !!(vgic->dist.active0[vcpu_id] & IRQ_BIT(irq));
}

static inline bool is_spi_active(vgic_t *vgic, int irq)
{
    return !!(vgic->dist.active[IRQ_IDX(irq)] & IRQ_BIT(irq));
}

static inline bool is_active(vgic_t *vgic, int irq, int vcpu_id)
{
    if (irq < NUM_VCPU_LOCAL_VIRQS) {
        return is_sgi_ppi_active(vgic, irq, vcpu_id);
    }
    return is_spi_active(vgic, irq);
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

static int vgic_find_empty_list_reg(vgic_t *vgic, vm_vcpu_t *vcpu)
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

static int vgic_vcpu_load_list_reg(vgic_t *vgic, vm_vcpu_t *vcpu, int idx, struct virq_handle *irq)
{
    vgic_vcpu_t *vgic_vcpu = get_vgic_vcpu(vgic, vcpu->vcpu_id);
    assert(vgic_vcpu);
    assert((idx >= 0) && (idx < ARRAY_SIZE(vgic_vcpu->lr_shadow)));

    assert(idx <= (seL4_Uint8)(-1));
    /* ToDo: The group is hard-coded, because we deal with "Normal Word"
     *       interrupt in VMs only at the moment. The group could become a field
     *       in struct virq_handle or gets taken from the distributor fields
     *       irq_groupN.
     */
    seL4_Error err = seL4_ARM_VCPU_InjectIRQ(vcpu->vcpu.cptr, irq->virq, 0,
                                             ARM_INTR_GROUP, (seL4_Uint8)idx);
    if (seL4_NoError != err) {
        ZF_LOGE("Failure loading vGIC list register %d on vCPU %d, sel4 error %d",
                idx, vcpu->vcpu_id, err);
        /* Return a generic error, the caller doesn't understand seL4 errors. */
        return -1;
    }

    vgic_vcpu->lr_shadow[idx] = irq;

    return 0;
}

int handle_vgic_maintenance(vm_vcpu_t *vcpu, int idx)
{
    /* STATE d) */
    assert(vcpu);
    vgic_t *vgic = get_vgic_from_vm(vcpu->vm);
    assert(vgic);
    vgic_vcpu_t *vgic_vcpu = get_vgic_vcpu(vgic, vcpu->vcpu_id);
    assert(vgic_vcpu);
    assert((idx >= 0) && (idx < ARRAY_SIZE(vgic_vcpu->lr_shadow)));
    virq_handle_t *slot = &vgic_vcpu->lr_shadow[idx];
    assert(*slot);
    virq_handle_t lr_virq = *slot;
    *slot = NULL;
    /* Clear pending */
    DIRQ("Maintenance IRQ %d\n", lr_virq->virq);
    set_pending(vgic, lr_virq->virq, false, vcpu->vcpu_id);
    virq_ack(vcpu, lr_virq);

    /* Check the overflow list for pending IRQs */
    struct virq_handle *virq = vgic_irq_dequeue(vgic, vcpu);
    if (virq) {
        return vgic_vcpu_load_list_reg(vgic, vcpu, idx, virq);
    }

    return 0;
}

static void vgic_dist_enable_irq(vgic_t *vgic, vm_vcpu_t *vcpu, int irq)
{
    assert(vgic);
    DDIST("enabling irq %d\n", irq);
    set_enable(vgic, irq, true, vcpu->vcpu_id);
    struct virq_handle *virq_data = virq_find_irq_data(vgic, vcpu, irq);
    if (virq_data) {
        /* STATE b) */
        if (!is_pending(vgic, virq_data->virq, vcpu->vcpu_id)) {
            virq_ack(vcpu, virq_data);
        }
    } else {
        DDIST("enabled irq %d has no handle\n", irq);
    }
}

static void vgic_dist_disable_irq(vgic_t *vgic, vm_vcpu_t *vcpu, int irq)
{
    assert(vgic);

    /* STATE g)
     *
     * It is IMPLEMENTATION DEFINED if a GIC allows disabling SGIs. Our vGIC
     * implementation does not allows it, such requests are simply ignored.
     * Since it is not uncommon that a guest OS tries disabling SGIs, e.g. as
     * part of the platform initialization, no dedicated messages are logged
     * here to avoid bloating the logs.
     */
    if (irq >= NUM_SGI_VIRQS) {
        DDIST("disabling irq %d\n", irq);
        set_enable(vgic, irq, false, vcpu->vcpu_id);
    }
}

static int vgic_dist_set_pending_irq(vgic_t *vgic, vm_vcpu_t *vcpu, int irq)
{
    assert(vgic);

    /* STATE c) */

    struct virq_handle *virq_data = virq_find_irq_data(vgic, vcpu, irq);

    if (!virq_data || !gic_dist_is_enabled(vgic) || !is_enabled(vgic, irq, vcpu->vcpu_id)) {
        DDIST("IRQ not enabled (%d) on vcpu %d\n", irq, vcpu->vcpu_id);
        return -1;
    }

    if (is_pending(vgic, virq_data->virq, vcpu->vcpu_id)) {
        return 0;
    }

    DDIST("Pending set: Inject IRQ from pending set (%d)\n", irq);
    set_pending(vgic, virq_data->virq, true, vcpu->vcpu_id);

    /* Enqueueing an IRQ and dequeueing it right after makes little sense
     * now, but in the future this is needed to support IRQ priorities.
     */
    int err = vgic_irq_enqueue(vgic, vcpu, virq_data);
    if (err) {
        ZF_LOGF("Failure enqueueing IRQ, increase MAX_IRQ_QUEUE_LEN");
        return -1;
    }

    int idx = vgic_find_empty_list_reg(vgic, vcpu);
    if (idx < 0) {
        /* There were no empty list registers available, but that's not a big
         * deal -- we have already enqueued this IRQ and eventually the vGIC
         * maintenance code will load it to a list register from the queue.
         */
        return 0;
    }

    struct virq_handle *virq = vgic_irq_dequeue(vgic, vcpu);
    assert(virq);

    return vgic_vcpu_load_list_reg(vgic, vcpu, idx, virq);
}

static int vgic_dist_clr_pending_irq(vgic_t *vgic, vm_vcpu_t *vcpu, int irq)
{
    assert(vgic);

    DDIST("clr pending irq %d\n", irq);
    set_pending(vgic, irq, false, vcpu->vcpu_id);
    /* TODO: remove from IRQ queue and list registers as well */
    return 0;
}

static memory_fault_result_t vgic_dist_reg_read(vgic_t *vgic, vm_vcpu_t *vcpu,
                                                seL4_Word offset)
{
    int err = 0;
    fault_t *fault = vcpu->vcpu_arch.fault;
    typeof(&(vgic->dist)) gic_dist = &(vgic->dist);
    uint32_t reg = 0;
    int reg_offset = 0;
    uintptr_t base_reg;
    uint32_t *reg_ptr;
    switch (offset) {
    case RANGE32(GIC_DIST_CTLR, GIC_DIST_CTLR):
        reg = gic_dist->ctlr;
        break;
    case RANGE32(GIC_DIST_TYPER, GIC_DIST_TYPER):
        reg = gic_dist->typer;
        break;
    case RANGE32(GIC_DIST_IIDR, GIC_DIST_IIDR):
        reg = gic_dist->iidr;
        break;
    case RANGE32(0x00C, 0x01C):
        /* Reserved */
        break;
    case RANGE32(0x020, 0x03C):
        /* Implementation defined */
        break;
    case RANGE32(0x040, 0x07C):
        /* Reserved */
        break;
    case RANGE32(GIC_DIST_IGROUPR0, GIC_DIST_IGROUPR0):
        reg = gic_dist->irq_group0[vcpu->vcpu_id];
        break;
    case RANGE32(GIC_DIST_IGROUPR1, GIC_DIST_IGROUPRN):
        reg_offset = GIC_DIST_REGN(offset, GIC_DIST_IGROUPR1);
        reg = gic_dist->irq_group[reg_offset];
        break;
    case RANGE32(GIC_DIST_ISENABLER0, GIC_DIST_ISENABLER0):
        reg = gic_dist->enable_set0[vcpu->vcpu_id];
        break;
    case RANGE32(GIC_DIST_ISENABLER1, GIC_DIST_ISENABLERN):
        reg_offset = GIC_DIST_REGN(offset, GIC_DIST_ISENABLER1);
        reg = gic_dist->enable_set[reg_offset];
        break;
    case RANGE32(GIC_DIST_ICENABLER0, GIC_DIST_ICENABLER0):
        reg = gic_dist->enable_clr0[vcpu->vcpu_id];
        break;
    case RANGE32(GIC_DIST_ICENABLER1, GIC_DIST_ICENABLERN):
        reg_offset = GIC_DIST_REGN(offset, GIC_DIST_ICENABLER1);
        reg = gic_dist->enable_clr[reg_offset];
        break;
    case RANGE32(GIC_DIST_ISPENDR0, GIC_DIST_ISPENDR0):
        reg = gic_dist->pending_set0[vcpu->vcpu_id];
        break;
    case RANGE32(GIC_DIST_ISPENDR1, GIC_DIST_ISPENDRN):
        reg_offset = GIC_DIST_REGN(offset, GIC_DIST_ISPENDR1);
        reg = gic_dist->pending_set[reg_offset];
        break;
    case RANGE32(GIC_DIST_ICPENDR0, GIC_DIST_ICPENDR0):
        reg = gic_dist->pending_clr0[vcpu->vcpu_id];
        break;
    case RANGE32(GIC_DIST_ICPENDR1, GIC_DIST_ICPENDRN):
        reg_offset = GIC_DIST_REGN(offset, GIC_DIST_ICPENDR1);
        reg = gic_dist->pending_clr[reg_offset];
        break;
    case RANGE32(GIC_DIST_ISACTIVER0, GIC_DIST_ISACTIVER0):
        reg = gic_dist->active0[vcpu->vcpu_id];
        break;
    case RANGE32(GIC_DIST_ISACTIVER1, GIC_DIST_ISACTIVERN):
        reg_offset = GIC_DIST_REGN(offset, GIC_DIST_ISACTIVER1);
        reg = gic_dist->active[reg_offset];
        break;
    case RANGE32(GIC_DIST_ICACTIVER0, GIC_DIST_ICACTIVER0):
        reg = gic_dist->active_clr0[vcpu->vcpu_id];
        break;
    case RANGE32(GIC_DIST_ICACTIVER1, GIC_DIST_ICACTIVERN):
        reg_offset = GIC_DIST_REGN(offset, GIC_DIST_ICACTIVER1);
        reg = gic_dist->active_clr[reg_offset];
        break;
    case RANGE32(GIC_DIST_IPRIORITYR0, GIC_DIST_IPRIORITYR7):
        reg_offset = GIC_DIST_REGN(offset, GIC_DIST_IPRIORITYR0);
        reg = gic_dist->priority0[vcpu->vcpu_id][reg_offset];
        break;
    case RANGE32(GIC_DIST_IPRIORITYR8, GIC_DIST_IPRIORITYRN):
        reg_offset = GIC_DIST_REGN(offset, GIC_DIST_IPRIORITYR8);
        reg = gic_dist->priority[reg_offset];
        break;
    case RANGE32(0x7FC, 0x7FC):
        /* Reserved */
        break;
    case RANGE32(GIC_DIST_ITARGETSR0, GIC_DIST_ITARGETSR7):
        reg_offset = GIC_DIST_REGN(offset, GIC_DIST_ITARGETSR0);
        reg = gic_dist->targets0[vcpu->vcpu_id][reg_offset];
        break;
    case RANGE32(GIC_DIST_ITARGETSR8, GIC_DIST_ITARGETSRN):
        reg_offset = GIC_DIST_REGN(offset, GIC_DIST_ITARGETSR8);
        reg = gic_dist->targets[reg_offset];
        break;
    case RANGE32(0xBFC, 0xBFC):
        /* Reserved */
        break;
    case RANGE32(GIC_DIST_ICFGR0, GIC_DIST_ICFGRN):
        reg_offset = GIC_DIST_REGN(offset, GIC_DIST_ICFGR0);
        reg = gic_dist->config[reg_offset];
        break;
#ifdef GIC_V2
    case RANGE32(0xD00, 0xDE4):
        base_reg = (uintptr_t) & (gic_dist->spi[0]);
        reg_ptr = (uint32_t *)(base_reg + (offset - 0xD00));
        reg = *reg_ptr;
        break;
#endif
    case RANGE32(0xDE8, 0xEFC):
        /* Reserved [0xDE8 - 0xE00) */
        /* GIC_DIST_NSACR [0xE00 - 0xF00) - Not supported */
        break;
    case RANGE32(GIC_DIST_SGIR, GIC_DIST_SGIR):
        reg = gic_dist->sgir;
        break;
    case RANGE32(0xF04, 0xF0C):
        /* Implementation defined */
        break;
    case RANGE32(GIC_DIST_CPENDSGIR0, GIC_DIST_CPENDSGIRN):
        reg_offset = GIC_DIST_REGN(offset, GIC_DIST_CPENDSGIR0);
        reg = gic_dist->sgi_pending_clr[vcpu->vcpu_id][reg_offset];
        break;
    case RANGE32(GIC_DIST_SPENDSGIR0, GIC_DIST_SPENDSGIRN):
        reg_offset = GIC_DIST_REGN(offset, GIC_DIST_SPENDSGIR0);
        reg = gic_dist->sgi_pending_set[vcpu->vcpu_id][reg_offset];
        break;
    case RANGE32(0xF30, 0xFBC):
        /* Reserved */
        break;
#ifdef GIC_V2
    case RANGE32(0xFC0, 0xFFB):
        base_reg = (uintptr_t) & (gic_dist->periph_id[0]);
        reg_ptr = (uint32_t *)(base_reg + (offset - 0xFC0));
        reg = *reg_ptr;
        break;
#endif
#ifdef GIC_V3
    case RANGE32(0x6100, 0x7F00):
        base_reg = (uintptr_t) & (gic_dist->irouter[0]);
        reg_ptr = (uint32_t *)(base_reg + (offset - 0x6100));
        reg = *reg_ptr;
        break;

    case RANGE32(0xFFD0, 0xFFFC):
        base_reg = (uintptr_t) & (gic_dist->pidrn[0]);
        reg_ptr = (uint32_t *)(base_reg + (offset - 0xFFD0));
        reg = *reg_ptr;
        break;
#endif

    default:
        ZF_LOGE("Unknown register offset 0x%x", offset);
        err = ignore_fault(fault);
        goto fault_return;
    }
    uint32_t mask = fault_get_data_mask(fault);
    fault_set_data(fault, reg & mask);
    err = advance_fault(fault);

fault_return:
    if (err) {
        return FAULT_ERROR;
    }
    return FAULT_HANDLED;
}

static inline void emulate_reg_write_access(uint32_t *vreg, fault_t *fault)
{
    *vreg = fault_emulate(fault, *vreg);
}

static memory_fault_result_t vgic_dist_reg_write(vgic_t *vgic, vm_vcpu_t *vcpu,
                                                 seL4_Word offset)
{
    int err = 0;
    fault_t *fault = vcpu->vcpu_arch.fault;
    typeof(&(vgic->dist)) gic_dist = &(vgic->dist);
    uint32_t reg = 0;
    uint32_t mask = fault_get_data_mask(fault);
    uint32_t reg_offset = 0;
    uint32_t data;
    switch (offset) {
    case RANGE32(GIC_DIST_CTLR, GIC_DIST_CTLR):
        vgic_dist_set_ctlr(vgic, fault_get_data(fault));
        break;
    case RANGE32(GIC_DIST_TYPER, GIC_DIST_TYPER):
        break;
    case RANGE32(GIC_DIST_IIDR, GIC_DIST_IIDR):
        break;
    case RANGE32(0x00C, 0x01C):
        /* Reserved */
        break;
    case RANGE32(0x020, 0x03C):
        /* Implementation defined */
        break;
    case RANGE32(0x040, 0x07C):
        /* Reserved */
        break;
    case RANGE32(GIC_DIST_IGROUPR0, GIC_DIST_IGROUPR0):
        emulate_reg_write_access(&gic_dist->irq_group0[vcpu->vcpu_id], fault);
        break;
    case RANGE32(GIC_DIST_IGROUPR1, GIC_DIST_IGROUPRN):
        reg_offset = GIC_DIST_REGN(offset, GIC_DIST_IGROUPR1);
        emulate_reg_write_access(&gic_dist->irq_group[reg_offset], fault);
        break;
    case RANGE32(GIC_DIST_ISENABLER0, GIC_DIST_ISENABLERN):
        data = fault_get_data(fault);
        /* Mask the data to write */
        data &= mask;
        while (data) {
            int irq;
            irq = CTZ(data);
            data &= ~(1U << irq);
            irq += (offset - GIC_DIST_ISENABLER0) * 8;
            vgic_dist_enable_irq(vgic, vcpu, irq);
        }
        break;
    case RANGE32(GIC_DIST_ICENABLER0, GIC_DIST_ICENABLERN):
        data = fault_get_data(fault);
        /* Mask the data to write */
        data &= mask;
        while (data) {
            int irq;
            irq = CTZ(data);
            data &= ~(1U << irq);
            irq += (offset - GIC_DIST_ICENABLER0) * 8;
            vgic_dist_disable_irq(vgic, vcpu, irq);
        }
        break;
    case RANGE32(GIC_DIST_ISPENDR0, GIC_DIST_ISPENDRN):
        data = fault_get_data(fault);
        /* Mask the data to write */
        data &= mask;
        while (data) {
            int irq;
            irq = CTZ(data);
            data &= ~(1U << irq);
            irq += (offset - GIC_DIST_ISPENDR0) * 8;
            vgic_dist_set_pending_irq(vgic, vcpu, irq);
        }
        break;
    case RANGE32(GIC_DIST_ICPENDR0, GIC_DIST_ICPENDRN):
        data = fault_get_data(fault);
        /* Mask the data to write */
        data &= mask;
        while (data) {
            int irq;
            irq = CTZ(data);
            data &= ~(1U << irq);
            irq += (offset - GIC_DIST_ICPENDR0) * 8;
            vgic_dist_clr_pending_irq(vgic, vcpu, irq);
        }
        break;
    case RANGE32(GIC_DIST_ISACTIVER0, GIC_DIST_ISACTIVER0):
        emulate_reg_write_access(&gic_dist->active0[vcpu->vcpu_id], fault);
        break;
    case RANGE32(GIC_DIST_ISACTIVER1, GIC_DIST_ISACTIVERN):
        reg_offset = GIC_DIST_REGN(offset, GIC_DIST_ISACTIVER1);
        emulate_reg_write_access(&gic_dist->active[reg_offset], fault);
        break;
    case RANGE32(GIC_DIST_ICACTIVER0, GIC_DIST_ICACTIVER0):
        emulate_reg_write_access(&gic_dist->active_clr0[vcpu->vcpu_id], fault);
        break;
    case RANGE32(GIC_DIST_ICACTIVER1, GIC_DIST_ICACTIVERN):
        reg_offset = GIC_DIST_REGN(offset, GIC_DIST_ICACTIVER1);
        emulate_reg_write_access(&gic_dist->active_clr[reg_offset], fault);
        break;
    case RANGE32(GIC_DIST_IPRIORITYR0, GIC_DIST_IPRIORITYRN):
        break;
    case RANGE32(0x7FC, 0x7FC):
        /* Reserved */
        break;
    case RANGE32(GIC_DIST_ITARGETSR0, GIC_DIST_ITARGETSRN):
        break;
    case RANGE32(0xBFC, 0xBFC):
        /* Reserved */
        break;
    case RANGE32(GIC_DIST_ICFGR0, GIC_DIST_ICFGRN):
        /* Not supported */
        break;
    case RANGE32(0xD00, 0xDE4):
        break;
    case RANGE32(0xDE8, 0xEFC):
        /* Reserved [0xDE8 - 0xE00) */
        /* GIC_DIST_NSACR [0xE00 - 0xF00) - Not supported */
        break;
    case RANGE32(GIC_DIST_SGIR, GIC_DIST_SGIR):
        data = fault_get_data(fault);
        int mode = (data & GIC_DIST_SGI_TARGET_LIST_FILTER_MASK) >> GIC_DIST_SGI_TARGET_LIST_FILTER_SHIFT;
        int virq = (data & GIC_DIST_SGI_INTID_MASK);
        uint16_t target_list = 0;
        switch (mode) {
        case GIC_DIST_SGI_TARGET_LIST_SPEC:
            /* Forward virq to vcpus specified in CPUTargetList */
            target_list = (data & GIC_DIST_SGI_CPU_TARGET_LIST_MASK) >> GIC_DIST_SGI_CPU_TARGET_LIST_SHIFT;
            break;
        case GIC_DIST_SGI_TARGET_LIST_OTHERS:
            /* Forward virq to all vcpus but the requesting vcpu */
            target_list = (1 << vcpu->vm->num_vcpus) - 1;
            target_list = target_list & ~(1 << vcpu->vcpu_id);
            break;
        case GIC_DIST_SGI_TARGET_SELF:
            /* Forward to virq to only the requesting vcpu */
            target_list = (1 << vcpu->vcpu_id);
            break;
        default:
            ZF_LOGE("Unknow SGIR Target List Filter mode");
            goto ignore_fault;
        }
        for (int i = 0; i < vcpu->vm->num_vcpus; i++) {
            vm_vcpu_t *target_vcpu = vcpu->vm->vcpus[i];
            if (!(target_list & (1 << i)) || !is_vcpu_online(target_vcpu)) {
                continue;
            }
            vm_inject_irq(target_vcpu, virq);
        }
        break;
    case RANGE32(0xF04, 0xF0C):
        break;
    case RANGE32(GIC_DIST_CPENDSGIR0, GIC_DIST_SPENDSGIRN):
        assert(!"vgic SGI reg not implemented!\n");
        break;
    case RANGE32(0xF30, 0xFBC):
        /* Reserved */
        break;
    case RANGE32(0xFC0, 0xFFB):
        break;
#ifdef GIC_V3
    case RANGE32(0x6100, 0x7F00):
        data = fault_get_data(fault);
        ZF_LOGF_IF(data, "bad dist: 0x%x 0x%x", offset, data);
        break;
#endif
    default:
        ZF_LOGE("Unknown register offset 0x%x, value: 0x%x\n", offset, fault_get_data(fault));
    }
ignore_fault:
    err = ignore_fault(fault);
    if (err) {
        return FAULT_ERROR;
    }
    return FAULT_HANDLED;
}

static memory_fault_result_t handle_vgic_dist_fault(vm_t *vm, vm_vcpu_t *vcpu, uintptr_t fault_addr,
                                                    size_t fault_length,
                                                    void *cookie)
{
    /* There is a fault object per vcpu with much more context, the parameters
     * fault_addr and fault_length are no longer used.
     */
    fault_t *fault = vcpu->vcpu_arch.fault;
    assert(fault);
    assert(fault_addr == fault_get_address(vcpu->vcpu_arch.fault));

    assert(vm == vcpu->vm);

    assert(cookie);
    vgic_t *vgic = (typeof(vgic))cookie;

    seL4_Word addr = fault_get_address(fault);
    assert(addr >= vgic->mapped_dist.paddr);
    seL4_Word offset = addr - vgic->mapped_dist.paddr;
    assert(offset < vgic->mapped_dist.size);

    return fault_is_read(fault) ? vgic_dist_reg_read(vgic, vcpu, offset)
           : vgic_dist_reg_write(vgic, vcpu, offset);
}

#if defined(GIC_V3)

static memory_fault_result_t vgic_rdist_reg_read(vgic_t *vgic, vm_vcpu_t *vcpu,
                                                 seL4_Word offset)
{
    int err = 0;
    fault_t *fault = vcpu->vcpu_arch.fault;
    vgic_v3_rdist_t *gic_rdist = &(vgic->rdist);
    uint32_t reg = 0;
    int reg_offset = 0;
    uintptr_t base_reg;
    uint32_t *reg_ptr;
    switch (offset) {
    case RANGE32(GICR_CTLR, GICR_CTLR):
        reg = gic_rdist->ctlr;
        break;
    case RANGE32(GICR_IIDR, GICR_IIDR):
        reg = gic_rdist->iidr;
        break;
    case RANGE32(GICR_TYPER, GICR_TYPER):
        reg = gic_rdist->typer;
        break;
    case RANGE32(GICR_WAKER, GICR_WAKER):
        reg = gic_rdist->waker;
        break;
    case RANGE32(0xFFD0, 0xFFFC):
        base_reg = (uintptr_t) & (gic_rdist->pidr4);
        reg_ptr = (uint32_t *)(base_reg + (offset - 0xFFD0));
        reg = *reg_ptr;
        break;
    case RANGE32(GICR_IGROUPR0, GICR_IGROUPR0):
        base_reg = (uintptr_t) & (vgic->dist.irq_group0[vcpu->vcpu_id]);
        reg_ptr = (uint32_t *)(base_reg + (offset - GICR_IGROUPR0));
        reg = *reg_ptr;
        break;
    case RANGE32(GICR_ICFGR1, GICR_ICFGR1):
        base_reg = (uintptr_t) & (vgic->dist.config[1]);
        reg_ptr = (uint32_t *)(base_reg + (offset - GICR_ICFGR1));
        reg = *reg_ptr;
        break;
    default:
        ZF_LOGE("Unknown register offset 0x%x\n", offset);
        err = ignore_fault(fault);
        goto fault_return;
    }
    uint32_t mask = fault_get_data_mask(fault);
    fault_set_data(fault, reg & mask);
    err = advance_fault(fault);

fault_return:
    if (err) {
        return FAULT_ERROR;
    }
    return FAULT_HANDLED;
}


static memory_fault_result_t vgic_rdist_reg_write(vgic_t *vgic, vm_vcpu_t *vcpu,
                                                  seL4_Word offset)
{
    int err = 0;
    fault_t *fault = vcpu->vcpu_arch.fault;
    vgic_v3_dist_t *gic_dist = &(vgic->dist);
    vgic_v3_rdist_t *gic_rdist = &(vgic->rdist);
    uint32_t reg = 0;
    uint32_t mask = fault_get_data_mask(fault);
    uint32_t reg_offset = 0;
    uint32_t data;
    switch (offset) {
    case RANGE32(GICR_WAKER, GICR_WAKER):
        /* Writes are ignored */
        break;
    case RANGE32(GICR_IGROUPR0, GICR_IGROUPR0):
        emulate_reg_write_access(&gic_dist->irq_group0[vcpu->vcpu_id], fault);
        break;
    case RANGE32(GICR_ISENABLER0, GICR_ISENABLER0):
        data = fault_get_data(fault);
        /* Mask the data to write */
        data &= mask;
        while (data) {
            int irq;
            irq = CTZ(data);
            data &= ~(1U << irq);
            vgic_dist_enable_irq(vgic, vcpu, irq);
        }
        break;
    case RANGE32(GICR_ICENABLER0, GICR_ICENABLER0):
        data = fault_get_data(fault);
        /* Mask the data to write */
        data &= mask;
        while (data) {
            int irq;
            irq = CTZ(data);
            data &= ~(1U << irq);
            set_enable(vgic, irq, false, vcpu->vcpu_id);
        }
        break;
    case RANGE32(GICR_ICACTIVER0, GICR_ICACTIVER0):
        // TODO fix this
        emulate_reg_write_access(&gic_dist->active_clr0[vcpu->vcpu_id], fault);
        break;
    case RANGE32(GICR_IPRIORITYR0, GICR_IPRIORITYRN):
        break;
    default:
        ZF_LOGE("Unknown register offset 0x%x, value: 0x%x\n", offset, fault_get_data(fault));
    }
ignore_fault:
    err = ignore_fault(fault);
    if (err) {
        return FAULT_ERROR;
    }
    return FAULT_HANDLED;
}

static memory_fault_result_t handle_vgic_rdist_fault(vm_t *vm, vm_vcpu_t *vcpu, uintptr_t fault_addr,
                                                     size_t fault_length,
                                                     void *cookie)
{
    fault_t *fault = vcpu->vcpu_arch.fault;
    assert(fault);
    assert(fault_addr == fault_get_address(vcpu->vcpu_arch.fault));

    assert(vm == vcpu->vm);

    assert(cookie);
    vgic_t *vgic = (typeof(vgic))cookie;

    seL4_Word addr = fault_get_address(fault);
    assert(addr >= vgic->mapped_rdist.paddr);
    seL4_Word offset = addr - vgic->mapped_rdist.paddr;
    assert(offset < vgic->mapped_rdist.size);

    return fault_is_read(fault) ? vgic_rdist_reg_read(vgic, vcpu, offset)
           : vgic_rdist_reg_write(vgic, vcpu, offset);

}

#endif // GIC_V3

static void vgic_dist_reset(vgic_t *vgic)
{

#if defined(GIC_V2)

    vgic_v2_dist_t *gic_dist = &(vgic->dist);
    memset(gic_dist, 0, sizeof(*gic_dist));

    gic_dist->typer           = 0x0000fce7; /* RO */
    gic_dist->iidr            = 0x0200043b; /* RO */

    for (int i = 0; i < CONFIG_MAX_NUM_NODES; i++) {
        gic_dist->enable_set0[i]   = 0x0000ffff; /* 16bit RO */
        gic_dist->enable_clr0[i]   = 0x0000ffff; /* 16bit RO */
    }

    /* Reset value depends on GIC configuration */
    gic_dist->config[0]       = 0xaaaaaaaa; /* RO */
    gic_dist->config[1]       = 0x55540000;
    gic_dist->config[2]       = 0x55555555;
    gic_dist->config[3]       = 0x55555555;
    gic_dist->config[4]       = 0x55555555;
    gic_dist->config[5]       = 0x55555555;
    gic_dist->config[6]       = 0x55555555;
    gic_dist->config[7]       = 0x55555555;
    gic_dist->config[8]       = 0x55555555;
    gic_dist->config[9]       = 0x55555555;
    gic_dist->config[10]      = 0x55555555;
    gic_dist->config[11]      = 0x55555555;
    gic_dist->config[12]      = 0x55555555;
    gic_dist->config[13]      = 0x55555555;
    gic_dist->config[14]      = 0x55555555;
    gic_dist->config[15]      = 0x55555555;

    /* Configure per-processor SGI/PPI target registers */
    for (int i = 0; i < CONFIG_MAX_NUM_NODES; i++) {
        for (int j = 0; j < ARRAY_SIZE(gic_dist->targets0[i]); j++) {
            for (int irq = 0; irq < sizeof(uint32_t); irq++) {
                gic_dist->targets0[i][j] |= ((1 << i) << (irq * 8));
            }
        }
    }
    /* Deliver the SPI interrupts to the first CPU interface */
    for (int i = 0; i < ARRAY_SIZE(gic_dist->targets); i++) {
        gic_dist->targets[i] = 0x1010101;
    }

    /* identification */
    gic_dist->periph_id[4]    = 0x00000004; /* RO */
    gic_dist->periph_id[8]    = 0x00000090; /* RO */
    gic_dist->periph_id[9]    = 0x000000b4; /* RO */
    gic_dist->periph_id[10]   = 0x0000002b; /* RO */
    gic_dist->component_id[0] = 0x0000000d; /* RO */
    gic_dist->component_id[1] = 0x000000f0; /* RO */
    gic_dist->component_id[2] = 0x00000005; /* RO */
    gic_dist->component_id[3] = 0x000000b1; /* RO */

#elif defined(GIC_V3)

    vgic_v3_dist_t *gic_dist = &(vgic->dist);
    memset(gic_dist, 0, sizeof(*gic_dist));

    gic_dist->typer            = 0x7B04B0; /* RO */
    gic_dist->iidr             = 0x1043B ; /* RO */

    gic_dist->enable_set[0]    = 0x0000ffff; /* 16bit RO */
    gic_dist->enable_clr[0]    = 0x0000ffff; /* 16bit RO */

    gic_dist->config[0]        = 0xaaaaaaaa; /* RO */

    gic_dist->pidrn[0]         = 0x44;     /* RO */
    gic_dist->pidrn[4]         = 0x92;     /* RO */
    gic_dist->pidrn[5]         = 0xB4;     /* RO */
    gic_dist->pidrn[6]         = 0x3B;     /* RO */

    gic_dist->cidrn[0]         = 0x0D;     /* RO */
    gic_dist->cidrn[1]         = 0xF0;     /* RO */
    gic_dist->cidrn[2]         = 0x05;     /* RO */
    gic_dist->cidrn[3]         = 0xB1;     /* RO */

    vgic_v3_rdist_t *gic_rdist = &(vgic->rdist);
    memset(gic_rdist, 0, sizeof(*gic_rdist));

    gic_rdist->typer           = 0x11;     /* RO */
    gic_rdist->iidr            = 0x1143B;  /* RO */

    gic_rdist->pidr0           = 0x93;     /* RO */
    gic_rdist->pidr1           = 0xB4;     /* RO */
    gic_rdist->pidr2           = 0x3B;     /* RO */
    gic_rdist->pidr4           = 0x44;     /* RO */

    gic_rdist->cidr0           = 0x0D;     /* RO */
    gic_rdist->cidr1           = 0xF0;     /* RO */
    gic_rdist->cidr2           = 0x05;     /* RO */
    gic_rdist->cidr3           = 0xB1;     /* RO */
#else
#error "set GIC_V2 or GIC_V3"
#endif
}

int vm_register_irq(vm_vcpu_t *vcpu, int irq, irq_ack_fn_t ack_fn, void *cookie)
{
    assert(vcpu);
    vgic_t *vgic = get_vgic_from_vm(vcpu->vm);
    assert(vgic);

    struct virq_handle *virq_data = calloc(1, sizeof(*virq_data));
    if (!virq_data) {
        return -1;
    }

    virq_init(virq_data, irq, ack_fn, cookie);

    int err = virq_add(vcpu, vgic, virq_data);
    if (err) {
        free(virq_data);
        return -1;
    }

    return 0;
}

int vm_inject_irq(vm_vcpu_t *vcpu, int irq)
{
    // vm->lock();

    assert(vcpu);
    vgic_t *vgic = get_vgic_from_vm(vcpu->vm);
    assert(vgic);

    DIRQ("VM received IRQ %d\n", irq);

    int err = vgic_dist_set_pending_irq(vgic, vcpu, irq);

    if (!fault_handled(vcpu->vcpu_arch.fault) && fault_is_wfi(vcpu->vcpu_arch.fault)) {
        ignore_fault(vcpu->vcpu_arch.fault);
    }

    // vm->unlock();

    return err;
}

#if defined(GIC_V2)

static memory_fault_result_t handle_vgic_vcpu_fault(vm_t *vm, vm_vcpu_t *vcpu, uintptr_t fault_addr,
                                                    size_t fault_length,
                                                    void *cookie)
{
    /* We shouldn't fault on the vgic vcpu region as it should be mapped in
     * with all rights */
    return FAULT_ERROR;
}

static vm_frame_t vgic_vcpu_iterator(uintptr_t addr, void *cookie)
{
    cspacepath_t frame;
    vm_frame_t frame_result = { seL4_CapNull, seL4_NoRights, 0, 0 };
    vm_t *vm = (vm_t *)cookie;

    int err = vka_cspace_alloc_path(vm->vka, &frame);
    if (err) {
        ZF_LOGE("Failed to allocate cslot for vgic vcpu");
        return frame_result;
    }
    seL4_Word vka_cookie;
    err = vka_utspace_alloc_at(vm->vka, &frame, kobject_get_type(KOBJECT_FRAME, 12), 12, GIC_VCPU_PADDR, &vka_cookie);
    if (err) {
        err = simple_get_frame_cap(vm->simple, (void *)GIC_VCPU_PADDR, 12, &frame);
        if (err) {
            ZF_LOGE("Failed to find device cap for vgic vcpu");
            return frame_result;
        }
    }
    frame_result.cptr = frame.capPtr;
    frame_result.rights = seL4_AllRights;
    frame_result.vaddr = GIC_CPU_PADDR;
    frame_result.size_bits = seL4_PageBits;
    return frame_result;
}

#endif /* GIC_V2 */

/*
 * 1) completely virtual the distributor
 * 2) remap vcpu to cpu. Full access
 */
int vm_install_vgic(vm_t *vm)
{
    /* VM should not have a vgic already. */
    assert(!vm->arch.vgic_context);

    vgic_t *vgic = calloc(1, sizeof(*vgic));
    if (!vgic) {
        assert(!"Unable to calloc memory for VGIC");
        return -1;
    }
    /* vgic doesn't require much further initialization, having all fields set
     * to zero is fine.
     */
    vgic_dist_reset(vgic);

    vgic->mapped_dist.paddr = GIC_DIST_PADDR;
    vgic->mapped_dist.size = PAGE_SIZE_4K;
    vgic->mapped_dist.vm_res = vm_reserve_memory_at(vm,
                                                    vgic->mapped_dist.paddr,
                                                    vgic->mapped_dist.size,
                                                    handle_vgic_dist_fault,
                                                    (void *)vgic);

#if defined(GIC_V2)

    /* Remap VCPU to CPU */
    vgic->mapped_cpu_if.paddr = GIC_CPU_PADDR;
    vgic->mapped_cpu_if.size = PAGE_SIZE_4K;
    vgic->mapped_cpu_if.vm_res = vm_reserve_memory_at(vm,
                                                      vgic->mapped_cpu_if.paddr,
                                                      vgic->mapped_cpu_if.size,
                                                      handle_vgic_vcpu_fault,
                                                      NULL);
    int err = vm_map_reservation(vm, vgic->mapped_cpu_if.vm_res,
                                 vgic_vcpu_iterator, (void *)vm);
    if (err) {
        free(vgic);
        return -1;
    }

#elif defined(GIC_V3)

    vgic->mapped_rdist.paddr = GIC_REDIST_PADDR;
    vgic->mapped_rdist.size = PAGE_SIZE_4K;
    vgic->mapped_rdist.vm_res = vm_reserve_memory_at(vm,
                                                     vgic->mapped_rdist.paddr,
                                                     vgic->mapped_rdist.size,
                                                     handle_vgic_rdist_fault,
                                                     (void *)vgic);

#else
#error "set GIC_V2 or GIC_V3"
#endif

    vm->arch.vgic_context = vgic;

    return 0;
}

int vm_vgic_maintenance_handler(vm_vcpu_t *vcpu)
{
    int idx = seL4_GetMR(seL4_VGICMaintenance_IDX);
    /* Currently not handling spurious IRQs */
    assert(idx >= 0);

    int err = handle_vgic_maintenance(vcpu, idx);
    if (!err) {
        seL4_MessageInfo_t reply;
        reply = seL4_MessageInfo_new(0, 0, 0, 0);
        seL4_Reply(reply);
    } else {
        ZF_LOGF("vGIC maintenance handler failed (error %d)", err);
    }
    return VM_EXIT_HANDLED;
}
