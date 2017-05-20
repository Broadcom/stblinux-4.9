#ifndef _CPUFREQ_CPUID_H
#define _CPUFREQ_CPUID_H

static inline void __cpuid(unsigned int *eax, unsigned int *ebx,
				unsigned int *ecx, unsigned int *edx)
{
	/* ecx is often an input as well as an output. */
	asm volatile(
#if defined(__i386__) && defined(__PIC__)
	    "push %%ebx\n"
	    "cpuid\n"
	    "movl %%ebx, %1\n"
	    "pop %%ebx\n"
#else
	    "cpuid\n"
#endif
	    : "=a" (*eax),
#if defined(__i386__) && defined(__PIC__)
	      "=r" (*ebx),
#else
	      "=b" (*ebx),
#endif
	      "=c" (*ecx),
	      "=d" (*edx)
	    : "0" (*eax), "2" (*ecx));
}
static inline void cpuid(unsigned int op,
			 unsigned int *eax, unsigned int *ebx,
			 unsigned int *ecx, unsigned int *edx)
{
	*eax = op;
	*ecx = 0;
	__cpuid(eax, ebx, ecx, edx);
}

/*
 * CPUID functions returning a single datum
 */
static inline unsigned int cpuid_eax(unsigned int op)
{
	unsigned int eax, ebx, ecx, edx;

	cpuid(op, &eax, &ebx, &ecx, &edx);

	return eax;
}

static inline unsigned int cpuid_ebx(unsigned int op)
{
	unsigned int eax, ebx, ecx, edx;

	cpuid(op, &eax, &ebx, &ecx, &edx);

	return ebx;
}

static inline unsigned int cpuid_ecx(unsigned int op)
{
	unsigned int eax, ebx, ecx, edx;

	cpuid(op, &eax, &ebx, &ecx, &edx);

	return ecx;
}

static inline unsigned int cpuid_edx(unsigned int op)
{
	unsigned int eax, ebx, ecx, edx;

	cpuid(op, &eax, &ebx, &ecx, &edx);

	return edx;
}

#endif /* _CPUFREQ_CPUID_H */
