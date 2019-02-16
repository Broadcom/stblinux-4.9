/*
 * Copyright (C) 2015 Broadcom Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#define BCM2835_PLLA			0	/* plla */
#define BCM2835_PLLB			1	/* pllb */
#define BCM2835_PLLC			2	/* pllc */
#define BCM2835_PLLD			3	/* plld */
#define BCM2835_PLLH			4	/* pllh */
						/* */
#define BCM2835_PLLA_CORE		5	/* plla_core */
#define BCM2835_PLLA_PER		6	/* plla_per */
#define BCM2835_PLLB_ARM		7	/* pllb_arm */
#define BCM2835_PLLC_CORE0		8	/* pllc_core0 */
#define BCM2835_PLLC_CORE1		9	/* pllc_core1 */
#define BCM2835_PLLC_CORE2		10	/* pllc_core2 */
#define BCM2835_PLLC_PER		11	/* pllc_per */
#define BCM2835_PLLD_CORE		12	/* plld_core */
#define BCM2835_PLLD_PER		13	/* plld_per */
#define BCM2835_PLLH_RCAL		14	/* pllh_rcal_prediv */
#define BCM2835_PLLH_AUX		15	/* pllh_aux */
#define BCM2835_PLLH_PIX		16	/* pllh_pix_prediv */

#define BCM2835_CLOCK_TIMER		17	/* timer */
#define BCM2835_CLOCK_OTP		18	/* otp */
#define BCM2835_CLOCK_UART		19	/* uart */
#define BCM2835_CLOCK_VPU		20	/* vpu */
#define BCM2835_CLOCK_V3D		21	/* v3d */
#define BCM2835_CLOCK_ISP		22	/* isp */
#define BCM2835_CLOCK_H264		23	/* h264 */
#define BCM2835_CLOCK_VEC		24	/* vec */
#define BCM2835_CLOCK_HSM		25	/* hsm */
#define BCM2835_CLOCK_SDRAM		26	/* sdram */
#define BCM2835_CLOCK_TSENS		27	/* tsens */
#define BCM2835_CLOCK_EMMC		28	/* emmc */
#define BCM2835_CLOCK_PERI_IMAGE	29	/* peri_image */
#define BCM2835_CLOCK_PWM		30	/* pwm */
#define BCM2835_CLOCK_PCM		31	/* pcm */

#define BCM2835_PLLA_DSI0		32	/* plla_dsi0 */
#define BCM2835_PLLA_CCP2		33	/* plla_ccp2 */
#define BCM2835_PLLD_DSI0		34	/* plld_dsi0 */
#define BCM2835_PLLD_DSI1		35	/* plld_dsi1 */

#define BCM2835_CLOCK_AVEO		36	/* aveo */
#define BCM2835_CLOCK_DFT		37	/* dft */
#define BCM2835_CLOCK_GP0		38	/* gp0 */
#define BCM2835_CLOCK_GP1		39	/* gp1 */
#define BCM2835_CLOCK_GP2		40	/* gp2 */
#define BCM2835_CLOCK_SLIM		41	/* slim */
#define BCM2835_CLOCK_SMI		42	/* smi */
#define BCM2835_CLOCK_TEC		43	/* tec */
#define BCM2835_CLOCK_DPI		44	/* dpi */
#define BCM2835_CLOCK_CAM0		45	/* cam0 */
#define BCM2835_CLOCK_CAM1		46	/* cam1 */
#define BCM2835_CLOCK_DSI0E		47	/* dsi0e */
#define BCM2835_CLOCK_DSI1E		48	/* dsi1e */

#define BCM2835_PLLA_MOR_CORE2		49	/* plla_mor_core2 */
#define BCM2835_PLLA_MOR_CORE3		50	/* plla_mor_core3 */
#define BCM2835_PLLC_MOR_CORE3		51	/* pllc_mor_core3 */
#define BCM2835_CLOCK_ARGON		52	/* argon */
#define BCM2835_CLOCK_EMMC2		53	/* emmc2 */
#define BCM2835_CLOCK_GISB		54	/* gisb */
#define BCM2835_CLOCK_ALTSCB		55	/* altscb */
#define BCM2835_CLOCK_GENET_250		56	/* genet_250 */
#define BCM2835_CLOCK_STB27		57	/* stb27 */
#define BCM2835_CLOCK_STB54		58	/* stb54 */
#define BCM2835_CLOCK_STB108		59	/* stb108 */
#define BCM2835_CLOCK_PIXEL_BVB		60	/* pixel_bvb */
#define BCM2835_CLOCK_GENET_125		61	/* genet_125 */
#define BCM2835_STBGENCTRL_HVDCPUALT	62	/* hvdcpualt */
#define BCM2835_STBGENCTRL_HVDCOREALT	63	/* hvdcorealt */
#define BCM2835_CLOCK_M2MC		64	/* m2mc */
#define BCM2835_CLOCK_XPT		65	/* xpt */
#define BCM2835_CLOCK_USBXHCI		66	/* usbxhci */
#define BCM2835_CLOCK_MAX		67	/* max */
