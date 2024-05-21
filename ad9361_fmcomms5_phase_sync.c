/*
 * Copyright (C) 2017 Analog Devices, Inc.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 */

#include "ad9361.h"

#include <errno.h>
#include <iio.h>
#include <stdio.h>
#include <gpiod.h>
#ifdef _WIN32
#include <windows.h>
#else
#include <unistd.h>
#include <time.h>
#endif
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#ifdef _MSC_BUILD
#define snprintf sprintf_s
#endif

// Device names
#define DEV_RX_NAME "cf-ad9361-A"
#define DEV_RX_SLAVE_NAME "cf-ad9361-B"
#define DEV_TX_NAME "cf-ad9361-dds-core-A"
#define DEV_TX_SLAVE_NAME "cf-ad9361-dds-core-B"
#define DEV_PHY_NAME "ad9361-phy"
#define DEV_PHY_SLAVE_NAME "ad9361-phy-B"

#define DDS_SCALE 0.2
#define SAMPLES 1024  // Was 8192
#define TOLERANCE 0.01 // Degrees
#define CALIBRATE_TRIES 10 // Was 30
#define STEP_SIZE 0.5
#define M_2PI 2 * M_PI
#define STALE_BUFFERS 5 // Was 20
#define CALIBRATE_TX 0
#define PAUSE 0

#define GPIOCHIP "1"
#define CONSUMER "PHASE_SYNC"
#define CALIBRATION_LINE 4
#define TX_LINE 6
#define RX_LINE 5

// DEBUG = 0 Off
// DEBUG = 1 Verbose
// DEBUG = 2 More verbose
#define DEBUG 1

// Set to 1 if external splitter is connected to all receive chains,
// this does not calibrate the transmitters.  This is useful if you want to
// calibrate from a the SMA connectors
#define EXTERNAL_REFERENCE_TONE 0

#define CHECK(expr)                                                            \
  if (expr < 0) {                                                              \
    return expr;                                                               \
  }

static struct iio_device *dev_phy, *dev_phy_slave;
static struct iio_device *dev_rx, *dev_rx_slave;
static struct iio_device *dev_tx, *dev_tx_slave;
static struct iio_channel *dds_out[2][8];
static struct iio_buffer *rxbuf;
static struct iio_channel *rxa_chan_real, *rxa_chan_imag;
static struct iio_channel *rxb_chan_real, *rxb_chan_imag;

static void ad9361_sleep_ms(void)
{
#ifdef _WIN32
    Sleep(1); /* milliseconds */
#else
    struct timespec time;

    time.tv_sec = 0;
    time.tv_nsec = 1000 * 1000;
    nanosleep(&time, NULL);
#endif
}

double scale_phase_0_360(double val)
{
    if (val >= 360.0)
        val -= 360.0;

    if (val < 0)
        val += 360.0;

    return val;
}

void dds_tx_phase_rotation(struct iio_device *dev, double val)
{
    long long i, q;
    int d, j;

    if (dev == dev_tx_slave)
        d = 1;
    else
        d = 0;

    i = scale_phase_0_360(val + 90.0) * 1000;
    q = scale_phase_0_360(val) * 1000;

    for (j = 0; j < 8; j++) {
        switch (j) {
        case 0:
        case 1:
        case 4:
        case 5:
            iio_channel_attr_write_longlong(dds_out[d][j], "phase", i);
            break;
        default:
            iio_channel_attr_write_longlong(dds_out[d][j], "phase", q);
        }
    }
}

double calculate_phase(int16_t *a, int16_t *b, int16_t *c, int16_t *d,
                       int samples)
{
    // Fast phase estimation with complex signals handling wrapped phase
    int k = 0;
    double real = 0, imag = 0;
    for (; k < samples; k++) {
        real += ((double)a[k] * (double)c[k] + (double)b[k] * (double)d[k]);
        imag += ((double)a[k] * (double)d[k] - (double)b[k] * (double)c[k]);
    }
    return atan2(imag, real);
}

double calculate_power(int16_t *a, int16_t *b, int samples) {
    double avg_power_ref = 0;
    double avg_power_targ = 0;
    for (int k = 0; k < samples; k++) {
	avg_power_ref += (double)a[k]*(double)a[k] + (double)b[k]*(double)b[k];
    }
    return avg_power_ref/((double) samples);
}

void near_end_loopback_ctrl(unsigned channel, bool enable)
{
    unsigned tmp;
    struct iio_device *dev = (channel > 3) ? dev_rx : dev_rx_slave;
    if (!dev)
        return;

    if (channel > 3)
        channel -= 4;

    if (iio_device_reg_read(dev, 0x80000418 + channel * 0x40, &tmp))
        return;

    if (enable)
        tmp |= 0x1;
    else
        tmp &= ~0xF;

    iio_device_reg_write(dev, 0x80000418 + channel * 0x40, tmp);
}

void configure_ports(unsigned val)
{
    unsigned lp_slave, lp_master, sw;
    char *rx_port, *tx_port;

    // https://wiki.analog.com/resources/eval/user-guides/ad-fmcomms5-ebz/multi-chip-sync#rf_phase_difference

    /*
     *  0 DISABLE: Use RF ports
     *  1 TX2A_A -> RX2A_A : BIST_LOOPBACK on B
     *  2 TX2A_B -> RX2A_A : BIST_LOOPBACK on B
     *  3 TX2A_A -> RX2A_B : BIST_LOOPBACK on A
     *  4 TX2A_B -> RX2A_B : BIST_LOOPBACK on A
     *
     */
    switch (val) {
    default:
    case 0:
        lp_slave = 0;
        lp_master = 0;
        sw = 0;
        tx_port = "A";
        rx_port = "A_BALANCED";
        gpiod_ctxless_set_value(
            GPIOCHIP,
            CALIBRATION_LINE,
            0,
            false,
            CONSUMER,
            NULL,
            NULL);
        break;
    case 1:
    case 2:
        lp_slave = 1;
        lp_master = 0;
        sw = val - 1;
        tx_port = "A";
        rx_port = "A_BALANCED";
        gpiod_ctxless_set_value(
            GPIOCHIP,
            CALIBRATION_LINE,
            1,
            false,
            CONSUMER,
            NULL,
            NULL);
        break;
    case 3:
    case 4:
        lp_slave = 0;
        lp_master = 1;
        sw = val - 1;
        tx_port = "A";
        rx_port = "A_BALANCED";
        gpiod_ctxless_set_value(
            GPIOCHIP,
            CALIBRATION_LINE,
            1,
            false,
            CONSUMER,
            NULL,
            NULL);
        break;
    }

    // Set up ports for FPGA BIST Loopback
    near_end_loopback_ctrl(2, lp_slave);  /* HPC */
    near_end_loopback_ctrl(3, lp_slave);  /* HPC */
    near_end_loopback_ctrl(6, lp_master); /* LPC */
    near_end_loopback_ctrl(7, lp_master); /* LPC */

    // Configure Loopback switches
    gpiod_ctxless_set_value(
        GPIOCHIP,
        TX_LINE,
        sw & 0x01,
        false,
        CONSUMER,
        NULL,
        NULL);

    gpiod_ctxless_set_value(
        GPIOCHIP,
        RX_LINE,
        (sw & 0x02)>>1,
        false,
        CONSUMER,
        NULL,
        NULL);

    // Map ports to switch orientation
    iio_channel_attr_write(iio_device_find_channel(dev_phy, "voltage1", false),
                           "rf_port_select", rx_port);
    iio_channel_attr_write(iio_device_find_channel(dev_phy, "voltage1", true),
                           "rf_port_select", tx_port);
    iio_channel_attr_write(
        iio_device_find_channel(dev_phy_slave, "voltage1", false),
        "rf_port_select", rx_port);
    iio_channel_attr_write(
        iio_device_find_channel(dev_phy_slave, "voltage1", true),
        "rf_port_select", tx_port);
}

int trx_phase_rotation(struct iio_device *dev, double val)
{
    struct iio_channel *out0, *out1;
    double phase, vcos, vsin;
    unsigned offset;
    int ret;

    bool output = (dev == dev_tx_slave) || (dev == dev_tx);

    phase = val * 2 * M_PI / 360.0;

    vcos = cos(phase);
    vsin = sin(phase);

    if (output) {
        double corr;
        corr = 1.0 /
               fmax(fabs(sin(phase) + cos(phase)), fabs(cos(phase) - sin(phase)));
        vcos *= corr;
        vsin *= corr;
    }

    /* Set both RX1 and RX2 */
    for (offset = 0; offset <= 2; offset += 2) {
        if (offset == 2) {
            out0 = iio_device_find_channel(dev, "voltage2", output);
            out1 = iio_device_find_channel(dev, "voltage3", output);
        } else {
            out0 = iio_device_find_channel(dev, "voltage0", output);
            out1 = iio_device_find_channel(dev, "voltage1", output);
        }
        if ((out0 == NULL) || (out1 == NULL))
            return -ENODEV;

        if (out1 && out0) {
            ret = iio_channel_attr_write_double(out0, "calibscale", (double)vcos);
            CHECK(ret);
            ret = iio_channel_attr_write_double(out0, "calibphase", (double)(-1.0 * vsin));
            CHECK(ret);
            ret = iio_channel_attr_write_double(out1, "calibscale", (double)vcos);
            CHECK(ret);
            ret = iio_channel_attr_write_double(out1, "calibphase", (double)vsin);
            CHECK(ret);
        }
    }
    return 0;
}

int streaming_interfaces(bool enable)
{
    if (enable) {
        rxa_chan_real = iio_device_find_channel(dev_rx, "voltage2", false);
        rxa_chan_imag = iio_device_find_channel(dev_rx, "voltage3", false);
        rxb_chan_real = iio_device_find_channel(dev_rx, "voltage6", false);
        rxb_chan_imag = iio_device_find_channel(dev_rx, "voltage7", false);
        if (!(rxa_chan_real && rxa_chan_imag && rxb_chan_real && rxb_chan_imag))
            streaming_interfaces(false);

        iio_channel_enable(rxa_chan_real);
        iio_channel_enable(rxa_chan_imag);
        iio_channel_enable(rxb_chan_real);
        iio_channel_enable(rxb_chan_imag);
        rxbuf = iio_device_create_buffer(dev_rx, SAMPLES, false);
        if (!rxbuf)
            streaming_interfaces(false);
    } else {
        if (rxbuf) {
            iio_buffer_destroy(rxbuf);
        }
        if (rxa_chan_real) {
            iio_channel_disable(rxa_chan_real);
        }
        if (rxa_chan_imag) {
            iio_channel_disable(rxa_chan_imag);
        }
        if (rxb_chan_real) {
            iio_channel_disable(rxb_chan_real);
        }
        if (rxb_chan_imag) {
            iio_channel_disable(rxb_chan_imag);
        }
        return -1;
    }
    return 0;
}

void read_buffer_data(struct iio_channel *chn, struct iio_buffer *buf,
                      void *dst, size_t len)
{
    uintptr_t src_ptr, dst_ptr = (uintptr_t)dst, end = dst_ptr + len;
    unsigned int bytes = iio_channel_get_data_format(chn)->length / 8;
    uintptr_t buf_end = (uintptr_t)iio_buffer_end(buf);
    ptrdiff_t buf_step = iio_buffer_step(buf);

    for (src_ptr = (uintptr_t)iio_buffer_first(buf, chn);
         src_ptr < buf_end && dst_ptr + bytes <= end;
         src_ptr += buf_step, dst_ptr += bytes)
        iio_channel_convert(chn, (void *)dst_ptr, (const void *)src_ptr);
}

double estimate_phase_diff(double *estimate, double *ref_pwr, double *target_pwr)
{
    ssize_t nbytes_rx = iio_buffer_refill(rxbuf);
    if (nbytes_rx <= 0)
        return nbytes_rx;

    int16_t myData0_i[SAMPLES], myData0_q[SAMPLES];
    int16_t myData2_i[SAMPLES], myData2_q[SAMPLES];

    // Read data from all channels
    read_buffer_data(rxa_chan_real, rxbuf, myData0_i, SAMPLES * sizeof(int16_t));
    read_buffer_data(rxa_chan_imag, rxbuf, myData0_q, SAMPLES * sizeof(int16_t));
    read_buffer_data(rxb_chan_real, rxbuf, myData2_i, SAMPLES * sizeof(int16_t));
    read_buffer_data(rxb_chan_imag, rxbuf, myData2_q, SAMPLES * sizeof(int16_t));

    ad9361_sleep_ms();

    *estimate =
        calculate_phase(myData0_i, myData0_q, myData2_i, myData2_q, SAMPLES) *
        180 / M_PI;
#if (DEBUG > 1)
    *ref_pwr = calculate_power(myData0_i, myData0_q, SAMPLES);
    *target_pwr = calculate_power(myData2_i, myData2_q, SAMPLES);
#endif

    return 0;
}

int calibrate_chain(struct iio_device *dev, double scale, double *phase)
{
    double est = 0, tmp;
    double ref_pwr = 0;
    double target_pwr = 0;
    int k = 0, ret = -2, g;
    double old_phase;
    double old_est = 10000;
    double step_mult = 1.0;

    if (streaming_interfaces(true) < 0)
        return -ENODEV;

    *phase = 0;

    for (; k < CALIBRATE_TRIES; k++) {

        if(k != 0) {
            if(k==1) {
                old_phase = *phase;
                *phase = est*-1;
                old_est = est;
                ret = trx_phase_rotation(dev, *phase);
                CHECK(ret);
            } else {
                old_phase = *phase;
                *phase = STEP_SIZE * step_mult * est * -1 + (*phase);
                old_est = est;
                ret = trx_phase_rotation(dev, *phase);
                CHECK(ret);
            }
        }

        for (g=0; g<STALE_BUFFERS; g++) {
            ret = estimate_phase_diff(&est, &ref_pwr, &target_pwr);
            CHECK(ret);
        }

        if (k==CALIBRATE_TRIES-1) {
            if (fabs(old_est) > fabs(est)){
                old_phase = *phase;
                *phase = STEP_SIZE * step_mult * est + (*phase);
                old_est = est;
                ret = trx_phase_rotation(dev, *phase);
                CHECK(ret);
            } else {
                *phase = old_phase;
                step_mult /= 2.0;
            }
        }

#if (PAUSE == 1)
        printf("  Phase check - %d\n", k);
        streaming_interfaces(false);
        getchar();
        streaming_interfaces(true);
#endif

#if (DEBUG > 1)
        printf("Phase error: %f | Phase Setting: %f\n", est, *phase);
        printf("Ref Power    : %f dB\n", 10*log10(ref_pwr));
        printf("Target Power : %f dB\n", 10*log10(target_pwr));
#endif
        if (fabs(est) < TOLERANCE) {
            ret = 0;
            break;
        } else if (k == CALIBRATE_TRIES - 1) {
#if (DEBUG > 1)
            printf("Calibration Failed! (Did not achieve error tolerance)\n");
#endif
	    }

        est *= scale;
    }

    streaming_interfaces(false);

#if (DEBUG > 0)
    printf("Remaining Phase error: %f\n", est);
    printf("Rotation: %f\n", *phase);
#endif

    return 0;
}

int quad_tracking(bool enable)
{
    struct iio_channel *chn =
        iio_device_find_channel(dev_phy, "voltage1", enable);
    if (chn == NULL)
        return -ENODEV;
    iio_channel_attr_write(chn, "quadrature_tracking_en", "0");
    chn = iio_device_find_channel(dev_phy_slave, "voltage1", enable);
    if (chn == NULL)
        return -ENODEV;
    iio_channel_attr_write(chn, "quadrature_tracking_en", "0");
    return 0;
}

int configure_transceiver(struct iio_device *dev, long long bw_hz,
                          long long fs_hz, long long lo_hz)
{
    double rx_gain=0;
    int ret = 0;
    // Set up channels
    struct iio_channel *chnRX1;
    struct iio_channel *chnTX1;
    struct iio_channel *chnRX2;
    struct iio_channel *chnTX2;
    // Configure LO channel
    chnRX1 = iio_device_find_channel(dev, "altvoltage0", true);
    chnTX1 = iio_device_find_channel(dev, "altvoltage1", true);
    if (!(chnRX1 && chnTX1))
        return -ENODEV;
    ret = iio_channel_attr_write_longlong(chnRX1, "frequency", lo_hz);
    CHECK(ret);
    ret = iio_channel_attr_write_longlong(chnTX1, "frequency", lo_hz);
    CHECK(ret);
    // Set up gains to know good values
    chnRX1 = iio_device_find_channel(dev, "voltage0", false);
    chnTX1 = iio_device_find_channel(dev, "voltage0", true);
    chnRX2 = iio_device_find_channel(dev, "voltage1", false);
    chnTX2 = iio_device_find_channel(dev, "voltage1", true);
    if (!(chnRX1 && chnTX1 && chnRX2 && chnTX2))
        return -ENODEV;
    ret = iio_channel_attr_write(chnRX1, "gain_control_mode", "manual");
    CHECK(ret);
    ret = iio_channel_attr_write(chnRX2, "gain_control_mode", "manual");
    CHECK(ret);
    ret = iio_channel_attr_read_double(chnRX1, "hardwaregain", &rx_gain);
    CHECK(ret);
    ret = iio_channel_attr_write_double(chnTX1, "hardwaregain", (rx_gain > 59) ? -89.75 : -30-rx_gain);
    CHECK(ret);
    ret = iio_channel_attr_write_double(chnTX2, "hardwaregain", (rx_gain > 59) ? -89.75 : -30-rx_gain);
    CHECK(ret);

    return 0;
}

int configure_dds(double fs, double scale)
{
    long long freq = (long long)fs * 0.01;
    int i, j, ret = 0;

    for (i = 0; i < 2; i++) {
        for (j = 0; j < 8; j++) {
            ret |= iio_channel_attr_write_longlong(dds_out[i][j], "frequency", freq);
            ret |= iio_channel_attr_write_double(dds_out[i][j], "scale", scale);
        }

        dds_tx_phase_rotation(i ? dev_tx_slave : dev_tx, 0.0);
        trx_phase_rotation(i ? dev_tx_slave : dev_tx, 0.0);
    }
    return ret;
}

int get_dds_channels()
{
    struct iio_device *dev;
    int i, j;
    char name[16];

    for (i = 0; i < 2; i++) {
        dev = i ? dev_tx : dev_tx_slave;

        for (j = 0; j < 8; j++) {
            snprintf(name, sizeof(name), "altvoltage%d", j);

            dds_out[i][j] = iio_device_find_channel(dev, name, true);
            if (!dds_out[i][j])
                return -errno;
        }
    }
    return 0;
}

int setup_iio_devices(struct iio_context *ctx)
{
    dev_rx = iio_context_find_device(ctx, DEV_RX_NAME);
    dev_rx_slave = iio_context_find_device(ctx, DEV_RX_SLAVE_NAME);
    dev_phy = iio_context_find_device(ctx, DEV_PHY_NAME);
    dev_phy_slave = iio_context_find_device(ctx, DEV_PHY_SLAVE_NAME);
    dev_tx = iio_context_find_device(ctx, DEV_TX_NAME);
    dev_tx_slave = iio_context_find_device(ctx, DEV_TX_SLAVE_NAME);
    return (dev_rx && dev_rx_slave && dev_phy && dev_phy_slave && dev_tx &&
            dev_tx_slave);
}

/* Synchronize all transmit and receive channels for FMComms5*/
int phase_sync(struct iio_context *ctx, long long sample_rate, long long lo)
{
    // Set analog bandwidth same as sample rate
    long long bw = sample_rate;
    struct iio_channel *chnTX1;
    struct iio_channel *chnTX2;
    struct iio_channel *chnTX1_slave;
    struct iio_channel *chnTX2_slave;

    int ret;

    // Set up devices
#if (PAUSE == 1)
    printf("Setting up IIO Devices\n");
    getchar();
#endif
    if (!setup_iio_devices(ctx))
        return -ENODEV;

    chnTX1 = iio_device_find_channel(dev_phy, "voltage0", true);
    chnTX2 = iio_device_find_channel(dev_phy, "voltage1", true);
    chnTX1_slave = iio_device_find_channel(dev_phy_slave, "voltage0", true);
    chnTX2_slave = iio_device_find_channel(dev_phy_slave, "voltage1", true);

    // Set up DDSs
#if (PAUSE == 1)
    printf("Setting up DDS\n");
    getchar();
#endif
    ret = get_dds_channels();
    CHECK(ret);

    // Sync chips together
#if (PAUSE == 1)
    printf("Calling MCS\n");
    getchar();
#endif
    ret = ad9361_multichip_sync(dev_phy, &dev_phy_slave, 1, CHECK_SAMPLE_RATES);
    CHECK(ret);

    // Set up DDS at given frequency
#if (PAUSE == 1)
    printf("Configuring DDS\n");
    getchar();
#endif
    ret = configure_dds(sample_rate, DDS_SCALE);
    CHECK(ret);

    // Set LO, bandwidth, and gain of transceivers
#if (PAUSE == 1)
    printf("Configuring Master\n");
    getchar();
#endif
    ret = configure_transceiver(dev_phy, bw, sample_rate, lo);
    CHECK(ret);
#if (PAUSE == 1)
    printf("Configuring Slave\n");
    getchar();
#endif
    ret = configure_transceiver(dev_phy_slave, bw, sample_rate, lo);
    CHECK(ret);

    // Turn off quad tracking
#if (PAUSE == 1)
    printf("Turning off quad tracking\n");
    getchar();
#endif
    quad_tracking(false);

    // Reset all phase shifts to zero
#if (PAUSE == 1)
    printf("Reset phase rx master\n");
    getchar();
#endif
    ret = trx_phase_rotation(dev_rx, 0.0);
    CHECK(ret);
#if (PAUSE == 1)
    printf("Reset phase rx slave\n");
    getchar();
#endif
    ret = trx_phase_rotation(dev_rx_slave, 0.0);
    CHECK(ret);
#if (PAUSE == 1)
    printf("Reset phase tx master\n");
    getchar();
#endif
    ret = trx_phase_rotation(dev_tx, 0.0);
    CHECK(ret);
#if (PAUSE == 1)
    printf("Reset phase tx slave\n");
    getchar();
#endif
    ret = trx_phase_rotation(dev_tx_slave, 0.0);
    CHECK(ret);

    // Align receiver on Chip A (TX from chip A) with BIST loopback
#if (PAUSE == 1)
    printf("Setup A->A. Loopback on B\n");
    getchar();
#endif
    configure_ports(1); // A->A Loopback B
    double phase_est_rx_slave = 0, phase_est = 0;
#if (PAUSE == 1)
    printf("Calibrate\n");
    getchar();
#endif
    ret = calibrate_chain(dev_rx_slave, 1, &phase_est_rx_slave);
    CHECK(ret);

    // Align receiver on Chip B (TX from chip A) with BIST loopback
#if (PAUSE == 1)
    printf("Reset Slave rotation\n");
    getchar();
#endif
    ret = trx_phase_rotation(dev_rx_slave, 0.0); // Reset reference channel
    CHECK(ret);

#if (PAUSE == 1)
    printf("Setup A->B. Loopback on A\n");
    getchar();
#endif
    configure_ports(3); // A->B Loopback A // Chip A -> Chip A | FPGA Loopback on B was 3 (Moved to 1)
#if (PAUSE == 1)
    printf("Calibrate\n");
    getchar();
#endif
    ret = calibrate_chain(dev_rx, -1, &phase_est);
    CHECK(ret);

    // At this point both receivers are aligned with Chip A TX
    // Align Chip B TX with a receiver
#if (CALIBRATE_TX == 1)
    ret = trx_phase_rotation(dev_rx_slave, 0);
    CHECK(ret);
    configure_ports(4); // B->B Loopback A
    ret = calibrate_chain(dev_tx_slave, -1, &phase_est);
    CHECK(ret);
#endif

    // Set rotation of chip B receiver to originally measured
#if (PAUSE == 1)
    printf("Put slave channel back at proper rotation\n");
    getchar();
#endif
    ret = trx_phase_rotation(dev_rx_slave, phase_est_rx_slave);
    CHECK(ret);

    if (!(chnTX1 && chnTX2))
        return -ENODEV;
#if (PAUSE == 1)
    printf("Lower hardware gains\n");
    getchar();
#endif
    ret = iio_channel_attr_write_double(chnTX1, "hardwaregain", -89.75);
    CHECK(ret);
    ret = iio_channel_attr_write_double(chnTX2, "hardwaregain", -89.75);
    CHECK(ret);
    ret = iio_channel_attr_write_double(chnTX1_slave, "hardwaregain", -89.75);
    CHECK(ret);
    ret = iio_channel_attr_write_double(chnTX2_slave, "hardwaregain", -89.75);
    CHECK(ret);

    return 0;
}

/* Synchronize all transmit and receive channels for FMComms5*/
int ad9361_fmcomms5_phase_sync(struct iio_context *ctx, long long lo)
{
    struct iio_channel *chan;
    struct iio_device *dev;
    long long sample_rate;
    int ret;

    // Get current sample rate
    dev = iio_context_find_device(ctx, DEV_PHY_NAME);
    if (dev == NULL)
        return -ENODEV;
    chan = iio_device_find_channel(dev, "voltage1", true);
    if (chan == NULL)
        return -ENODEV;
    ret = iio_channel_attr_read_longlong(chan, "sampling_frequency", &sample_rate);
    CHECK(ret);

    ret = phase_sync(ctx, sample_rate, lo);

    if (ret < 0)
        streaming_interfaces(false);

    // Reset ports out to RF
    configure_ports(0);

    return ret;
}
