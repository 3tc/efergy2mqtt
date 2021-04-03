// EFERGY ENERGY SENSOR RTL-SDR DECODER via rtl_fm
//
// Compile:
//  gcc -O3 -lm -o EfergyDecode EfergyRPI_log.c
//
// Run:
//  rtl_fm -f 433.25M -s 200000 -r 96000 -A fast -g 19.7 | ./EfergyRPI_log
//
//------------------------------------------------------------------------------
// Originally Authored by Nathaniel Elijah
//
// (Formerly named EfergyRPI_001.c )
// Copyright 2013 Nathaniel Elijah
// Permission is hereby granted to use this Software for any purpose
// including combining with commercial products, creating derivative
// works, and redistribution of source or binary code, without
// limitation or consideration. Any redistributed copies of this
// Software must include the above Copyright Notice.
// THIS SOFTWARE IS PROVIDED "AS IS". THE AUTHOR OF THIS CODE MAKES NO
// WARRANTIES REGARDING THIS SOFTWARE, EXPRESS OR IMPLIED, AS TO ITS
// SUITABILITY OR FITNESS FOR A PARTICULAR PURPOSE.
//------------------------------------------------------------------------------
// Changes by Gough (me@goughlui.com)
//
// Suggested compilation command is:
// gcc -O3 -o EfergyRPI_001 EfergyRPI_001.c -lm
// Note: -lm coming last avoids error from compiler which complains of
// undefined reference to function `pow'. The addition of -O3 turns on
// compiler optimizations which may or may not improve performance.
//
// Logging filename defined by additional command line argument.
//
// Added logging to file by append, with adjustable "samples to flush".
// This can avoid SSD wear while also avoiding too much lost data in case
// of power loss or crash.
//
// Also have ability to change line endings to DOS \r\n format.
//
// Consider changing the Voltage to match your local line voltage for
// best results.
//
// Poor signals seem to cause rtl_fm and this code to consume CPU and slow
// down. Best results for me seem to be with the E4000 tuner. My R820T
// requires higher gain but sees some decode issues. Sometimes it stops
// altogether, but that's because rtl_fm seems to starve in providing
// samples.
//
// Should build and run on anything where rtl-fm works. You'll need to grab
// build, and install rtl-sdr from http://sdr.osmocom.org/trac/wiki/rtl-sdr
// first.
//
// Execute similarly
// rtl_fm -f 433550000 -s 200000 -r 96000 -g 19.7 2>/dev/null | ./EfergyRPI_log
// but play with the value for gain (in this case, 19.7) to achieve best result.
// -----------------------------------------------------------------------------
// Changes by github user magellannh
//
// 08-14-2014 - Changed logic to support different Efergy sensor types without 
// changing settings logic uses message size to determine if checksum or crc 
// should be used. Also, changed code so instead of doing bit by bit processing 
// when samples arrive, a frame's worth of samples is saved after a valid 
// preamble and then processed. One advantage of this approach is that the 
// center is recalculated for each frame for more reliable decoding, especially 
// if there's lots of 433Mhz noise, which for me was throwing off the old center 
// calculation (noise from weather sensors). This is less efficient, but still 
// uses < 7% of cpu on raspberry pi (plus another 25% for rtl_fm w/ -A fast 
// option)
// 08-13-2014 - Added code to check the CRC-CCIT (Xmodem)  crc used by Elite 3.0
// TPM
// 08/12/2014 - Some debugging and sample analysis code added
//
// Bug Fix - Changed frame bytearray to unsigned char and added cast on  byte 
// used in pow() function to explicitly make it signed char.  Default 
// signed/unsigned for char is compiler dependent and this resulted in various 
// problems.
//
// New Feature  - Added frame analysis feature that dumps debug information to 
// help characterize the FSK sample data received by rtl_fm.  The feature is 
// invoked using a "-d" option followed by an optional debug level (1..4). The 
// output is sent to stdout.
//
// Usage Examples: (original examples above still work as before plus these new 
// options)
//
//  rtl_fm -f 433.51e6 -s 200000 -r 96000 -A fast  | ./EfergyRPI_log -d 1
//      This mode shows the least information which is just the best guess at 
//      the decoded frame and a KW calculation using bytes 4, 5, and 6.  The 
//      checksum is computed and displayed, but not validated.
//  rtl_fm -f 433.51e6 -s 200000 -r 96000 -A fast  | ./EfergyRPI_log -d 2
//      This  level shows average plus and minus sample values and centering 
//      which can help with finding the best frequency. Adjust frequency to get 
//      wave center close to  0 .  If center is too high, lower frequency, 
//      otherwise increase it.
//  rtl_fm -f 433.51e6 -s 200000 -r 96000 -A fast  | ./EfergyRPI_log -d 3
//      This mode outputs a summary with counts of consecutive positive or 
//      negative samples.  These consecutive pulse counts are what the main code 
//      uses to decode 0 and 1 data bits.  See comments below for more details 
//      on expected pulse counts.
//  rtl_fm -f 433.51e6 -s 200000 -r 96000 -A fast  | ./EfergyRPI_log -d 4
//      This mode shows everything in modes 1..3 plus a raw dump of the sample 
//      data received from rtl_fm.
//
//  *Notice the "-A fast" option on  rtl_fm.  This cut Raspberry Pi cpu load 
//  from 50% to 25% and decode still worked fine. Also, with an R820T USB 
//  dongle, leaving  rtl_fm gain in 'auto' mode  produced the best results.
// -----------------------------------------------------------------------------
// Changes by github user guiguan (root@guiguan.net)
//
// 18/09/2015 - Added TARGET_UID setting to only capture frames with the
// targeted unique identifier and drop the rest. This is necessary when your 
// neighbours are using the same kind of sensors.
// Changed the unit from kW to W in debugging output. Also, the threshold from 
// 10 to 10000, which means 10kW.
// Included unistd.h for sleep in linux environment
// Slightly formatted the source code.

#include <stdio.h>
#include <stdint.h>
#include <time.h>
#include <math.h>
#include <stdlib.h> // For exit function
#include <string.h>
#include <unistd.h>

#define VOLTAGE         240 /* For non-TPM type sensors, set to line voltage, 
such as 240; For Efergy Elite 3.0 TPM,  set to 1 */
#define TARGET_UID     -1 /* The 3rd byte of the frame received is used to
uniquely identify the transmitter. If set, only frames match this UID will be
captured; otherwise, set to -1 to disable this feature */

#define FRAMEBYTECOUNT          9  /* Attempt to decode up to this many bytes.   */
#define MINLOWBITS              3  /* Min number of positive samples for a logic 0 */
#define MINHIGHBITS             9  /* Min number of positive samples for a logic 1 */
#define MIN_POSITIVE_PREAMBLE_SAMPLES       40 /* Number of positive samples in  an efergy  preamble */
#define MIN_NEGATIVE_PREAMBLE_SAMPLES       40 /* Number of negative samples for a valid preamble  */
#define EXPECTED_BYTECOUNT_IF_CHECKSUM_USED 8
#define EXPECTED_BYTECOUNT_IF_CRC_USED      9

int analysis_wavecenter;

#define LOGTYPE         1   // Allows changing line-endings - 0 is for Unix /n, 1 for Windows /r/n
#define SAMPLES_TO_FLUSH    10  // Number of samples taken before writing to file.
// Setting this too low will cause excessive wear to flash due to updates to
// filesystem! You have been warned! Set to 10 samples for 6 seconds = every min.
int loggingok;   // Global var indicating logging on or off
int samplecount; // Global var counter for samples taken since last flush
FILE *fp;    // Global var file handle

// Instead of processing frames bit by bit as samples arrive from rtl_fm, all samples are stored once a preamble is detected until
// enough samples have been saved to cover the expected maximum frame size.  This maximum number of samples needed for a frame
// is an estimate with padding which will hopefully be enough to store a full frame with some extra.
//
// It seems that with most Efergy formats, each data bit is encoded  using some combination of about 18-20 rtl_fm samples.
// zero bits are usually received as 10-13 negative samples followed by 4-7 positive samples, while 1 bits
// come in as 4-7 negative samples followed by 10-13 positive samples ( these #s may have wide tolerences)
// If the signal has excessive noise, it's theoretically possible to fill up this storage and still have more frame data coming in.
// The code handles this overflow by trunkating the frame, but when this happens, it usually means the data is  junk anyway.
//
// To skip over noise frames, the code checks for a sequence with both a positive and negative preamble back to back (can be pos-neg or neg-pos)
//  From empirical testing, the preamble is usually about 180 negative samples followed by 45-50 positive samples.
//
// At some frequencies the sign of the sample data becomes perfectly inverted.  When this happens,  frame data can still be decoded by keying off
// negative pulse sequences instead of positive pulse sequences.  This inversion condition can be detected by looking at the sign of the first samples
// received after the preamble.  If the first set of samples is negative, the data can get decoded from the positive sample pulses.  If the first set of
//samples after the preamble is  greater than 0 then the data probably can be found by decoding negative samples using the same rules that are usually
// used when decoding with the positive samples.  The analysis code automatically checks for this 'inverted' signal condition and decodes from either
// the positive or negative pulse stream depending on that check.

#define APPROX_SAMPLES_PER_BIT  19
#define FRAMEBITCOUNT           (FRAMEBYTECOUNT*8)  /* bits for entire frame (not including preamble) */
#define SAMPLE_STORE_SIZE       (FRAMEBITCOUNT*APPROX_SAMPLES_PER_BIT)

int sample_storage[SAMPLE_STORE_SIZE];
int sample_store_index;

int decode_bytes_from_pulse_counts(int pulse_store[], int pulse_store_index, unsigned char bytes[]) {
    int i;
    int dbit = 0;
    int bitpos = 0;
    unsigned char bytedata = 0;
    int bytecount = 0;

    for (i = 0; i < FRAMEBYTECOUNT; i++)
        bytes[i] = 0;
    for (i = 0; i < pulse_store_index; i++) {
        if (pulse_store[i] > MINLOWBITS) {
            dbit++;
            bitpos++;
            bytedata = bytedata << 1;
            if (pulse_store[i] > MINHIGHBITS)
                bytedata = bytedata | 0x1;
            if (bitpos > 7) {
                bytes[bytecount] = bytedata;
                bytedata = 0;
                bitpos = 0;
                bytecount++;
                if (bytecount == FRAMEBYTECOUNT) {
                    return bytecount;
                }
            }
        }
    }
    return bytecount;
}

unsigned char compute_checksum(unsigned char bytes[], int bytecount) {
    // Calculate simple 1 byte checksum on message bytes
    unsigned char tbyte = 0x00;
    int i;
    for (i = 0; i < (bytecount - 1); i++) {
        tbyte += bytes[i];
    }
    return tbyte;
}

uint16_t compute_crc(unsigned char bytes[], int bytecount) {
    // Calculate  CRC-CCIT (Xmodem)  crc using 0x1021 polynomial
    uint16_t crc = 0;
    int i;
    for (i = 0; i < bytecount - 2; i++) {
        crc = crc ^ ((uint16_t) bytes[i] << 8);
        int j;
        for (j = 0; j < 8; j++) {
            if (crc & 0x8000)
                crc = (crc << 1) ^ 0x1021;
            else
                crc <<= 1;
        }
    }
    return crc;
}

int calculate_wave_center(int *avg_positive_sample, int *avg_negative_sample) {
    int i;
    int64_t avg_neg = 0;
    int64_t avg_pos = 0;
    int pos_count = 0;
    int neg_count = 0;
    for (i = 0; i < sample_store_index; i++)
        if (sample_storage[i] >= 0) {
            avg_pos += sample_storage[i];
            pos_count++;
        } else {
            avg_neg += sample_storage[i];
            neg_count++;
        }
    if (pos_count != 0)
        avg_pos /= pos_count;
    if (neg_count != 0)
        avg_neg /= neg_count;
    *avg_positive_sample = avg_pos;
    *avg_negative_sample = avg_neg;
    int diff = (avg_neg + ((avg_pos - avg_neg) / 2));
    return diff;
}

int generate_pulse_count_array(int display_pulse_details, int pulse_count_storage[]) {

    // From empirical analysis with an Elite 3.0 TPM transmitter, sometimes the data can be decoded by counting negative pulses rather than positive
    // pulses.  It seems that if the first sequence after the preamble is positive pulses, the data can be decoded by parsing using the negative pulse counts.
    // This flag decoder automatically detect this.
    int store_positive_pulses = (sample_storage[2] < analysis_wavecenter);

    if (display_pulse_details) printf("\nPulse stream for this frame (P-Consecutive samples > center, N-Consecutive samples < center)\n");

    int wrap_count = 0;
    int pulse_count = 0;
    int space_count = 0;
    int pulse_store_index = 0;
    int space_store_index = 0;
    int display_pulse_info = 1;
    int i;
    for (i = 0; i < sample_store_index; i++) {
        int samplec = sample_storage[i] - analysis_wavecenter;
        if (samplec < 0) {
            if (pulse_count > 0) {
                if (store_positive_pulses)
                    pulse_count_storage[pulse_store_index++] = pulse_count;
                if (display_pulse_details) printf("%2dP ", pulse_count);
                wrap_count++;
            }
            pulse_count = 0;
            space_count++;
        } else {
            if (space_count > 0) {
                if (store_positive_pulses == 0)
                    pulse_count_storage[pulse_store_index++] = space_count;
                if (display_pulse_details) printf("%2dN ", space_count);
                wrap_count++;
            }
            space_count = 0;
            pulse_count++;
        }
        if (wrap_count >= 16) {
            if (display_pulse_details) printf("\n");
            wrap_count = 0;
        }
    }
    if (display_pulse_details) printf("\n\n");

    return pulse_store_index;
}

void display_frame_data(int debug_level, char *msg, unsigned char bytes[], int bytecount) {

#if TARGET_UID > 0
    // only receive frames that has the targeted UID
    if (bytecount < 3 || bytes[2] != TARGET_UID) {
        return;
    }
#endif

    time_t ltime;
    char buffer[80];
    time( &ltime );
    struct tm *curtime = localtime( &ltime );
    strftime(buffer, 80, "%x,%X", curtime);

    // Some magic here to figure out whether the message has a 1 byte checksum or 2 byte crc
    char *data_ok_str = (char *) 0;
    unsigned char checksum = 0;
    uint16_t crc = 0;

    if (bytecount == EXPECTED_BYTECOUNT_IF_CHECKSUM_USED) {
        checksum = compute_checksum(bytes, bytecount);
        if (checksum == bytes[bytecount - 1])
            data_ok_str = "chksum ok";
    } else if (bytecount == EXPECTED_BYTECOUNT_IF_CRC_USED) {
        crc = compute_crc(bytes, bytecount);
        if (crc == ((bytes[bytecount - 2] << 8) | bytes[bytecount - 1]))
            data_ok_str = "crc ok";
    }

    double current_adc = (bytes[4] * 256) + bytes[5];
    double result  = (VOLTAGE * current_adc) / ((double) (32768) / (double) pow(2, (signed char) bytes[6]));
    if (debug_level > 0) {
        if (debug_level == 1)
            printf("%s  %s ", buffer, msg);
        else
            printf("%s ", msg);

        int i;
        for (i = 0; i < bytecount; i++)
            printf("%02x ", bytes[i]);

        if (data_ok_str != (char *) 0)
            printf(data_ok_str);
        else {
            checksum = compute_checksum(bytes, bytecount);
            crc = compute_crc(bytes, bytecount);
            printf(" cksum: %02x crc16: %04x ", checksum, crc);
        }
        if (result < 10000)
            printf("  W: %4.3f\n", result);
        else {
            printf("  W: <out of range>\n");
            if (data_ok_str != (char *) 0)
                printf("*For Efergy True Power Moniter (TPM), set VOLTAGE=1 before compiling\n");
        }
    } else if (data_ok_str != (char *) 0) {
        printf("%s,%f\n", buffer, result);
        if (loggingok) {
            if (LOGTYPE) {
                fprintf(fp, "%s,%f\r\n", buffer, result);
            } else {
                fprintf(fp, "%s,%f\n", buffer, result);
            }
            samplecount++;
            if (samplecount == SAMPLES_TO_FLUSH) {
                samplecount = 0;
                fflush(fp);
            }
        }
        fflush(stdout);
    } else
        printf("Checksum/CEC Error.  Enable debug output with -d option\n");
}

void analyze_efergy_message(int debug_level) {

    // See how balanced/centered the sample data is.  Best case is  diff close to 0
    int avg_pos, avg_neg;
    int difference = calculate_wave_center(&avg_pos, &avg_neg);

    if (debug_level > 1) {
        time_t ltime;
        char buffer[80];
        time( &ltime );
        struct tm *curtime = localtime( &ltime );
        strftime(buffer, 80, "%x,%X", curtime);
        printf("\nAnalysis of rtl_fm sample data for frame received on %s\n", buffer);
        printf("     Number of Samples: %6d\n", sample_store_index);
        printf("    Avg. Sample Values: %6d (negative)   %6d (positive)\n", avg_neg, avg_pos);
        printf("           Wave Center: %6d (this frame) %6d (last frame)\n", difference, analysis_wavecenter);
    }
    analysis_wavecenter = difference; // Use the calculated wave center from this sample to process next frame

    if (debug_level == 4) { // Raw Sample Dump only in highest debug level
        int wrap_count = 0;
        printf("\nShowing raw rtl_fm sample data received between start of frame and end of frame\n");
        int i;
        for (i = 0; i < sample_store_index; i++) {
            printf("%6d ", sample_storage[i] - analysis_wavecenter);
            wrap_count++;
            if (wrap_count >= 16) {
                printf("\n");
                wrap_count = 0;
            }
        }
        printf("\n\n");
    }

    int display_pulse_details = (debug_level >= 3 ? 1 : 0);
    int pulse_count_storage[SAMPLE_STORE_SIZE];
    int pulse_store_index = generate_pulse_count_array(display_pulse_details, pulse_count_storage);
    unsigned char bytearray[FRAMEBYTECOUNT];
    int bytecount = decode_bytes_from_pulse_counts(pulse_count_storage, pulse_store_index, bytearray);
    char *frame_msg;
    if (sample_storage[2] < analysis_wavecenter)
        frame_msg = "Msg:";
    else
        frame_msg = "Msg (from negative pulses):";
    display_frame_data(debug_level, frame_msg, bytearray, bytecount);

    if (debug_level > 1) printf("\n");
}

void  main (int argc, char**argv)
{
    int debug_level = 0;

    // Give rtl_fm program some time to get initialized so its startup messages don't interleave with ours
    sleep(1);

    if ((argc == 2) && (strncmp(argv[1], "-h", 2) == 0)) {
        printf("\nUsage: %s              - Normal mode\n", argv[0]);
        printf("       %s <filename>   - Normal mode plus log samples to output file\n", argv[0]);
        printf("       %s -d [1,2,3,4] - Set debug/verbosity.  Default level 0 has minimum output\n", argv[0]);
        exit(0);
    } else if ((argc == 3) && (strncmp(argv[1], "-d", 2) == 0)) {
        debug_level = strtol(argv[2], NULL, 0);
        if ((debug_level < 1) || (debug_level > 4)) {
            fprintf(stderr, "\nDebug level (-d option) must be between 1 and 4\n");
            exit(EXIT_FAILURE);
        }
    } else if ((argc == 2) && (strcmp(argv[1], "-d") == 0))
        debug_level = 3;
    else if (argc == 2) {
        fp = fopen(argv[1], "a"); // Log file opened in append mode to avoid destroying data
        samplecount = 0; // Reset sample counter
        loggingok = 1;
        if (fp == NULL) {
            fprintf(stderr, "\nFailed to open log file!\n");
            exit(EXIT_FAILURE);
        }
    } else {
        loggingok = 0;
    }

    if (debug_level > 0)
        printf("\nEfergy Power Monitor Decoder - (debug level %d)\n\n", debug_level);
    else
        printf("\nEfergy Energy Monitor Decoder\n\n");

    int prvsamp;
    analysis_wavecenter = 0;

    while ( !feof(stdin) ) {

        // Look for a valid Efergy Preamble sequence which we'll define as
        // a sequence of at least MIN_PEAMBLE_SIZE positive and negative or negative and positive pulses. eg 50N+50P or 50P+50N
        int negative_preamble_count = 0;
        int positive_preamble_count = 0;
        prvsamp = 0;
        while ( !feof(stdin) ) {
            int cursamp  = (int16_t) (fgetc(stdin) | fgetc(stdin) << 8);
            // Check for preamble
            if ((prvsamp >= analysis_wavecenter) && (cursamp >= analysis_wavecenter)) {
                positive_preamble_count++;
            } else if ((prvsamp < analysis_wavecenter) && (cursamp < analysis_wavecenter)) {
                negative_preamble_count++;
            } else if ((prvsamp >= analysis_wavecenter) && (cursamp < analysis_wavecenter)) {
                if ((positive_preamble_count > MIN_POSITIVE_PREAMBLE_SAMPLES) &&
                        (negative_preamble_count > MIN_NEGATIVE_PREAMBLE_SAMPLES))
                    break;
                negative_preamble_count = 0;
            } else if ((prvsamp < analysis_wavecenter) && (cursamp >= analysis_wavecenter)) {
                if ((positive_preamble_count > MIN_POSITIVE_PREAMBLE_SAMPLES) &&
                        (negative_preamble_count > MIN_NEGATIVE_PREAMBLE_SAMPLES))
                    break;
                positive_preamble_count = 0;
            }
            prvsamp = cursamp;
        } // end of find preamble while loop

        sample_store_index = 0;
        while ( !feof(stdin) ) {
            int cursamp  = (int16_t) (fgetc(stdin) | fgetc(stdin) << 8);
            sample_storage[sample_store_index] = cursamp;
            if (sample_store_index < (SAMPLE_STORE_SIZE - 1))
                sample_store_index++;
            else {
                analyze_efergy_message(debug_level);
                break;
            }
        } // Frame processing while
    } // outermost while

    if (loggingok) {
        fclose(fp); // If rtl-fm gives EOF and program terminates, close file gracefully.
    }
}