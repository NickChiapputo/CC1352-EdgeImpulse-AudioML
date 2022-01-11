/*
 * Copyright (c) 2015-2020, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/* Include ----------------------------------------------------------------- */
#include "ei_run_classifier.h"
#include "ei_classifier_porting.h"
#include "numpy.hpp"

#include <stdint.h>
#include <stddef.h>
#include <stdint.h>
#include "edge-impulse-sdk/CMSIS/DSP/Include/arm_math.h"

/* POSIX Header files */
#include <pthread.h>
#include <semaphore.h>

/* Driver Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/I2S.h>

/* Driver configuration */
#include "ti_drivers_config.h"
#include "AudioCodec.h"


#define THREADSTACKSIZE   2048

/* The higher the sampling frequency, the less time we have to process the data, but the higher the sound quality. */
#define SAMPLE_RATE     16000   /* Supported values: 8kHz, 16kHz, 32kHz and 44.1kHz */
#define INPUT_OPTION    AudioCodec_MIC_ONBOARD
#define OUTPUT_OPTION   AudioCodec_SPEAKER_NONE

/* The more storage space we have, the more delay we have, but the more time we have to process the data. */
#define NUMBUFS         3           /* Total number of buffers to loop through */
#define BUFSIZE         11200        /* I2S buffer size */
#define FEATURE_LENGTH  BUFSIZE/2

/* Semaphore used to indicate that data must be processed */
static sem_t semDataReadyForTreatment;
static sem_t semErrorCallback;

/* Lists containing transactions. Each transaction is in turn in these three lists */
List_List i2sReadList;
List_List treatmentList;

/* Buffers containing the data: written by read-interface, modified by treatment, and read by write-interface */
static uint8_t buf1[ BUFSIZE ];
static uint8_t buf2[ BUFSIZE ];
static uint8_t buf3[ BUFSIZE ];
static uint8_t* i2sBufList[ NUMBUFS ] = { buf1, buf2, buf3 };

/* Transactions will successively be part of the i2sReadList, the treatmentList and the i2sWriteList */
I2S_Transaction i2sTransaction1;
I2S_Transaction i2sTransaction2;
I2S_Transaction i2sTransaction3;
static I2S_Transaction *i2sTransactionList[NUMBUFS] = { &i2sTransaction1, &i2sTransaction2,  &i2sTransaction3 };

I2S_Handle i2sHandle;


/* Constants and variables for state checking and recording. */
#define GO_LABEL_INDEX      0
#define NOISE_LABEL_INDEX   1
#define STOP_LABEL_INDEX    2
uint8_t state = 0x00;   // 0x00 = STOP, 0x01 = GO


/* Structs and variables for GPIO control */
#define GPIO_BASE               0x40022000UL
#define DOUT_CONTROL_OFFSET     0x80
#define DOUT_SET_OFFSET         0x90
#define DOUT_CLEAR_OFFSET       0xA0
#define DOUT_TOGGLE_OFFSET      0xB0
#define DIN_OFFSET              0xC0
#define DOE_OFFSET              0xD0
#define EVFLAGS_OFFSET          0xE0

#define DOUT_CONTROL_BASE       (GPIO_BASE + DOUT_CONTROL_OFFSET)
#define DOE_BASE                (GPIO_BASE + DOE_OFFSET)

typedef struct
{
    volatile uint32_t DOUT3_0;      /* Data Out 0 to 3      Address offset: 0x00 */
    volatile uint32_t DOUT7_4;      /* Data Out 4 to 7      Address offset: 0x04 */
    volatile uint32_t DOUT11_8;     /* Data Out 8 to 11     Address offset: 0x08 */
    volatile uint32_t DOUT15_12;    /* Data Out 12 to 15    Address offset: 0x0C */
    volatile uint32_t DOUT19_16;    /* Data Out 16 to 19    Address offset: 0x10 */
    volatile uint32_t DOUT23_20;    /* Data Out 20 to 23    Address offset: 0x14 */
    volatile uint32_t DOUT27_24;    /* Data Out 24 to 27    Address offset: 0x18 */
    volatile uint32_t DOUT31_28;    /* Data Out 28 to 31    Address offset: 0x1C */
} DOUT_TypeDef;

typedef struct
{
    volatile uint32_t value;
} DOUT_CONTROL_TypeDef;

typedef struct
{
    volatile uint32_t value;
} DOE_TypeDef;

#define DOUT            ((DOUT_TypeDef *)           GPIO_BASE)
#define DOUT_Control    ((DOUT_CONTROL_TypeDef *)   DOUT_CONTROL_BASE)
#define DOE             ((DOE_TypeDef *)            DOE_BASE)


static void I2SerrCallback( I2S_Handle handle, int_fast16_t status, I2S_Transaction *transactionPtr )
{
    /* The content of this callback is executed if an I2S error occurs */
    // sem_post(&semErrorCallback);
}

static void I2SwriteCallback( I2S_Handle handle, int_fast16_t status, I2S_Transaction *transactionPtr )
{
    /*
     * The content of this callback is executed every time a write-transaction is started
     */
}

static void I2SreadCallback( I2S_Handle handle, int_fast16_t status, I2S_Transaction *transactionPtr )
{
    /*
     * The content of this callback is executed every time a read-transaction
     * is started
     */

    /* We must consider the previous transaction (the current one is not over) */
    I2S_Transaction *transactionFinished = (I2S_Transaction*)List_prev(&transactionPtr->queueElement);

    if(transactionFinished != NULL){

        /* The finished transaction contains data that must be treated */
        List_remove(&i2sReadList, (List_Elem*)transactionFinished);
        List_put(&treatmentList, (List_Elem*)transactionFinished);

        /* Start the treatment of the data */
        sem_post(&semDataReadyForTreatment);
    }
}


int16_t * features;
int raw_feature_get_data( size_t offset, size_t length, float *out_ptr )
{
    // Convert requested data from 16-bit signed integer to floating point.
    arm_q15_to_float( features + offset, out_ptr, length );
    return 0;
}

void AudioCodec_setup()
{
    /* Initialize TLV320AIC3254 Codec on Audio BP */
    uint8_t status = AudioCodec_open();
    if( AudioCodec_STATUS_SUCCESS != status )
    {
        /* Error Initializing codec */
        while(1);
    }

    /* Configure Codec */
    status =  AudioCodec_config( AudioCodec_TI_3254, AudioCodec_16_BIT,
                                 SAMPLE_RATE, AudioCodec_MONO, OUTPUT_OPTION,
                                 INPUT_OPTION );
    if( AudioCodec_STATUS_SUCCESS != status )
    {
        /* Error Initializing codec */
        while(1);
    }

    /* Volume control */
    AudioCodec_micVolCtrl( AudioCodec_TI_3254, AudioCodec_MIC_ONBOARD, 75 );

    return;
}

void I2S_setup()
{
    /*
     *  Open the I2S driver
     */
    I2S_Params i2sParams;
    I2S_Params_init( &i2sParams );
    
    i2sParams.samplingFrequency =  SAMPLE_RATE;
    i2sParams.fixedBufferLength =  BUFSIZE;
    
    i2sParams.writeCallback     =  I2SwriteCallback;
    i2sParams.readCallback      =  I2SreadCallback;
    i2sParams.errorCallback     =  I2SerrCallback;

    i2sParams.SD1Channels       =  I2S_CHANNELS_MONO_INV;


    i2sHandle = I2S_open( CONFIG_I2S_0, &i2sParams );
    if( i2sHandle == NULL )
    {
        /* Error Opening the I2S driver */
        while(1);
    }


    /* Initialize the queues and the I2S transactions */
    List_clearList( &i2sReadList );
    List_clearList( &treatmentList );


    uint8_t k;
    for( k = 0; k < NUMBUFS; k++ )
    {
        I2S_Transaction_init( i2sTransactionList[ k ] );
        i2sTransactionList[ k ] -> bufPtr  = i2sBufList[ k ];
        i2sTransactionList[ k ] -> bufSize = BUFSIZE;
        List_put( &i2sReadList, (List_Elem*) i2sTransactionList[ k ] );
    }


    I2S_setReadQueueHead( i2sHandle,  (I2S_Transaction*) List_head( &i2sReadList ) );


    /* Start I2S streaming */
    I2S_startClocks( i2sHandle );
    I2S_startRead( i2sHandle );

    return;
}

/*
 *  ======== echoThread ========
 */
void *controlThread( void *arg0 )
{
    AudioCodec_setup();
    I2S_setup();


    /* Prepare the semaphore */
    int retc = sem_init( &semDataReadyForTreatment, 0, 0 );
    if( retc == -1 )
    {
        while (1);
    }


    // 

    // Enable DIO6 and DIO7.
    DOE -> value |= ( 1 << 7 ) | ( 1 << 6 );

    // Turn DIO7 off, DIO6 on.
    DOUT_Control -> value |=  ( 1 << 6 );
    DOUT_Control -> value &= ~( 1 << 7 );


    /* Treatment */
    while(1)
    {
        /* Wait for transaction ready for treatment */
        retc = sem_wait( &semDataReadyForTreatment );
        if( retc == -1 )
        {
            ei_printf( "retc fail.\r\n" );
            while(1);
        }

        I2S_Transaction* transactionToTreat = (I2S_Transaction*) List_head( &treatmentList );

        if(transactionToTreat != NULL)
        {
            features = (int16_t*) transactionToTreat -> bufPtr;


            #if FEATURE_LENGTH != EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE
                ei_printf( "The size of your 'features' array is not correct. Expected %d items, but had %u\r\n",
                            EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, FEATURE_LENGTH );
                while(1);
            #endif


            ei_impulse_result_t result = { 0 };

            signal_t features_signal;
            features_signal.total_length = FEATURE_LENGTH;
            features_signal.get_data = &raw_feature_get_data;

            EI_IMPULSE_ERROR res = run_classifier( &features_signal, &result, false );

            // ei_printf( "run_classifier returned: %d. %s-%.5f    %s-%.5f    %s-%.5f\r\n", 
            //     res, 
            //     result.classification[ 0 ].label, result.classification[ 0 ].value, 
            //     result.classification[ 1 ].label, result.classification[ 1 ].value, 
            //     result.classification[ 2 ].label, result.classification[ 2 ].value );
            if( result.classification[ GO_LABEL_INDEX ].value > 0.50 )
            {
                ei_printf( "GO\r\n" );

                // Turn DIO7 on, DIO6 off.
                DOUT_Control -> value |=  (1 << 7);
                DOUT_Control -> value &= ~(1 << 6);
            }
            else if( result.classification[ STOP_LABEL_INDEX ].value > 0.50 )
            {
                ei_printf( "STOP\r\n" );

                // Turn DIO7 off, DIO6 on.
                DOUT_Control -> value &= ~(1 << 7);
                DOUT_Control -> value |=  (1 << 6);
            }
            else
            {
                ei_printf( "NOISE\r\n" );
            }

            if( res != 0 ) 
            {
                ei_printf( "run_classifier fail.\r\n" );
                return (void*)1;
            }

            /*
            ei_printf( "Predicitions (DSP: %d ms., Classification: %d ms., Anomaly: %d ms.): \r\n",
                        result.timing.dsp, result.timing.classification, result.timing.anomaly );

            ei_printf( "[" );
            for( size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++ )
            {
                ei_printf( "%.5f", result.classification[ ix ].value );
                #if EI_CLASSIFIER_HAS_ANOMALY == 1
                    ei_printf( ", " );
                #else
                    if( ix != EI_CLASSIFIER_LABEL_COUNT - 1 )
                    {
                        ei_printf( ", " );
                    }
                #endif
            }

            #if EI_CLASSIFIER_HAS_ANOMALY == 1
                ei_printf( "%.3f", result.anomaly );
            #endif
            ei_printf( "]\r\n" );
            */

            /* Place in the write-list the transaction we just treated */
            List_remove( &treatmentList, (List_Elem*) transactionToTreat );
            List_put( &i2sReadList, (List_Elem*) transactionToTreat );
        }
    }
}



extern "C" void *ei_main(void *arg0)
{
    pthread_t           thread0;
    pthread_attr_t      attrs;
    struct sched_param  priParam;
    int                 retc;
    int                 detachState;

    /* Call driver init functions */
    I2S_init();
    GPIO_init();

    /* Configure the LED pin */
    GPIO_setConfig( CONFIG_GPIO_LED_0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW );

    /* Turn on user LED */
    GPIO_write( CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON );


    /* Set priority and stack size attributes */
    pthread_attr_init( &attrs );
    priParam.sched_priority = 1;

    detachState = PTHREAD_CREATE_DETACHED;
    retc = pthread_attr_setdetachstate( &attrs, detachState );
    if( retc != 0 )
    {
        /* pthread_attr_setdetachstate() failed */
        while (1);
    }

    pthread_attr_setschedparam( &attrs, &priParam );

    retc |= pthread_attr_setstacksize( &attrs, THREADSTACKSIZE );
    if( retc != 0 )
    {
        /* pthread_attr_setstacksize() failed */
        while (1);
    }

    /* Create receive thread */
    retc = pthread_create( &thread0, &attrs, controlThread, NULL );
    if( retc != 0 )
    {
        /* pthread_create() failed */
        while (1);
    }

    retc = sem_init( &semErrorCallback, 0, 0 );
    if( retc == -1 )
    {
        /* sem_init() failed */
        while (1);
    }

    /* Wait forever for an error */
    sem_wait( &semErrorCallback );

    /* Cancel the echo thread (blocks until the thread is closed) */
    pthread_cancel( &thread0 );

    /* Close the driver */
    I2S_stopClocks( i2sHandle );
    I2S_close( i2sHandle );

    /* End this task */
    return NULL;
}
