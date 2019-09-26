/*******************************************************************************
  BM64 Bluetooth Static Driver implementation

  Company:
    Microchip Technology Inc.

  File Name:
    drv_bm64_sha1.h

  Summary:
   BM64 Bluetooth Static Driver header file for SHA hash functions.

  Description:
    This file is the header file for the internal functions of the BM64
    driver related to the SHA hash functions for the BM64.
 
*******************************************************************************/

/*****************************************************************************
 Copyright (C) 2017-2018 Microchip Technology Inc. and its subsidiaries.

Subject to your compliance with these terms, you may use Microchip software 
and any derivatives exclusively with Microchip products. It is your 
responsibility to comply with third party license terms applicable to your 
use of third party software (including open source software) that may 
accompany Microchip software.

THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER 
EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED 
WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A PARTICULAR 
PURPOSE.

IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND 
WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS 
BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE 
FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN 
ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY, 
THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
*****************************************************************************/

#ifndef DRV_BM64_SHA1_H
#define DRV_BM64_SHA1_H

#include <stdint.h>

/****************************************************************************
  Section:
    Data Types
  ***************************************************************************/

// Context storage for a hash operation
typedef struct
{
    uint32_t h0;                                            // Hash state h0
    uint32_t h1;                                            // Hash state h1
    uint32_t h2;                                            // Hash state h2
    uint32_t h3;                                            // Hash state h3
    uint32_t h4;                                            // Hash state h4
    uint32_t bytesSoFar;                                    // Total number of bytes hashed so far
    uint8_t partialBlock[64] __attribute__((aligned(4)));   // Beginning of next 64 byte block
    uint32_t * workingBuffer;                               // Pointer to a working buffer for hash calculation
} DRV_BM64_SHA1_CONTEXT;

/****************************************************************************
  Section:
    Function Prototypes
  ***************************************************************************/
  
// *****************************************************************************
/*
  Function:
    void DRV_BM64_SHA1_Initialize(DRV_BM64_SHA1_CONTEXT* context, uint8_t * workingBuffer);

  Summary:
    Initializes a SHA-1 context to perform a SHA-1 hash.

  Description:
    This routine initializes a hash context for the SHA-1 hash.
    
  Precondition:
    None.

  Parameters:
    context - The context to initialize.
    workingBuffer - A working buffer used by the module to calculate the hash.  If 
        the CRYPTO_HASH_CONFIG_SHA_SMALL_RAM macro is defined in sha_config.h, this 
        buffer must contain 16 uint32_t words.  Otherwise, this buffer must contain 
        80 32-bit words, but performance will be slightly improved.

  Returns:
    None.

  Example:
    <code>
    // Initialization for CRYPTO_HASH_CONFIG_SHA_SMALL_RAM
    uint32_t buffer[16];
    DRV_BM64_SHA1_CONTEXT context;
    DRV_BM64_SHA1_Initialize (&context, buffer);
    </code>

  Remarks:
    You must initialize a context before calculating a SHA-1 hash.
*/
void DRV_BM64_SHA1_Initialize (DRV_BM64_SHA1_CONTEXT* context, uint32_t * workingBuffer);

// *****************************************************************************
/*
  Function:
    void DRV_BM64_SHA1_DataAdd (DRV_BM64_SHA1_CONTEXT* context, uint8_t * data, uint16_t len);

  Summary:
    Adds data to a hash being calculated.

  Description:
    This routine adds data to a SHA-1 hash being calculated.  When the data 
    length reaches a block size (64 bytes), this function will calculate the hash 
    over that block and store the current hash value in the hash context.
    
  Precondition:
    The hash context must be initialized with SHA1_Initialize.

  Parameters:
    context - The context of the hash being calculated.
    data - The data being added.
    len - The length of the data being added.

  Returns:
    None.

  Example:
    <code>
    // Initialization for CRYPTO_HASH_CONFIG_SHA_SMALL_RAM
    uint8_t data[] = "Hello.";
    uint32_t buffer[16];
    DRV_BM64_SHA1_CONTEXT context;

    DRV_BM64_SHA1_Initialize (&context, buffer);

    DRV_BM64_SHA1_DataAdd (&context, data, 6);
    </code>

  Remarks:
    None.
*/
void DRV_BM64_SHA1_DataAdd (DRV_BM64_SHA1_CONTEXT* context, uint8_t * data, uint16_t len);

// *****************************************************************************
/*
  Function:
    void SHA1_Calculate(DRV_BM64_SHA1_CONTEXT* context, uint8_t * result);

  Summary:
    Finishes calculating a hash.

  Description:
    This routine finishes calculating a SHA-1 hash.  It will automatically add 
    the padding required by the hashing algorithm and return the hash digest.
    
  Precondition:
    The hash context must be initialized with SHA1_Initialize.

  Parameters:
    context - The context of the hash being calculated.
    result - A 20-byte buffer to store the calculated hash digest.

  Returns:
    None.

  Example:
    <code>
    // Initialization for CRYPTO_HASH_CONFIG_SHA_SMALL_RAM
    uint8_t data[] = "Hello.";
    uint32_t buffer[16];
    DRV_BM64_SHA1_CONTEXT context;
    uint8_t digest[20];

    DRV_BM64_SHA1_Initialize (&context, buffer);

    DRV_BM64_SHA1_DataAdd (&context, data, 6);
    DRV_BM64_SHA1_Calculate (&context, digest);
    </code>

  Remarks:
    None.
*/
void DRV_BM64_SHA1_Calculate(DRV_BM64_SHA1_CONTEXT* context, uint8_t * result);
#endif

