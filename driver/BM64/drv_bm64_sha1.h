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

/******************************************************************************
Copyright (c) 2017 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
********************************************************************/

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

