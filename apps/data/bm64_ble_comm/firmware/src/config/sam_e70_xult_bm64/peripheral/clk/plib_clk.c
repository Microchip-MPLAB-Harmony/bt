/*******************************************************************************
* Copyright (C) 2019 Microchip Technology Inc. and its subsidiaries.
*
* Subject to your compliance with these terms, you may use Microchip software
* and any derivatives exclusively with Microchip products. It is your
* responsibility to comply with third party license terms applicable to your
* use of third party software (including open source software) that may
* accompany Microchip software.
*
* THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
* EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
* WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
* PARTICULAR PURPOSE.
*
* IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
* INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
* WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
* BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
* FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN
* ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
* THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
*******************************************************************************/

#include "device.h"
#include "plib_clk.h"






/*********************************************************************************
Initialize Main Clock (MAINCK)
*********************************************************************************/
static void CLK_MainClockInitialize(void)
{
    /* Disable Main Crystal Oscillator and Enable External Clock Signal on XIN pin  */
    PMC_REGS->CKGR_MOR = (PMC_REGS->CKGR_MOR & ~CKGR_MOR_MOSCXTEN_Msk) | CKGR_MOR_KEY_PASSWD | CKGR_MOR_MOSCXTBY_Msk;

     /* External clock signal (XIN pin) is selected as the Main Clock (MAINCK) source.
        Switch Main Clock (MAINCK) to External signal on XIN pin */
    PMC_REGS->CKGR_MOR |= CKGR_MOR_KEY_PASSWD | CKGR_MOR_MOSCSEL_Msk;

    /* Wait until MAINCK is switched to External Clock Signal (XIN pin) */
    while ( (PMC_REGS->PMC_SR & PMC_SR_MOSCSELS_Msk) != PMC_SR_MOSCSELS_Msk);


    /* Disable the RC Oscillator */
    PMC_REGS->CKGR_MOR = CKGR_MOR_KEY_PASSWD | (PMC_REGS->CKGR_MOR & ~CKGR_MOR_MOSCRCEN_Msk);

}

/*********************************************************************************
Initialize PLLA (PLLACK)
*********************************************************************************/

static void CLK_PLLAInitialize(void)
{
    /* Configure and Enable PLLA */
    PMC_REGS->CKGR_PLLAR = CKGR_PLLAR_ONE_Msk | CKGR_PLLAR_PLLACOUNT(0x3f) |
                              CKGR_PLLAR_MULA(25 - 1) |
                              CKGR_PLLAR_DIVA(1);

    while ( (PMC_REGS->PMC_SR & PMC_SR_LOCKA_Msk) != PMC_SR_LOCKA_Msk);

}


/*********************************************************************************
Initialize UTMI PLL  (UPLLCK)
*********************************************************************************/

static void CLK_UTMIPLLInitialize(void)
{

    /* Configure Crystal Clock Frequency (12MHz or 16MHz) to select USB PLL (UPLL) multiplication factor
       UPLL multiplication factor is x40 to generate 480MHz from 12MHz
       UPLL multiplication factor is x30 to generate 480MHz from 16MHz  */
    UTMI_REGS->UTMI_CKTRIM= UTMI_CKTRIM_FREQ_XTAL12;

    /* Enable UPLL and configure UPLL lock time */
    PMC_REGS->CKGR_UCKR = CKGR_UCKR_UPLLEN_Msk | CKGR_UCKR_UPLLCOUNT(0x3F);

    /* Wait until PLL Lock occurs */
    while ((PMC_REGS->PMC_SR & PMC_SR_LOCKU_Msk) != PMC_SR_LOCKU_Msk);

    /* UPLL clock frequency is 480MHz (Divider=1) */
    PMC_REGS->PMC_MCKR &= (~PMC_MCKR_UPLLDIV2_Msk);

    /* Wait until clock is ready */
    while ( (PMC_REGS->PMC_SR & PMC_SR_MCKRDY_Msk) != PMC_SR_MCKRDY_Msk);
}

/*********************************************************************************
Initialize Master clock (MCK)
*********************************************************************************/

static void CLK_MasterClockInitialize(void)
{
    /* Program PMC_MCKR.PRES and wait for PMC_SR.MCKRDY to be set   */
    PMC_REGS->PMC_MCKR = (PMC_REGS->PMC_MCKR & ~PMC_MCKR_PRES_Msk) | PMC_MCKR_PRES_CLK_1;
    while ((PMC_REGS->PMC_SR & PMC_SR_MCKRDY_Msk) != PMC_SR_MCKRDY_Msk);

    /* Program PMC_MCKR.MDIV and Wait for PMC_SR.MCKRDY to be set   */
    PMC_REGS->PMC_MCKR = (PMC_REGS->PMC_MCKR & ~PMC_MCKR_MDIV_Msk) | PMC_MCKR_MDIV_PCK_DIV2;
    while ((PMC_REGS->PMC_SR & PMC_SR_MCKRDY_Msk) != PMC_SR_MCKRDY_Msk);

    /* Program PMC_MCKR.CSS and Wait for PMC_SR.MCKRDY to be set    */
    PMC_REGS->PMC_MCKR = (PMC_REGS->PMC_MCKR & ~PMC_MCKR_CSS_Msk) | PMC_MCKR_CSS_PLLA_CLK;
    while ((PMC_REGS->PMC_SR & PMC_SR_MCKRDY_Msk) != PMC_SR_MCKRDY_Msk);

}



/*********************************************************************************
Initialize Generic clock
*********************************************************************************/

static void CLK_GenericClockInitialize(void)
{


    /* Setup Generic/Peripheral Clock for I2S1 */
    PMC_REGS->PMC_PCR =  PMC_PCR_PID(70) | PMC_PCR_CMD_Msk   | PMC_PCR_GCLKEN_Msk |  PMC_PCR_EN_Msk | PMC_PCR_GCLKCSS_PLLA_CLK | PMC_PCR_GCLKDIV(20);

    MATRIX_REGS->CCFG_PCCR |=  CCFG_PCCR_I2SC1CC_Msk ;

}



/*********************************************************************************
Initialize Programmable Clock (PCKx)
*********************************************************************************/

static void CLK_ProgrammableClockInitialize(void)
{
    /* Disable selected programmable clock  */
    PMC_REGS->PMC_SCDR = PMC_SCDR_PCK2_Msk;

    /* Configure selected programmable clock    */
    PMC_REGS->PMC_PCK[2]= PMC_PCK_CSS_MAIN_CLK | PMC_PCK_PRES(0);

    /* Enable selected programmable clock   */
    PMC_REGS->PMC_SCER =    PMC_SCER_PCK2_Msk;

    /* Wait for clock to be ready   */
    while( (PMC_REGS->PMC_SR & (PMC_SR_PCKRDY2_Msk) ) != (PMC_SR_PCKRDY2_Msk));


}


/*********************************************************************************
Clock Initialize
*********************************************************************************/
void CLK_Initialize( void )
{


    /* Initialize Main Clock */
    CLK_MainClockInitialize();

    /* Initialize PLLA */
    CLK_PLLAInitialize();

    /* Initialize UTMI PLL */
    CLK_UTMIPLLInitialize();

    /* Initialize Master Clock */
    CLK_MasterClockInitialize();


    /* Initialize Generic Clock */
    CLK_GenericClockInitialize();

    /* Initialize Programmable Clock */
    CLK_ProgrammableClockInitialize();

    /* Enable Peripheral Clock */
    PMC_REGS->PMC_PCER0=0xc33c00;
}
