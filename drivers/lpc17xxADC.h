/*-----------------------------------------------------------------------------
 * Name:    lpc17xxADC.h
 * Purpose: A/D Converter interface header file
 * Rev.:    1.0.0
 *-----------------------------------------------------------------------------*/

/* Copyright (c) 2013 - 2014 AKG

   
 *---------------------------------------------------------------------------*/

#ifndef LPC17XX_ADC_H_
	#define LPC17XX_ADC_H

	#include <stdint.h>

	/**
	  \fn          int32_t ADC_Initialize (void)
	  \brief       Initialize Analog-to-Digital Converter
	  \returns
	   - \b  0: function succeeded
	   - \b -1: function failed
	*/
	/**
	  \fn          int32_t ADC_Uninitialize (void)
	  \brief       De-initialize Analog-to-Digital Converter
	  \returns
	   - \b  0: function succeeded
	   - \b -1: function failed
	*/
	/**
	  \fn          int32_t ADC_StartConversion (void)
	  \brief       Start conversion
	  \returns
	   - \b  0: function succeeded
	   - \b -1: function failed
	*/
	/**
	  \fn          int32_t ADC_ConversionDone (void)
	  \brief       Check if conversion finished
	  \returns
	   - \b  0: conversion finished
	   - \b -1: conversion in progress
	*/
	/**
	  \fn          int32_t ADC_GetValue (void)
	  \brief       Get converted value
	  \returns
	   - <b> >=0</b>: converted value
	   - \b -1: conversion in progress or failed
	*/
	/**
	  \fn          uint32_t ADC_GetResolution (void)
	  \brief       Get resolution of Analog-to-Digital Converter
	  \returns     Resolution (in bits)
	*/

	extern int32_t  ADC_Initialize      (void);
	extern int32_t  ADC_Uninitialize    (void);
	extern int32_t  ADC_StartConversion (void);
	extern int32_t  ADC_ConversionDone  (void);
	extern int32_t  ADC_GetValue        (void);
	extern uint32_t ADC_GetResolution   (void);

#endif /* LPC17XX_ADC_H */
