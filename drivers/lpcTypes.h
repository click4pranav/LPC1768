/**********************************************************************
* $Id$			lpcTypes.h		13-03-2018
*//**
* @file			lpcTypes.h
* @brief		Contains the NXP ABL typedefs for C standard types.
*     			It is intended to be used in ISO C conforming development
*     			environments and checks for this insofar as it is possible
*     			to do so.
* @version	2.0
* @date			13 MAR 2018
* @author		K G
*
* All rights reserved.
*/

/* Type group ----------------------------------------------------------- */
/** @defgroup LPC_Types LPC_Types
 * @ingroup LPC1700CMSIS_FwLib_Drivers
 * @{
 */

#ifndef LPC_TYPES_H
	#define LPC_TYPES_H

	/* Includes ------------------------------------------------------------------- */
	#include <stdint.h>


	/* Public Types --------------------------------------------------------------- */
	/** @defgroup LPC_Types_Public_Types LPC_Types Public Types
	 * @{
	 */

	/**
	 * @brief Boolean Type definition
	 */
	typedef enum {FALSE = 0, TRUE = !FALSE} Bool;

	/**
	 * @brief Flag Status and Interrupt Flag Status type definition
	 */
	typedef enum {RESET = 0, SET = !RESET} FlagStatus, IntStatus, SetState;
	#define PARAM_SETSTATE(State) ((State==RESET) || (State==SET))

	/**
	 * @brief Functional State Definition
	 */
	typedef enum {DISABLE = 0, ENABLE = !DISABLE} FunctionalState;
	#define PARAM_FUNCTIONALSTATE(State) ((State==DISABLE) || (State==ENABLE))

	/**
	 * @ Status type definition
	 */
	typedef enum {ERROR = 0, SUCCESS = !ERROR} Status;


	/**
	 * Read/Write transfer type mode (Block or non-block)
	 */
	typedef enum
	{
		NONE_BLOCKING = 0,		//< None Blocking type */
		BLOCKING				//< Blocking type */
	} TRANSFER_BLOCK_Type;


	// Pointer to Function returning Void (any number of parameters) */
	typedef void (*PFV)();

	// Pointer to Function returning int32_t (any number of parameters) */
	typedef int32_t(*PFI)();

	/**
	 * @}
	 */


	/* Public Macros -------------------------------------------------------------- */
	/** @defgroup LPC_Types_Public_Macros  LPC_Types Public Macros
	 * @{
	 */

	/* _BIT(n) sets the bit at position "n"
	 * _BIT(n) is intended to be used in "OR" and "AND" expressions:
	 * e.g., "(_BIT(3) | _BIT(7))".
	 */
	#undef _BIT
	/* Set bit macro */
	#define _BIT(n)	(1<<n)

	/* _SBF(f,v) sets the bit field starting at position "f" to value "v".
	 * _SBF(f,v) is intended to be used in "OR" and "AND" expressions:
	 * e.g., "((_SBF(5,7) | _SBF(12,0xF)) & 0xFFFF)"
	 */
	#undef _SBF
	/* Set bit field macro */
	#define _SBF(f,v) (v<<f)

	/* _BITMASK constructs a symbol with 'field_width' least significant
	 * bits set.
	 * e.g., _BITMASK(5) constructs '0x1F', _BITMASK(16) == 0xFFFF
	 * The symbol is intended to be used to limit the bit field width
	 * thusly:
	 * <a_register> = (any_expression) & _BITMASK(x), where 0 < x <= 32.
	 * If "any_expression" results in a value that is larger than can be
	 * contained in 'x' bits, the bits above 'x - 1' are masked off.  When
	 * used with the _SBF example above, the example would be written:
	 * a_reg = ((_SBF(5,7) | _SBF(12,0xF)) & _BITMASK(16))
	 * This ensures that the value written to a_reg is no wider than
	 * 16 bits, and makes the code easier to read and understand.
	 */
	#undef _BITMASK
	/* Bitmask creation macro */
	#define _BITMASK(field_width) ( _BIT(field_width) - 1)


#endif /* LPC_TYPES_H */

/**
 * @}
 */

/* --------------------------------- End Of File ------------------------------ */

