/*******************************************************************************
**                      Author(s) Identity                                    **
********************************************************************************
**                                                                            **
** Initials     Name                                                          **
** ---------------------------------------------------------------------------**
** HPL          Heinz-Peter	Liechtenecker                                     **
** DW           Dominik 	Wieland                                           **
** 																			  **
** 																			  **
*******************************************************************************/

/*******************************************************************************
**                      Revision Control History                              **
*******************************************************************************/
/*
 * V0.0: 30-01-2016, HPL: Initial Version
 * V0.1: 21-07-2016, DW:  Port of SW from DAVE3 to DAVE4
 */


#ifndef FIR_FILTER_H_
#define FIR_FILTER_H_


/*******************************************************************************
**                      Includes                                              **
*******************************************************************************/

/*******************************************************************************
**                      Private Constant Definitions to be changed            **
*******************************************************************************/
#define FIR_SIZE           6 /**< Size of circular buffer */
#define MOVING_AVERAGE     0 /**< Constant for filter coefficient selection */


/*******************************************************************************
**                      Private Macro Definitions                             **
*******************************************************************************/

/*******************************************************************************
**                      Global Type Definitions                               **
*******************************************************************************/
/**
 * @brief Structure for FIR Filter
 */
struct structFIR {
	float             CIRC_BUFF[FIR_SIZE]; /**< Circular Buffer for Values */
	float             FIR_COEFF[FIR_SIZE]; /**< Array for coefficients */
	float          	  VALUE;			   /**< current value */
	int               POS;				   /**< Current pointer position */
};

/*******************************************************************************
**                      Private Type Definitions                              **
*******************************************************************************/

/*******************************************************************************
**                      Global Function Declarations                          **
*******************************************************************************/
struct structFIR FIR_FILTER(struct structFIR temp, float NewValue);
struct structFIR Initialize_FIR_Filter(struct structFIR temp, int type);

/*******************************************************************************
**                      Private Function Declarations                         **
*******************************************************************************/

/*******************************************************************************
**                      Global Constant Definitions                           **
*******************************************************************************/

/*******************************************************************************
**                      Private Constant Definitions                          **
*******************************************************************************/

/*******************************************************************************
**                      Global Variable Definitions                           **
*******************************************************************************/

/*******************************************************************************
**                      Private Variable Definitions                          **
*******************************************************************************/

/*******************************************************************************
**                      Global Function Definitions                           **
*******************************************************************************/

/*******************************************************************************
**                      Private Function Definitions                          **
*******************************************************************************/

#endif /* FIR_FILTER_H_ */
