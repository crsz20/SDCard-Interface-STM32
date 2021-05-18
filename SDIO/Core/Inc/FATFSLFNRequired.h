/*
 * FATFSLFNRequired.h
 *
 *  Created on: May 17, 2021
 *      Author: cris1
 */

#ifndef INC_FATFSLFNREQUIRED_H_
#define INC_FATFSLFNREQUIRED_H_

#include "../../Middlewares/Third_Party/FatFs/src/ff.h"

WCHAR ff_convert (WCHAR src, UINT dir);
WCHAR ff_wtoupper(WCHAR chr);

#endif /* INC_FATFSLFNREQUIRED_H_ */
