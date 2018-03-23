/*-----------------------------------------------------------------------------
* Filename   : fuse.c
* Title      : Fuse map for AVR chip
* Author     : Patrick Kennedy (PK3)
* Created    : 08/02/2016
* Modified   : 02/26/2018
* Version    : 0.1
* Description:
*   Fuse map
//---------------------------------------------------------------------------*/

#include <avr/io.h>

FUSES = {
    .low = LFUSE_DEFAULT,
    .high = HFUSE_DEFAULT,
    .extended = EFUSE_DEFAULT
};
