// CH32V003 SWIO bit-banged programmer/probe for ESP32-C3
// Based on reference implementation by Charles Lohr (2023) & monte-monte (2024), MIT/NewBSD license.

#ifndef _CH32V003_SWIO_H
#define _CH32V003_SWIO_H

#include <Arduino.h>
#include "soc/gpio_struct.h"
#include "driver/gpio.h"
#include "ch32v003_firmware.h"

#define MAX_IN_TIMEOUT 1000

#define DisableISR portDISABLE_INTERRUPTS
#define EnableISR  portENABLE_INTERRUPTS

struct SWIOState
{
	int t1coeff;
	int pinmask;
	int r_glitch_high;

	uint32_t statetag;
	uint32_t lastwriteflags;
	uint32_t currentstateval;
	uint32_t flash_unlocked;
	uint32_t autoincrement;
};

#if defined(CONFIG_IDF_TARGET_ESP32)
#define GPIO_IN GPIO.in
#define GPIO_SET GPIO.out_w1ts
#define GPIO_CLEAR GPIO.out_w1tc
#define GPIO_ENABLE_SET GPIO.enable_w1ts
#define GPIO_ENABLE_CLEAR GPIO.enable_w1tc
#elif defined(CONFIG_IDF_TARGET_ESP32C3)
#define GPIO_IN GPIO.in.val
#define GPIO_SET GPIO.out_w1ts.val
#define GPIO_CLEAR GPIO.out_w1tc.val
#define GPIO_ENABLE_SET GPIO.enable_w1ts.val
#define GPIO_ENABLE_CLEAR GPIO.enable_w1tc.val
#endif

#define STTAG( x ) (*((uint32_t*)(x)))
#define IRAM IRAM_ATTR

static int DoSongAndDanceToEnterPgmMode( struct SWIOState * state );
static void MCFWriteReg32( struct SWIOState * state, uint8_t command, uint32_t value ) IRAM;
static int MCFReadReg32( struct SWIOState * state, uint8_t command, uint32_t * value ) IRAM;

static int ReadWord( struct SWIOState * iss, uint32_t word, uint32_t * ret );
static int ReadByte( struct SWIOState * iss, uint32_t address_to_read, uint8_t * data );
static int WriteByte( struct SWIOState * iss, uint32_t address_to_write, uint8_t data );
static int WriteWord( struct SWIOState * state, uint32_t word, uint32_t val );
static int WaitForFlash( struct SWIOState * state );
static int WaitForDoneOp( struct SWIOState * state );
static int Write64Block( struct SWIOState * iss, uint32_t address_to_write, const uint8_t * data );
static int ReadBinaryBlob( struct SWIOState * iss, uint32_t address_to_read_from, uint32_t read_size, uint8_t * data );
typedef void (*swio_progress_cb_t)( const char * stage, int done, int total );
static swio_progress_cb_t g_swio_progress_cb = nullptr;
static struct SWIOState g_swio_active_state;
static int g_swio_active_pin = -1;

static int WriteBinaryBlob( struct SWIOState * iss, uint32_t address_to_write, uint32_t blob_size, const uint8_t * blob );
static int UnlockFlash( struct SWIOState * iss );
static int EraseFlash( struct SWIOState * iss, uint32_t address, uint32_t length, int type );
static void ResetInternalProgrammingState( struct SWIOState * iss );
static int HaltMode( struct SWIOState * iss, int mode );

#define DMDATA0        0x04
#define DMDATA1        0x05
#define DMCONTROL      0x10
#define DMSTATUS       0x11
#define DMHARTINFO     0x12
#define DMABSTRACTCS   0x16
#define DMCOMMAND      0x17
#define DMABSTRACTAUTO 0x18
#define DMPROGBUF0     0x20
#define DMPROGBUF1     0x21
#define DMPROGBUF2     0x22
#define DMPROGBUF3     0x23
#define DMPROGBUF4     0x24
#define DMPROGBUF5     0x25
#define DMPROGBUF6     0x26
#define DMPROGBUF7     0x27

#define DMCPBR       0x7C
#define DMCFGR       0x7D
#define DMSHDWCFGR   0x7E

#define R32_FLASH_BASE 0x40022000
#define R32_FLASH_ACTLR 0x40022000
#define R32_FLASH_KEYR 0x40022004
#define R32_FLASH_OBKEYR 0x40022008
#define R32_FLASH_STATR 0x4002200C
#define R32_FLASH_CTLR 0x40022010
#define R32_FLASH_ADDR 0x40022014
#define R32_FLASH_OBR 0x4002201C
#define R32_FLASH_WPR 0x40022020
#define R32_FLASH_MODEKEYR 0x40022024
#define R32_FLASH_BOOT_MODEKEYR 0x40022028

/* FLASH Keys */
#define RDP_Key                    ((uint16_t)0x00A5)
#define FLASH_KEY1                 ((uint32_t)0x45670123)
#define FLASH_KEY2                 ((uint32_t)0xCDEF89AB)
#define CR_LOCK_Set                ((uint32_t)0x00000080)

#define FLASH_STATR_WRPRTERR       ((uint8_t)0x10)
#define CR_PAGE_PG                 ((uint32_t)0x00010000)
#define CR_BUF_LOAD                ((uint32_t)0x00040000)
#define FLASH_CTLR_MER             ((uint16_t)0x0004)     /* Mass Erase */
#define CR_STRT_Set                ((uint32_t)0x00000040)
#define CR_PAGE_ER                 ((uint32_t)0x00020000)
#define CR_BUF_RST                 ((uint32_t)0x00080000)

static inline void Send1Bit( int t1coeff, int pinmask ) IRAM;
static inline void Send0Bit( int t1coeff, int pinmask ) IRAM;
static inline int ReadBit( struct SWIOState * state ) IRAM;

static inline void PrecDelay( int delay )
{
#ifdef __XTENSA__
	asm volatile(
"1:	addi %[delay], %[delay], -1\n"
"	bbci %[delay], 31, 1b\n" : [delay]"+r"(delay) );
#endif
#ifdef __riscv
	asm volatile(
"1:	addi %[delay], %[delay], -1\n"
"bne %[delay], x0, 1b\n" : [delay]"+r"(delay) );
#endif
}

static inline void Send1Bit( int t1coeff, int pinmask )
{
	GPIO_CLEAR = pinmask;
	PrecDelay( t1coeff );
	GPIO_SET = pinmask;
	PrecDelay( t1coeff );
}

static inline void Send0Bit( int t1coeff, int pinmask )
{
	int longwait = t1coeff * 4;
	GPIO_CLEAR = pinmask;
	PrecDelay( longwait );
	GPIO_SET = pinmask;
	PrecDelay( t1coeff );
}

// returns 0 if 0
// returns 1 if 1
// returns 2 if timeout.
static inline int ReadBit( struct SWIOState * state )
{
	int t1coeff = state->t1coeff;
	int pinmask = state->pinmask;

	int timeout = 0;
	int ret = 0;
	int medwait = t1coeff * 2;
	GPIO_CLEAR = pinmask;
	PrecDelay( t1coeff );
	GPIO_ENABLE_CLEAR = pinmask;
	GPIO_SET = pinmask;

	if( state->r_glitch_high )
	{
		int halfwait = t1coeff / 2;
		if( halfwait < 1 ) halfwait = 1;
		PrecDelay( halfwait );
		GPIO_ENABLE_SET = pinmask;
		GPIO_ENABLE_CLEAR = pinmask;
		PrecDelay( halfwait );
	}
	else
	{
		PrecDelay( medwait );
	}
	ret = GPIO_IN;

	if( state->r_glitch_high && !(ret & pinmask) )
	{
		PrecDelay( medwait );
		GPIO_ENABLE_SET = pinmask;
		GPIO_ENABLE_CLEAR = pinmask;
	}

	for( timeout = 0; timeout < MAX_IN_TIMEOUT; timeout++ )
	{
		if( GPIO_IN & pinmask )
		{
			GPIO_ENABLE_SET = pinmask;
			int fastwait = t1coeff / 2;
			if( fastwait < 1 ) fastwait = 1;
			PrecDelay( fastwait );
			return !!(ret & pinmask);
		}
	}

	// Force high anyway so, though hazarded, we can still move along.
	GPIO_ENABLE_SET = pinmask;
	return 2;
}

static void MCFWriteReg32( struct SWIOState * state, uint8_t command, uint32_t value )
{
	int t1coeff = state->t1coeff;
	int pinmask = state->pinmask;

	GPIO_SET = pinmask;
	GPIO_ENABLE_SET = pinmask;

	DisableISR();
	Send1Bit( t1coeff, pinmask );
	uint32_t mask;
	for( mask = 1<<6; mask; mask >>= 1 )
	{
		if( command & mask )
			Send1Bit(t1coeff, pinmask);
		else
			Send0Bit(t1coeff, pinmask);
	}
	Send1Bit( t1coeff, pinmask );
	for( mask = 1<<31; mask; mask >>= 1 )
	{
		if( value & mask )
			Send1Bit(t1coeff, pinmask);
		else
			Send0Bit(t1coeff, pinmask);
	}
	EnableISR();
	esp_rom_delay_us(8);
}

// returns 0 if no error, otherwise error.
static int MCFReadReg32( struct SWIOState * state, uint8_t command, uint32_t * value )
{
	int t1coeff = state->t1coeff;
	int pinmask = state->pinmask;

	GPIO_SET = pinmask;
	GPIO_ENABLE_SET = pinmask;

	DisableISR();
	Send1Bit( t1coeff, pinmask );
	int i;
	uint32_t mask;
	for( mask = 1<<6; mask; mask >>= 1 )
	{
		if( command & mask )
			Send1Bit(t1coeff, pinmask);
		else
			Send0Bit(t1coeff, pinmask);
	}
	Send0Bit( t1coeff, pinmask );
	uint32_t rval = 0;
	for( i = 0; i < 32; i++ )
	{
		rval <<= 1;
		int r = ReadBit( state );
		if( r == 1 )
			rval |= 1;
		if( r == 2 )
		{
			EnableISR();
			return -1;
		}
	}
	*value = rval;
	EnableISR();
	esp_rom_delay_us(8);
	return 0;
}

static inline void ExecuteTimePairs( struct SWIOState * state, const uint16_t * pairs, int numpairs, int iterations )
{
	int t1coeff = state->t1coeff;
	int pinmask = state->pinmask;
	int j, k;
	for( k = 0; k < iterations; k++ )
	{
		const uint16_t * tp = pairs;
		for( j = 0; j < numpairs; j++ )
		{
			int t1v = t1coeff * (*(tp++)) - 1;
			GPIO_CLEAR = pinmask;
			PrecDelay( t1v );
			GPIO_SET = pinmask;
			t1v = t1coeff * (*(tp++)) - 1;
			PrecDelay( t1v );
		}
	}
}

static int DoSongAndDanceToEnterPgmMode( struct SWIOState * state )
{
	int pinmask = state->pinmask;

	static const uint16_t timepairs1[] = {
		32, 12,
		36, 12,
		392, 366
	};

	static const uint16_t timepairs2[] = {
		15, 12,
		32, 12,
		36, 12,
		392, 366
	};

	static const uint16_t timepairs3[] = {
		15, 807,
		24, 8,
		32, 8,
		24, 10,
		20, 8,
		239, 8,
		32, 20,
		8, 32,
		24, 8,
		32, 8,
		26, 7,
		20, 8,
		239, 8,
		32, 20,
		8, 22,
		24, 8,
		24, 8,
		31, 6,
		25, 307,
	};

	GPIO_SET = pinmask;
	GPIO_ENABLE_SET = pinmask;

	DisableISR();

	ExecuteTimePairs( state, timepairs1, 3, 1 );
	ExecuteTimePairs( state, timepairs2, 4, 199 );
	ExecuteTimePairs( state, timepairs3, 19, 10 );

	int present = ReadBit( state );
	GPIO_ENABLE_SET = pinmask;
	GPIO_CLEAR = pinmask;
	esp_rom_delay_us( 2000 );
	GPIO_SET = pinmask;
	EnableISR();
	esp_rom_delay_us( 10 );

	return present;
}

static int WaitForFlash( struct SWIOState * iss )
{
	struct SWIOState * dev = iss;
	uint32_t rw, timeout = 0;
	do
	{
		rw = 0;
		ReadWord( dev, 0x4002200C, &rw ); // FLASH_STATR => 0x4002200C
	} while( (rw & 1) && timeout++ < 500 );  // BSY flag.

	WriteWord( dev, 0x4002200C, 0 );

	if( rw & FLASH_STATR_WRPRTERR )
		return -44;

	if( rw & 1 )
		return -5;

	return 0;
}

static int WaitForDoneOp( struct SWIOState * iss )
{
	int r;
	uint32_t rrv = 0;
	int ret = 0;
	struct SWIOState * dev = iss;
	int timeout = 0;
	do
	{
		r = MCFReadReg32( dev, DMABSTRACTCS, &rrv );
		if( r ) return r;
		if( timeout++ > 2000 )
		{
			uint32_t dms = 0;
			MCFReadReg32( dev, DMSTATUS, &dms );
			Serial.printf( "[SWIO] WaitForDoneOp timeout! DMABSTRACTCS=0x%08X DMSTATUS=0x%08X\r\n", (unsigned)rrv, (unsigned)dms );
			return -98;
		}
	}
	while( rrv & (1<<12) );
	if( (rrv >> 8 ) & 7 )
	{
		MCFWriteReg32( dev, DMABSTRACTCS, 0x00000700 );
		ret = -33;
	}
	return ret;
}

static void StaticUpdatePROGBUFRegs( struct SWIOState * dev )
{
	MCFWriteReg32( dev, DMDATA0, 0xe00000f4 );   // DATA0's location in memory.
	MCFWriteReg32( dev, DMCOMMAND, 0x0023100a ); // Copy data to x10
	WaitForDoneOp( dev );
	MCFWriteReg32( dev, DMDATA0, 0xe00000f8 );   // DATA1's location in memory.
	MCFWriteReg32( dev, DMCOMMAND, 0x0023100b ); // Copy data to x11
	WaitForDoneOp( dev );
	MCFWriteReg32( dev, DMDATA0, 0x40022010 );   // FLASH->CTLR
	MCFWriteReg32( dev, DMCOMMAND, 0x0023100c ); // Copy data to x12
	WaitForDoneOp( dev );
	MCFWriteReg32( dev, DMDATA0, CR_PAGE_PG|CR_BUF_LOAD );
	MCFWriteReg32( dev, DMCOMMAND, 0x0023100d ); // Copy data to x13
	WaitForDoneOp( dev );
}

static void ResetInternalProgrammingState( struct SWIOState * iss )
{
	iss->statetag = 0;
	iss->lastwriteflags = 0;
	iss->currentstateval = 0;
	iss->flash_unlocked = 0;
	iss->autoincrement = 0;
}

static int ReadWord( struct SWIOState * iss, uint32_t address_to_read, uint32_t * data )
{
	struct SWIOState * dev = iss;

	int autoincrement = 1;
	if( address_to_read == 0x40022010 || address_to_read == 0x4002200C )
		autoincrement = 0;

	if( iss->statetag != STTAG( "RDSQ" ) || address_to_read != iss->currentstateval || autoincrement != iss->autoincrement )
	{
		if( iss->statetag != STTAG( "RDSQ" ) || autoincrement != iss->autoincrement )
		{
			MCFWriteReg32( dev, DMABSTRACTAUTO, 0 );

			MCFWriteReg32( dev, DMPROGBUF0, 0x40044180 );
			if( autoincrement )
			{
				MCFWriteReg32( dev, DMPROGBUF1, 0xc1040411 );
			}
			else
			{
				MCFWriteReg32( dev, DMPROGBUF1, 0xc1040001 );
			}
			MCFWriteReg32( dev, DMPROGBUF2, 0x9002c180 );

			if( iss->statetag != STTAG( "WRSQ" ) )
			{
				StaticUpdatePROGBUFRegs( dev );
			}
			MCFWriteReg32( dev, DMABSTRACTAUTO, 1 );
			iss->autoincrement = autoincrement;
		}

		MCFWriteReg32( dev, DMDATA1, address_to_read );
		MCFWriteReg32( dev, DMCOMMAND, 0x00241000 );

		iss->statetag = STTAG( "RDSQ" );
		iss->currentstateval = address_to_read;

		int r = WaitForDoneOp( dev );
		if( r ) return r;
	}

	if( iss->autoincrement )
		iss->currentstateval += 4;

	return MCFReadReg32( dev, DMDATA0, data );
}

static int ReadByte( struct SWIOState * iss, uint32_t address_to_read, uint8_t * data )
{
	int ret = 0;
	struct SWIOState * dev = iss;

	iss->statetag = STTAG( "XXXX" );

	MCFWriteReg32( dev, DMABSTRACTAUTO, 0x00000000 );

	MCFWriteReg32( dev, DMPROGBUF0, 0x00048403 ); // lb x8, 0(x9)
	MCFWriteReg32( dev, DMPROGBUF1, 0x00100073 ); // c.ebreak

	MCFWriteReg32( dev, DMDATA0, address_to_read );
	MCFWriteReg32( dev, DMCOMMAND, 0x00231009 );
	MCFWriteReg32( dev, DMCOMMAND, 0x00241000 );
	MCFWriteReg32( dev, DMCOMMAND, 0x00221008 );

	ret |= WaitForDoneOp( dev );
	iss->currentstateval = -1;

	uint32_t rr;
	ret |= MCFReadReg32( dev, DMDATA0, &rr );
	*data = rr;
	return ret;
}

static int WriteByte( struct SWIOState * iss, uint32_t address_to_write, uint8_t data )
{
	struct SWIOState * dev = iss;

	int ret = 0;
	iss->statetag = STTAG( "XXXX" );

	MCFWriteReg32( dev, DMABSTRACTAUTO, 0x00000000 );

	MCFWriteReg32( dev, DMPROGBUF0, 0x00848023 ); // sb x8, 0(x9)
	MCFWriteReg32( dev, DMPROGBUF1, 0x00100073 ); // c.ebreak

	MCFWriteReg32( dev, DMDATA0, address_to_write );
	MCFWriteReg32( dev, DMCOMMAND, 0x00231009 );
	MCFWriteReg32( dev, DMDATA0, data );
	MCFWriteReg32( dev, DMCOMMAND, 0x00271008 );

	ret |= WaitForDoneOp( dev );
	iss->currentstateval = -1;
	return ret;
}

static int WriteWord( struct SWIOState * iss, uint32_t address_to_write, uint32_t data )
{
	struct SWIOState * dev = iss;

	int ret = 0;

	int is_flash = 0;
	if( ( address_to_write & 0xff000000 ) == 0x08000000 || ( address_to_write & 0x1FFFF800 ) == 0x1FFFF000 )
	{
		is_flash = 1;
	}

	if( iss->statetag != STTAG( "WRSQ" ) || is_flash != iss->lastwriteflags )
	{
		int did_disable_req = 0;
		if( iss->statetag != STTAG( "WRSQ" ) )
		{
			MCFWriteReg32( dev, DMABSTRACTAUTO, 0x00000000 );
			did_disable_req = 1;
			MCFWriteReg32( dev, DMPROGBUF0, 0xc0804184 );
			MCFWriteReg32( dev, DMPROGBUF1, 0xc1840491 );

			if( iss->statetag != STTAG( "RDSQ" ) )
			{
				StaticUpdatePROGBUFRegs( dev );
			}
		}

		if( iss->lastwriteflags != is_flash || iss->statetag != STTAG( "WRSQ" ) )
		{
			if( is_flash )
			{
				MCFWriteReg32( dev, DMPROGBUF2, 0x9002c214 );
			}
			else
			{
				MCFWriteReg32( dev, DMPROGBUF2, 0x00019002 );
			}
		}

		MCFWriteReg32( dev, DMDATA1, address_to_write );
		MCFWriteReg32( dev, DMDATA0, data );

		if( did_disable_req )
		{
			MCFWriteReg32( dev, DMCOMMAND, 0x00271008 );
			MCFWriteReg32( dev, DMABSTRACTAUTO, 1 );
		}
		iss->lastwriteflags = is_flash;

		iss->statetag = STTAG( "WRSQ" );
		iss->currentstateval = address_to_write;

		if( is_flash )
			ret |= WaitForDoneOp( dev );
	}
	else
	{
		if( address_to_write != iss->currentstateval )
		{
			MCFWriteReg32( dev, DMABSTRACTAUTO, 0 );
			MCFWriteReg32( dev, DMDATA1, address_to_write );
			MCFWriteReg32( dev, DMABSTRACTAUTO, 1 );
		}
		MCFWriteReg32( dev, DMDATA0, data );
		ret |= WaitForDoneOp( dev );
	}

	iss->currentstateval += 4;
	return ret;
}

static int UnlockFlash( struct SWIOState * iss )
{
	struct SWIOState * dev = iss;

	uint32_t rw = 0;
	ReadWord( dev, 0x40022010, &rw ); // FLASH->CTLR = 0x40022010
	if( rw & 0x8080 )
	{
		WriteWord( dev, 0x40022004, 0x45670123 ); // FLASH->KEYR
		WriteWord( dev, 0x40022004, 0xCDEF89AB );
		WriteWord( dev, 0x40022008, 0x45670123 ); // OBKEYR
		WriteWord( dev, 0x40022008, 0xCDEF89AB );
		WriteWord( dev, 0x40022024, 0x45670123 ); // MODEKEYR
		WriteWord( dev, 0x40022024, 0xCDEF89AB );

		ReadWord( dev, 0x40022010, &rw );
		if( rw & 0x8080 )
		{
			return -9;
		}
	}
	iss->flash_unlocked = 1;
	return 0;
}

static int EraseFlash( struct SWIOState * iss, uint32_t address, uint32_t length, int type )
{
	struct SWIOState * dev = iss;

	uint32_t rw;

	if( !iss->flash_unlocked )
	{
		if( ( rw = UnlockFlash( iss ) ) )
			return rw;
	}

	if( type == 1 )
	{
		iss->statetag = STTAG( "XXXX" );
		WriteWord( dev, 0x40022010, 0 );
		WriteWord( dev, 0x40022010, FLASH_CTLR_MER );
		WriteWord( dev, 0x40022010, CR_STRT_Set|FLASH_CTLR_MER );
		if( WaitForFlash( dev ) ) return -13;
		WriteWord( dev, 0x40022010, 0 );
	}
	else
	{
		uint32_t chunk_to_erase = address;

		while( chunk_to_erase < address + length )
		{
			if( WaitForFlash( dev ) ) return -14;

			WriteWord( dev, 0x40022010, CR_PAGE_ER );
			WriteWord( dev, 0x40022014, chunk_to_erase );
			WriteWord( dev, 0x40022010, CR_STRT_Set|CR_PAGE_ER );
			if( WaitForFlash( dev ) ) return -15;

			WriteWord( dev, 0x40022010, 0 );
			chunk_to_erase += 64;
		}
	}
	return 0;
}

static int Write64Block( struct SWIOState * iss, uint32_t address_to_write, const uint8_t * blob )
{
	struct SWIOState * dev = iss;

	const int blob_size = 64;
	uint32_t wp = address_to_write;
	uint32_t ew = wp + blob_size;
	int group = -1;
	int is_flash = 0;
	int rw = 0;

	if( (address_to_write & 0xff000000) == 0x08000000 || (address_to_write & 0xff000000) == 0x00000000 || (address_to_write & 0x1FFFF800) == 0x1FFFF000 )
	{
		if( !iss->flash_unlocked )
		{
			if( ( rw = UnlockFlash( dev ) ) )
				return rw;
		}

		is_flash = 1;
		rw = EraseFlash( dev, address_to_write, blob_size, 0 );
		if( rw ) return rw;
	}

	while( wp < ew )
	{
		if( is_flash )
		{
			group = (wp & 0xffffffc0);
			WriteWord( dev, 0x40022010, CR_PAGE_PG );
			WriteWord( dev, 0x40022010, CR_BUF_RST | CR_PAGE_PG );

			for( int j = 0; j < 16; j++ )
			{
				int index = (wp - address_to_write);
				uint32_t data = 0xffffffff;
				if( index + 3 < blob_size )
					memcpy( &data, &blob[index], 4 );
				else if( (int32_t)(blob_size - index) > 0 )
					memcpy( &data, &blob[index], blob_size - index );
				WriteWord( dev, wp, data );
				wp += 4;
			}
			WriteWord( dev, 0x40022014, group );
			WriteWord( dev, 0x40022010, CR_PAGE_PG|CR_STRT_Set );
			if( (rw = WaitForFlash( dev ) ) ) return rw;
		}
		else
		{
			int index = (wp - address_to_write);
			uint32_t data = 0xffffffff;
			if( index + 3 < blob_size )
				memcpy( &data, &blob[index], 4 );
			else if( (int32_t)(blob_size - index) > 0 )
				memcpy( &data, &blob[index], blob_size - index );
			WriteWord( dev, wp, data );
			wp += 4;
		}
	}

	return 0;
}

static int ReadBinaryBlob( struct SWIOState * iss, uint32_t address_to_read_from, uint32_t read_size, uint8_t * blob )
{
	struct SWIOState * dev = iss;

	uint32_t rpos = address_to_read_from;
	uint32_t rend = address_to_read_from + read_size;

	while( rpos < rend )
	{
		int r;
		int remain = rend - rpos;

		if( ( rpos & 3 ) == 0 && remain >= 4 )
		{
			uint32_t rw;
			r = ReadWord( dev, rpos, &rw );
			if( r ) return r;
			int rem = remain;
			if( rem > 4 ) rem = 4;
			memcpy( blob, &rw, rem );
			blob += 4;
			rpos += 4;
		}
		else
		{
			for( ; remain > 0; remain-- )
			{
				uint8_t rw;
				r = ReadByte( dev, rpos, &rw );
				if( r ) return r;
				memcpy( blob, &rw, 1 );
				blob += 1;
				rpos += 1;
			}
		}
	}
	int r = WaitForDoneOp( dev );
	return r;
}

static int WriteBinaryBlob( struct SWIOState * iss, uint32_t address_to_write, uint32_t blob_size, const uint8_t * blob )
{
	struct SWIOState * dev = iss;

	uint32_t rw;
	int is_flash = 0;

	if( blob_size == 0 ) return 0;

	if( (address_to_write & 0xff000000) == 0x08000000 || (address_to_write & 0xff000000) == 0x00000000 || (address_to_write & 0x1FFFF800) == 0x1FFFF000 )
		is_flash = 1;

	if( is_flash )
		address_to_write |= 0x08000000;

	if( is_flash && !iss->flash_unlocked )
	{
		if( ( rw = UnlockFlash( iss ) ) )
			return rw;
	}

	uint8_t tempblock[64];
	uint8_t tempblock2[64];
	int sblock = address_to_write >> 6;
	int eblock = ( address_to_write + blob_size + 0x3f ) >> 6;
	int rsofar = 0;

	for( int b = sblock; b < eblock; b++ )
	{
		int offset_in_block = address_to_write - (b * 64);
		if( offset_in_block < 0 ) offset_in_block = 0;
		int end_o_plus_one_in_block = ( address_to_write + blob_size ) - (b * 64);
		if( end_o_plus_one_in_block > 64 ) end_o_plus_one_in_block = 64;
		int base = b * 64;

		if( offset_in_block == 0 && end_o_plus_one_in_block == 64 )
		{
			int r = 0;
			for( int i = 0; i < 10; i++ )
			{
				r = Write64Block( dev, base, blob + rsofar );
				ReadBinaryBlob( dev, base, 64, tempblock );
				if( !memcmp( blob + rsofar, tempblock, 64 ) ) break;
				if( i == 9 )
				{
					Serial.printf( "[SWIO] Block verify failed at offset %d\r\n", rsofar );
					return -99;
				}
			}
			rsofar += 64;
			if( r ) return r;
		}
		else
		{
			if( is_flash )
			{
				memset( tempblock, 0xff, 64 );
				int tocopy = end_o_plus_one_in_block - offset_in_block;
				memcpy( tempblock + offset_in_block, blob + rsofar, tocopy );

				int r = 0;
				for( int i = 0; i < 10; i++ )
				{
					r = Write64Block( dev, base, tempblock );
					ReadBinaryBlob( dev, base, 64, tempblock2 );
					if( !memcmp( tempblock, tempblock2, 64 ) ) break;
					if( i == 9 ) return -99;
				}
				rsofar += tocopy;
				if( r ) return r;
				if( WaitForFlash( dev ) ) return -5;
			}
		}

		if( (rsofar & 0xff) == 0 || (uint32_t)rsofar >= blob_size )
		{
			if( g_swio_progress_cb )
			{
				g_swio_progress_cb( "Writing & verifying...", rsofar, (int)blob_size );
			}
		}
		if( (rsofar & 0x3ff) == 0 || (uint32_t)rsofar >= blob_size )
		{
			Serial.printf( "[SWIO] Wrote & verified %d / %u bytes\r\n", rsofar, (unsigned)blob_size );
		}
	}

	WaitForDoneOp( dev );
	return 0;
}

static int HaltMode( struct SWIOState * iss, int mode )
{
	struct SWIOState * dev = iss;

	switch( mode )
	{
	case 5: // Don't reboot, just halt
	case 0: // Halt
		MCFWriteReg32( dev, DMSHDWCFGR, 0x5aa50000 | (1<<10) );
		MCFWriteReg32( dev, DMCFGR, 0x5aa50000 | (1<<10) );
		MCFWriteReg32( dev, DMCFGR, 0x5aa50000 | (1<<10) );

		MCFWriteReg32( dev, DMCONTROL, 0x80000001 );
		MCFWriteReg32( dev, DMCONTROL, 0x80000001 );
		WaitForDoneOp( dev );
		break;
	case 1: // Reboot and resume
		MCFWriteReg32( dev, DMCONTROL, 0x80000001 );
		MCFWriteReg32( dev, DMCONTROL, 0x80000001 );
		MCFWriteReg32( dev, DMCONTROL, 0x80000003 );
		MCFWriteReg32( dev, DMCONTROL, 0x40000001 );
		break;
	case 2: // Resume
		MCFWriteReg32( dev, DMSHDWCFGR, 0x5aa50000 | (1<<10) );
		MCFWriteReg32( dev, DMCFGR, 0x5aa50000 | (1<<10) );
		MCFWriteReg32( dev, DMCFGR, 0x5aa50000 | (1<<10) );

		MCFWriteReg32( dev, DMCONTROL, 0x40000001 );
		WaitForDoneOp( dev );
		break;
	}
	return 0;
}

static void swio_configure_gpio( int pin, int mode, struct SWIOState * state )
{
	gpio_num_t g = (gpio_num_t)pin;
	gpio_reset_pin( g );
	gpio_pullup_en( g );
	gpio_pulldown_dis( g );
	gpio_set_level( g, 1 );

	if( mode == 0 )
	{
		// Push-pull during write, tri-stated with internal pull-up + brief active-high glitch during read
		gpio_set_direction( g, GPIO_MODE_INPUT_OUTPUT );
		gpio_set_drive_capability( g, GPIO_DRIVE_CAP_0 );
		state->r_glitch_high = 1;
	}
	else if( mode == 1 )
	{
		// Push-pull during write, tri-stated with pull-up during read (no glitch)
		gpio_set_direction( g, GPIO_MODE_INPUT_OUTPUT );
		gpio_set_drive_capability( g, GPIO_DRIVE_CAP_2 );
		state->r_glitch_high = 0;
	}
	else
	{
		// Open-drain with pull-up
		gpio_set_direction( g, GPIO_MODE_INPUT_OUTPUT_OD );
		gpio_set_drive_capability( g, GPIO_DRIVE_CAP_2 );
		state->r_glitch_high = 0;
	}
}

static bool swio_try_attempt( struct SWIOState * state, int pin, int mode, int t1coeff, bool song_and_dance, uint32_t * out_dmstatus )
{
	memset( state, 0, sizeof( *state ) );
	state->pinmask = (1 << pin);
	state->t1coeff = t1coeff;
	swio_configure_gpio( pin, mode, state );
	esp_rom_delay_us( 50 );

	if( song_and_dance )
	{
		// Active-low bus reset pulse + DM reset before song-and-dance if sd==0 didn't connect
		GPIO_ENABLE_SET = state->pinmask;
		GPIO_CLEAR = state->pinmask;
		esp_rom_delay_us( 2000 );
		GPIO_SET = state->pinmask;
		esp_rom_delay_us( 100 );
		DoSongAndDanceToEnterPgmMode( state );
		MCFWriteReg32( state, DMSHDWCFGR, 0x5aa50000 | (1<<10) );
		MCFWriteReg32( state, DMCFGR, 0x5aa50000 | (1<<10) );
		MCFWriteReg32( state, DMCONTROL, 0x00000000 );
		esp_rom_delay_us( 50 );
		MCFWriteReg32( state, DMCONTROL, 0x00000001 );
		esp_rom_delay_us( 50 );
	}

	MCFWriteReg32( state, DMSHDWCFGR, 0x5aa50000 | (1<<10) );
	MCFWriteReg32( state, DMCFGR, 0x5aa50000 | (1<<10) );
	MCFWriteReg32( state, DMCFGR, 0x5aa50000 | (1<<10) );
	MCFWriteReg32( state, DMABSTRACTAUTO, 0x00000000 );

	for( int attempt = 0; attempt < 5; attempt++ )
	{
		uint32_t reg1 = 0, reg2 = 0;
		int r1 = MCFReadReg32( state, DMSTATUS, &reg1 );
		int r2 = MCFReadReg32( state, DMSTATUS, &reg2 );
		if( r1 == 0 && r2 == 0 && reg1 == reg2 && reg1 != 0x00000000 && reg1 != 0xffffffff && ((reg1 & 0xc0) != 0x40) )
		{
			*out_dmstatus = reg1;
			state->statetag = STTAG( "STRT" );
			return true;
		}
	}
	return false;
}

static bool swio_auto_connect( struct SWIOState * state, int pin )
{
	if( g_swio_active_pin == pin && g_swio_active_state.t1coeff > 0 )
	{
		uint32_t dms1 = 0, dms2 = 0;
		if( MCFReadReg32( &g_swio_active_state, DMSTATUS, &dms1 ) == 0 &&
		    MCFReadReg32( &g_swio_active_state, DMSTATUS, &dms2 ) == 0 &&
		    dms1 == dms2 && dms1 != 0x00000000 && dms1 != 0xffffffff && ((dms1 & 0xc0) != 0x40) )
		{
			*state = g_swio_active_state;
			Serial.printf( "[SWIO] Reusing active connection on GPIO%d (t1coeff=%d, DMSTATUS=0x%08X)\r\n",
			               pin, state->t1coeff, (unsigned)dms1 );
			return true;
		}
		g_swio_active_pin = -1;
	}

	pinMode( pin, INPUT_PULLDOWN );
	delay( 2 );
	int pd = digitalRead( pin );
	pinMode( pin, INPUT_PULLUP );
	delay( 2 );
	int pu = digitalRead( pin );
	Serial.printf( "[SWIO] GPIO%d electrical check: pulldown=%d, pullup=%d\r\n", pin, pd, pu );

	static const int coeffs[] = { 7, 8, 9, 10, 6, 12, 14, 16, 5, 4 };
	const char * mode_names[] = { "PushPull+GlitchHigh", "PushPull+PullUp", "OpenDrain+PullUp" };

	for( int sd = 0; sd <= 1; sd++ )
	{
		for( int mode = 0; mode < 3; mode++ )
		{
			for( size_t ci = 0; ci < sizeof(coeffs)/sizeof(coeffs[0]); ci++ )
			{
				uint32_t dmstatus = 0;
				if( swio_try_attempt( state, pin, mode, coeffs[ci], sd != 0, &dmstatus ) )
				{
					g_swio_active_state = *state;
					g_swio_active_pin = pin;
					Serial.printf( "[SWIO] Connected on GPIO%d! mode=%s, t1coeff=%d, song_and_dance=%d, DMSTATUS=0x%08X\r\n",
					               pin, mode_names[mode], coeffs[ci], sd, (unsigned)dmstatus );
					return true;
				}
			}
		}
	}

	pinMode( pin, INPUT_PULLUP );
	Serial.printf( "[SWIO] Failed to connect to CH32V003 on GPIO%d. Check jumper wire to SWDIO pad.\r\n", pin );
	return false;
}

// Safe single-word memory read that verifies PROGBUF contents before executing
static int swio_read_word_safe( struct SWIOState * state, uint32_t addr, uint32_t * out_val )
{
	MCFWriteReg32( state, DMABSTRACTAUTO, 0x00000000 );
	// Clear any prior cmderr
	MCFWriteReg32( state, DMABSTRACTCS, 0x00000700 );

	// lw x8, 0(x9)  -> 0x0004A403
	// ebreak        -> 0x00100073
	MCFWriteReg32( state, DMPROGBUF0, 0x0004A403 );
	MCFWriteReg32( state, DMPROGBUF1, 0x00100073 );

	uint32_t pb0 = 0, pb1 = 0;
	int r0 = MCFReadReg32( state, DMPROGBUF0, &pb0 );
	int r1 = MCFReadReg32( state, DMPROGBUF1, &pb1 );
	if( r0 != 0 || r1 != 0 || pb0 != 0x0004A403 || pb1 != 0x00100073 )
	{
		Serial.printf( "[SWIO] PROGBUF verify failed! pb0=0x%08X pb1=0x%08X\r\n", (unsigned)pb0, (unsigned)pb1 );
		return -50;
	}

	// Load addr into x9
	MCFWriteReg32( state, DMDATA0, addr );
	MCFWriteReg32( state, DMCOMMAND, 0x00231009 );
	int rw = WaitForDoneOp( state );
	if( rw ) return rw;

	// Execute PROGBUF (lw x8, 0(x9); ebreak)
	MCFWriteReg32( state, DMCOMMAND, 0x00241000 );
	rw = WaitForDoneOp( state );
	if( rw ) return rw;

	// Copy x8 into DMDATA0
	MCFWriteReg32( state, DMCOMMAND, 0x00221008 );
	rw = WaitForDoneOp( state );
	if( rw ) return rw;

	state->statetag = STTAG( "XXXX" );
	return MCFReadReg32( state, DMDATA0, out_val );
}

static void swio_probe_chip( int pin )
{
	Serial.printf( "\r\n=== Probing CH32V003 via SWIO on GPIO%d ===\r\n", pin );
	struct SWIOState state;
	if( !swio_auto_connect( &state, pin ) )
	{
		return;
	}

	uint32_t dmstatus = 0, dmcontrol = 0, dmhartinfo = 0, dmabstractcs = 0;
	MCFReadReg32( &state, DMSTATUS, &dmstatus );
	MCFReadReg32( &state, DMCONTROL, &dmcontrol );
	MCFReadReg32( &state, DMHARTINFO, &dmhartinfo );
	Serial.printf( "[SWIO] Step 1 (Pre-Halt): DMSTATUS=0x%08X  DMCONTROL=0x%08X  DMHARTINFO=0x%08X\r\n",
	               (unsigned)dmstatus, (unsigned)dmcontrol, (unsigned)dmhartinfo );

	MCFWriteReg32( &state, DMCONTROL, 0x80000001 );
	MCFWriteReg32( &state, DMCONTROL, 0x80000001 );
	MCFReadReg32( &state, DMSTATUS, &dmstatus );
	MCFReadReg32( &state, DMABSTRACTCS, &dmabstractcs );
	Serial.printf( "[SWIO] Step 2 (Post-Halt): DMSTATUS=0x%08X  DMABSTRACTCS=0x%08X\r\n",
	               (unsigned)dmstatus, (unsigned)dmabstractcs );

	uint32_t test_data0 = 0;
	MCFWriteReg32( &state, DMDATA0, 0xA55A1234 );
	MCFReadReg32( &state, DMDATA0, &test_data0 );
	MCFReadReg32( &state, DMSTATUS, &dmstatus );
	Serial.printf( "[SWIO] Step 3 (DMDATA0 test): wrote 0xA55A1234 -> read 0x%08X | DMSTATUS=0x%08X\r\n",
	               (unsigned)test_data0, (unsigned)dmstatus );

	uint32_t ob0 = 0, uid1 = 0, uid2 = 0, uid3 = 0, fsize = 0;
	int r_ob  = swio_read_word_safe( &state, 0x1FFFF800, &ob0 );
	int r_u1  = swio_read_word_safe( &state, 0x1FFFF7E8, &uid1 );
	int r_u2  = swio_read_word_safe( &state, 0x1FFFF7EC, &uid2 );
	int r_u3  = swio_read_word_safe( &state, 0x1FFFF7F0, &uid3 );
	int r_fs  = swio_read_word_safe( &state, 0x1FFFF7E0, &fsize );

	if( r_ob == 0 && r_u1 == 0 && r_fs == 0 )
	{
		Serial.printf( "[SWIO] Option Bytes (USER/RDPR): 0x%04X / 0x%04X\r\n", (unsigned)(ob0 >> 16), (unsigned)(ob0 & 0xFFFF) );
		Serial.printf( "[SWIO] Chip UID: %08X-%08X-%08X | Flash Size: %u kB\r\n",
		               (unsigned)uid1, (unsigned)uid2, (unsigned)uid3, (unsigned)(fsize & 0xFFFF) );
	}
	else
	{
		Serial.printf( "[SWIO] Warning: Failed reading system info words (r=%d,%d,%d)\r\n", r_ob, r_u1, r_fs );
	}

	uint8_t flash_head[64];
	memset( flash_head, 0, sizeof(flash_head) );
	int r_fl = 0;
	for( int w = 0; w < 16; w++ )
	{
		uint32_t word_val = 0;
		r_fl = swio_read_word_safe( &state, 0x08000000 + w * 4, &word_val );
		if( r_fl != 0 ) break;
		memcpy( &flash_head[w * 4], &word_val, 4 );
	}

	if( r_fl == 0 )
	{
		Serial.println( "[SWIO] First 64 bytes of Flash at 0x08000000:" );
		for( int row = 0; row < 4; row++ )
		{
			Serial.printf( "  0x%08X: ", 0x08000000 + row * 16 );
			for( int col = 0; col < 16; col++ )
			{
				Serial.printf( "%02X ", flash_head[row * 16 + col] );
			}
			Serial.println();
		}

		bool all_ff = true;
		bool all_e339 = true;
		for( int i = 0; i < 64; i += 2 )
		{
			if( flash_head[i] != 0xFF || flash_head[i+1] != 0xFF ) all_ff = false;
			if( flash_head[i] != 0x39 || flash_head[i+1] != 0xE3 ) all_e339 = false;
		}
		bool matches_stock = (memcmp( flash_head, ch32v003_firmware_bin, 64 ) == 0);

		if( all_ff || all_e339 )
		{
			Serial.println( "[SWIO] VERDICT: CH32V003 Flash is BLANK / UNPROGRAMMED! (Send 'f' or 'F' to flash stock firmware)" );
		}
		else if( matches_stock )
		{
			Serial.println( "[SWIO] VERDICT: First 64 bytes MATCH compiled stock CH32V003 firmware!" );
		}
		else
		{
			Serial.println( "[SWIO] VERDICT: CH32V003 Flash contains non-blank data, but differs from compiled stock firmware." );
		}
	}
	else
	{
		Serial.printf( "[SWIO] Failed reading Flash at 0x08000000 (err=%d)\r\n", r_fl );
	}

	uint32_t end_dms = 0;
	MCFReadReg32( &state, DMSTATUS, &end_dms );
	g_swio_active_state = state;
	g_swio_active_pin = pin;
	Serial.printf( "[SWIO] End of Probe DMSTATUS=0x%08X\r\n", (unsigned)end_dms );
	Serial.println( "=== Probe Complete ===\r\n" );
}

static bool swio_flash_chip( int pin )
{
	Serial.printf( "\r\n=== Flashing CH32V003 (%u bytes) via SWIO on GPIO%d ===\r\n", (unsigned)ch32v003_firmware_len, pin );
	if( g_swio_progress_cb ) g_swio_progress_cb( "Connecting SWIO...", 0, (int)ch32v003_firmware_len );
	struct SWIOState state;
	if( !swio_auto_connect( &state, pin ) )
	{
		return false;
	}

	MCFWriteReg32( &state, DMCONTROL, 0x80000001 );
	MCFWriteReg32( &state, DMCONTROL, 0x80000001 );

	Serial.println( "[SWIO] Unlocking & mass-erasing Flash..." );
	if( g_swio_progress_cb ) g_swio_progress_cb( "Erasing Flash...", 0, (int)ch32v003_firmware_len );
	int r_erase = EraseFlash( &state, 0, 0, 1 );
	if( r_erase != 0 )
	{
		Serial.printf( "[SWIO] ERROR: Mass erase failed with code %d\r\n", r_erase );
		g_swio_active_pin = -1;
		return false;
	}

	Serial.println( "[SWIO] Writing & verifying 64-byte blocks at 0x08000000..." );
	if( g_swio_progress_cb ) g_swio_progress_cb( "Writing & verifying...", 0, (int)ch32v003_firmware_len );
	int r_write = WriteBinaryBlob( &state, 0x08000000, ch32v003_firmware_len, ch32v003_firmware_bin );
	if( r_write != 0 )
	{
		Serial.printf( "[SWIO] ERROR: Flash write failed with code %d\r\n", r_write );
		g_swio_active_pin = -1;
		return false;
	}

	Serial.println( "[SWIO] Rebooting CH32V003 into newly flashed firmware..." );
	if( g_swio_progress_cb ) g_swio_progress_cb( "Rebooting CH32...", (int)ch32v003_firmware_len, (int)ch32v003_firmware_len );
	HaltMode( &state, 1 );
	g_swio_active_pin = -1;
	delay( 20 );
	pinMode( pin, INPUT_PULLUP );
	Serial.println( "[SWIO] SUCCESS! CH32V003 flashed and rebooted!" );
	Serial.println( "[SWIO] Press the Power button on the carrier PCB now to turn on CH32V003 output & establish UART handshake.\r\n" );
	return true;
}

static bool swio_flash_auto( swio_progress_cb_t cb, int * out_pin )
{
	g_swio_progress_cb = cb;
	const int candidate_pins[] = { 7, 9 };
	for( size_t i = 0; i < sizeof(candidate_pins)/sizeof(candidate_pins[0]); i++ )
	{
		int pin = candidate_pins[i];
		if( swio_flash_chip( pin ) )
		{
			if( out_pin ) *out_pin = pin;
			g_swio_progress_cb = nullptr;
			return true;
		}
	}
	g_swio_progress_cb = nullptr;
	return false;
}

#endif // _CH32V003_SWIO_H

