
#ifndef  _TYPEDEF_H_
#define  _TYPEDEF_H_

#define	u8		   volatile unsigned char
#define	u16	   volatile unsigned int
#define	u32		volatile unsigned long

#define	uc8		const unsigned char
#define	uc16		const unsigned int
#define	uc32		const unsigned long

#define	BYTE		volatile unsigned char
#define	WORD		volatile unsigned int

enum 	{XRESET=0, XSET=!XRESET};
enum 	{LOW=0, HIGH=!LOW};
enum 	{FALSE=0, TRUE=!FALSE};
enum	{OUTPUT=0, INPUT=!OUTPUT};
enum	{NOPULLUP=0, PULLUP=!NOPULLUP};
#ifndef NULL
  #define	NULL		0
#endif

#define  BIT0 	   0x01 		/**< The value of bit 0 */
#define  BIT1 	   0x02 		/**< The value of bit 1 */
#define  BIT2 	   0x04 		/**< The value of bit 2 */
#define  BIT3 	   0x08 		/**< The value of bit 3 */
#define  BIT4 	   0x10 		/**< The value of bit 4 */
#define  BIT5 	   0x20 		/**< The value of bit 5 */
#define  BIT6 	   0x40 		/**< The value of bit 6 */
#define  BIT7 	   0x80 		/**< The value of bit 7 */
#define  BIT8 	   0x0100 	/**< The value of bit 8 */
#define  BIT9 	   0x0200 	/**< The value of bit 9 */
#define  BIT10    0x0400 	/**< The value of bit 10 */
#define  BIT11    0x0800 	/**< The value of bit 11 */
#define  BIT12    0x1000 	/**< The value of bit 12 */
#define  BIT13    0x2000 	/**< The value of bit 13 */
#define  BIT14    0x4000 	/**< The value of bit 14 */
#define  BIT15    0x8000 	/**< The value of bit 15 */



typedef struct {
	unsigned char b0 : 1;
	unsigned char b1 : 1;
	unsigned char b2 : 1;
	unsigned char b3 : 1;
	unsigned char b4 : 1;
	unsigned char b5 : 1;
	unsigned char b6 : 1;
	unsigned char b7 : 1;
} __bits_t;

typedef struct {
	unsigned int b0 : 1;
	unsigned int b1 : 1;
	unsigned int b2 : 1;
	unsigned int b3 : 1;
	unsigned int b4 : 1;
	unsigned int b5 : 1;
	unsigned int b6 : 1;
	unsigned int b7 : 1;
	unsigned int b8 : 1;
	unsigned int b9 : 1;
	unsigned int b10 : 1;
	unsigned int b11 : 1;
	unsigned int b12 : 1;
	unsigned int b13 : 1;
	unsigned int b14 : 1;
	unsigned int b15 : 1;
} __wbits_t;


typedef union 
{
	volatile unsigned char value;
	volatile __bits_t bits;
} byte_t;


typedef union 
{
   volatile unsigned int value;
	struct 
   {
		volatile unsigned char low;
		volatile unsigned char high;
	} bytes;
	volatile __wbits_t bits;
} word_t;

typedef union 
{
   volatile unsigned long value;
	struct 
   {
		volatile unsigned char b0;
		volatile unsigned char b1;
		volatile unsigned char b2;
		volatile unsigned char b3;
	} bytes;
} long_t;


// mov 	A,m 
#define GCC_MOVA(varname)	asm volatile("mov A,%0":"=m"(varname):"m"(varname))
// mov 	m,A
#define GCC_MOVM(varname)	asm volatile("mov %0,A":"=m"(varname):"m"(varname))
// and 	A,m
#define GCC_ANDA(varname)	asm volatile("and A,%0":"=m"(varname):"m"(varname))
// andm 	A,m
#define GCC_ANDM(varname)	asm volatile("andm A,%0":"=m"(varname):"m"(varname))
// or 	A,m
#define GCC_ORA(varname)	asm volatile("or  A,%0":"=m"(varname):"m"(varname))
// orm 	A,m
#define GCC_ORM(varname)	asm volatile("orm  A,%0":"=m"(varname):"m"(varname))
// orm 	A,m
#define SWAP_M2M(d,s)	\
			asm volatile("swapa	%0":"=m"(s):"m"(s));	\
			asm volatile("mov  	%0,A":"=m"(d):"m"(d))
			
// 
#define ASM_DEC(varname)	asm volatile("dec %0"::"m"(varname))
#define ASM_OFFSETL2A(varname)	asm volatile("mov A,offset %0"::"m"(varname))   
#define ASM_OFFSETH2A(varname)	asm volatile("mov A,(offset %0 >> 8)"::"m"(varname))

#define  Movm2m(d,s) \
         asm volatile("mov A,%0"::"m"(s)); \
         asm volatile("mov %0,A"::"m"(d))

#define  GetOffsetLow(d,s) \
         asm volatile("mov A,low(offset %0)":"=m"(s)); \
         asm volatile("mov %0,A"::"m"(d))

#define  GetOffsetHigh(d,s) \
         asm volatile("mov A,high(offset %0)":"=m"(s)); \
         asm volatile("mov %0,A"::"m"(d))

#define  GetOffset2bpmp1(sa) \
         asm volatile("mov A,high(offset %0)":"=m"(sa)); \
         asm volatile("mov %0,A"::"m"(_bp)); \
         asm volatile("mov A,low(offset %0)":"=m"(sa)); \
         asm volatile("mov %0,A"::"m"(_mp1))

#define  MovMatrixmp0(da,sa,ix) \
         asm volatile("mov A,low(offset %0)":"=m"(sa)); \
         asm volatile("add A,%0"::"m"(ix)); \
         asm volatile("mov %0,A":"=m"(_mp0)); \
         asm volatile("mov A,%0":"=m"(_iar0)); \
         asm volatile("mov %0,A"::"m"(da))

#define  MatrixIndexMP0(ma,ix)\
         asm volatile("mov A,low(offset %0)":"=m"(ma)); \
         asm volatile("add A,%0"::"m"(ix)); \
         asm volatile("mov %0,A":"=m"(_mp0))

#define	WriteMatrix(ma,mx,wd) \
			asm volatile("mov A,high(offset %0)":"=m"(ma)); \
         asm volatile("mov %0,A":"=m"(_bp)); \
         asm volatile("mov A,low(offset %0)":"=m"(ma)); \
         asm volatile("add A,%0":"=m"(mx)); \
         asm volatile("mov %0,A":"=m"(_mp1)); \
         asm volatile("mov A,%0":"=m"(wd)); \
         asm volatile("mov %0,A":"=m"(_iar1))
         
#define	ReadMatrix(ma,mx,rd) \
			asm volatile("mov A,low(offset %0)":"=m"(ma)); \
			asm volatile("add A,%0"::"m"(mx)); \
			asm volatile("mov %0,A":"=m"(_mp1)); \
			asm volatile("mov A,high(offset %0)":"=m"(ma)); \
			asm volatile("mov %0,A":"=m"(_bp)); \
         asm volatile("mov A,%0":"=m"(_iar1)); \
         asm volatile("mov %0,A":"=m"(rd))

#endif // _TYPEDEF_H_

